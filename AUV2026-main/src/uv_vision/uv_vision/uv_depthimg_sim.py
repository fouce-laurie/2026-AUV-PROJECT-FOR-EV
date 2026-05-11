#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
import struct
import argparse
import os
from pathlib import Path
import message_filters
from uv_vision import stereocam


def image_publish(frame, img_bridge, image_pub, header=None, encoding='bgr8'):
    frame = np.array(frame)
    msg = img_bridge.cv2_to_imgmsg(frame, encoding=encoding)
    if header is not None:
        msg.header = header
    image_pub.publish(msg)


class SimDepthNode(Node):
    def __init__(self, node_name, left_topic, right_topic, depth_topic, depth_raw_topic, calib_path, queue_size=10):
        super().__init__(node_name)
        self.bridge = CvBridge()

        # Stereo algorithm instance (来自 uv_vision/stereocam.py)
        self.sc = stereocam.StereoCamera()
        self.sc.cal_parameters_init(calib_path)
        self.sc.rectification_init()

        # 发布：可视化深度图（兼容旧节点）与原始深度（32FC1）
        self.depth_vis_pub = self.create_publisher(Image, depth_topic, 10)
        self.depth_raw_pub = self.create_publisher(Image, depth_raw_topic, 10)
        # 自动生成的点云话题，基于 raw topic 名称
        pc_topic = depth_raw_topic + '_points'
        self.pc_pub = self.create_publisher(PointCloud2, pc_topic, 10)

        # 尝试使用 message_filters 进行时间同步；若失败，退回到简单缓存+定时器
        try:
            self.sub_l = message_filters.Subscriber(self, Image, left_topic)
            self.sub_r = message_filters.Subscriber(self, Image, right_topic)
            self.sync = message_filters.ApproximateTimeSynchronizer([self.sub_l, self.sub_r], queue_size, 0.05)
            self.sync.registerCallback(self.stereo_callback)
            self.get_logger().info(f"使用 message_filters 订阅: {left_topic} & {right_topic}")
        except Exception as e:
            self.get_logger().warning(f"message_filters 同步不可用或失败，回退到缓存方法: {e}")
            self.left_img = None
            self.right_img = None
            self.create_subscription(Image, left_topic, self.left_cb, 10)
            self.create_subscription(Image, right_topic, self.right_cb, 10)
            self.check_timer = self.create_timer(0.05, self.check_and_process)

    def left_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            img = self.bridge.imgmsg_to_cv2(msg)
        self.left_img = (img, msg.header)

    def right_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            img = self.bridge.imgmsg_to_cv2(msg)
        self.right_img = (img, msg.header)

    def check_and_process(self):
        if getattr(self, 'left_img', None) and getattr(self, 'right_img', None):
            left_img, left_header = self.left_img
            right_img, right_header = self.right_img
            # 简单时间差校验
            t_left = left_header.stamp.sec + left_header.stamp.nanosec / 1e9
            t_right = right_header.stamp.sec + right_header.stamp.nanosec / 1e9
            if abs(t_left - t_right) < 0.1:
                self.process_pair(left_img, right_img, left_header)
                self.left_img = None
                self.right_img = None

    def stereo_callback(self, left_msg, right_msg):
        try:
            left_cv = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
        except Exception:
            left_cv = self.bridge.imgmsg_to_cv2(left_msg)
        try:
            right_cv = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')
        except Exception:
            right_cv = self.bridge.imgmsg_to_cv2(right_msg)
        self.process_pair(left_cv, right_cv, left_msg.header)

    def process_pair(self, left_cv, right_cv, header):
        # 如果可用则做重映射/校正
        try:
            r_l, r_r = self.sc.rectifyImage(left_cv, right_cv)
        except Exception:
            r_l, r_r = left_cv, right_cv

        # 深度计算（与原 uv_depthimg 算法一致）
        depth_map = self.sc.getdepth(r_l, r_r)

        # 发布原始深度（32FC1）
        try:
            raw_msg = self.bridge.cv2_to_imgmsg(depth_map.astype(np.float32), encoding='32FC1')
            raw_msg.header = header
            self.depth_raw_pub.publish(raw_msg)
        except Exception as e:
            self.get_logger().error(f"发布原始深度失败: {e}")

        # 同时把 depth_map 转为 PointCloud2 并发布
        try:
            # depth_map: HxW depth (meters)
            depth = depth_map.astype(np.float32)
            h, w = depth.shape
            cam = self.sc.camera_matrix_left
            fx = float(cam[0, 0])
            fy = float(cam[1, 1])
            cx = float(cam[0, 2])
            cy = float(cam[1, 2])

            # 生成像素坐标网格
            u = np.arange(w)
            v = np.arange(h)
            uu, vv = np.meshgrid(u, v)

            # 仅保留有效深度点
            mask = np.isfinite(depth) & (depth > 0)
            if mask.any():
                z = depth[mask]
                x = (uu[mask].astype(np.float32) - cx) * z / fx
                y = (vv[mask].astype(np.float32) - cy) * z / fy

                points = np.vstack((x, y, z)).T.astype(np.float32)
                # 构造 PointCloud2
                header_pc = header
                pc_msg = PointCloud2()
                pc_msg.header = header_pc
                pc_msg.height = 1
                pc_msg.width = points.shape[0]
                pc_msg.is_dense = False
                pc_msg.is_bigendian = False

                # fields: x, y, z as FLOAT32
                pc_msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                ]
                pc_msg.point_step = 12
                pc_msg.row_step = pc_msg.point_step * pc_msg.width
                pc_msg.data = points.tobytes()

            else:
                # 空点云
                pc_msg = PointCloud2()
                pc_msg.header = header
                pc_msg.height = 1
                pc_msg.width = 0
                pc_msg.is_dense = False
                pc_msg.is_bigendian = False
                pc_msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                ]
                pc_msg.point_step = 12
                pc_msg.row_step = 0
                pc_msg.data = b''

            self.pc_pub.publish(pc_msg)
        except Exception as e:
            self.get_logger().error(f"发布点云失败: {e}")

        # 发布可视化深度（兼容旧逻辑）
        try:
            _, depth_vis = self.sc.depth2img(depth_map)
            depth_vis = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)
            vis_msg = self.bridge.cv2_to_imgmsg(depth_vis, encoding='bgr8')
            vis_msg.header = header
            self.depth_vis_pub.publish(vis_msg)
        except Exception as e:
            self.get_logger().error(f"发布可视化深度失败: {e}")


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--left-topic', nargs='+', type=str, default=['/sim/front_cam/left/image_color'])
    parser.add_argument('--right-topic', nargs='+', type=str, default=['/sim/front_cam/right/image_color'])
    parser.add_argument('--param-path', nargs='+', type=str, default=['datas/stereo_calib.npz'])
    parser.add_argument('--depth-topic', nargs='+', type=str, default=['depthmap'])
    parser.add_argument('--depth-raw-topic', nargs='+', type=str, default=['depthmap_raw'])
    opt = parser.parse_args()

    left_topic = opt.left_topic[0]
    right_topic = opt.right_topic[0]
    calib = opt.param_path[0]

    # 如果提供的标定文件不存在，尝试从常见位置回退查找（环境变量或 Cruise/datas）
    if not os.path.exists(calib):
        candidates = []
        uuv_datas = os.environ.get('UUV_DATAS_DIR')
        if uuv_datas:
            candidates.append(os.path.join(uuv_datas, os.path.basename(calib)))
            candidates.append(os.path.join(uuv_datas, 'front.npz'))
            candidates.append(os.path.join(uuv_datas, 'down.npz'))

        # 猜测 Cruise 根目录（此文件位置为 Cruise/src/uv_vision/uv_vision）
        try:
            workspace_root = Path(__file__).resolve().parents[2]
            candidates.append(str(workspace_root / 'src' / 'datas' / os.path.basename(calib)))
            candidates.append(str(workspace_root / 'src' / 'datas' / 'front.npz'))
            candidates.append(str(workspace_root / 'src' / 'datas' / 'down.npz'))
        except Exception:
            pass

        found = None
        for c in candidates:
            if c and os.path.exists(c):
                found = c
                break

        if found:
            calib = found
            print(f"使用回退标定文件: {calib}")
        else:
            print(f"标定文件未找到: {opt.param_path[0]}\n可用的候选路径: {candidates}\n请通过 --param-path 指定正确的标定文件（例如 --param-path Cruise/datas/front.npz）。")
            sys.exit(1)
    depth_topic = opt.depth_topic[0]
    depth_raw_topic = opt.depth_raw_topic[0]

    rclpy.init(args=args)
    node = SimDepthNode('uv_depthimg_sim', left_topic, right_topic, depth_topic, depth_raw_topic, calib)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
