import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from sensor_msgs.msg import Image
import argparse

from uv_vision import stereocam


def image_publish(frame, img_bridge, image_pub):
    # 转为numpy.array
    frame = np.array(frame)
    # 转换为ros2消息类型，且解码方式为b(blue)、g(green)、r(red)
    data = img_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    # 发布 转换好的 图像类型消息
    image_pub.publish(data)


class CaptureNode(Node):
    def __init__(self, name, front_cam, path):
        super().__init__(name)
        self.bridge = CvBridge()

        self.get_logger().info("大家好，我是%s!" % name)

        if front_cam != "none":
            # 前置摄像头
            self.depthmap_pub = self.create_publisher(
                Image, "depthmap", 10)

            self.front_cam_timer = self.create_timer(
                0.01, self.front_cam_timer_callback)

            self.front_sc = stereocam.StereoCamera()
            self.front_sc.cal_parameters_init(path)
            self.front_sc.device_parameter_config(device=front_cam)
            self.front_sc.rectification_init()

    def front_cam_timer_callback(self):

        s, f = self.front_sc.capture()

        if s:
            img_l, img_r = self.front_sc.get_halfimg(f)

            img_l, img_r = self.front_sc.rectifyImage(img_l, img_r)
            
            depth_map = self.front_sc.getdepth(img_l, img_r)
            
            depth,depth_image = self.front_sc.depth2img(depth_map)
            
            depth_image = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)

            image_publish(depth_image,self.bridge,self.depthmap_pub)
            
            self.get_logger().info('发布了图像')
        else:
            self.get_logger().info('图像获取失败')


def main(args=None):
    # 加载参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--front-cam', nargs='+', type=str,
                        default=['none'], help='前置摄像头')
    parser.add_argument('--param-path', nargs='+', type=str,
                        default=['datas/stereo_calib.npz'], help='摄像头参数')
    opt = parser.parse_args()

    if opt.front_cam[0] == "none" :
        print("No Camera opened")
    else:
        rclpy.init(args=args)  # 初始化rclpy
        node = CaptureNode(
            "uv_depthimg", opt.front_cam[0],opt.param_path[0])  # 新建一个节点
        rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
        rclpy.shutdown()  # 关闭rclpy
        
