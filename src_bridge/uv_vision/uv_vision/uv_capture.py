import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from sensor_msgs.msg import Image
import argparse
from datetime import datetime
from uv_vision import stereocam
import os
import sys
from rclpy.utilities import remove_ros_args  # 导入ROS参数过滤工具

cnt_down = 0
cnt_front = 0

def image_publish(frame, img_bridge, image_pub):
    # 转为numpy.array
    frame = np.array(frame)
    # 转换为ros2消息类型，且解码方式为b(blue)、g(green)、r(red)
    data = img_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    # 发布 转换好的 图像类型消息
    image_pub.publish(data)
        # 如果提供了保存路径，则保存图像

class CaptureNode(Node):
    def __init__(self, name, opt):
        super().__init__(name)
        self.bridge = CvBridge()

        self.get_logger().info("大家好，我是%s!" % name)

        if opt.front_cam[0] != "none":
            # 前置摄像头
            self.frontcam_rawleft_pub = self.create_publisher(
                Image, "front_cam/raw/left", 10)
            self.frontcam_rawright_pub = self.create_publisher(
                Image, "front_cam/raw/right", 10)
            self.frontcam_raw_pub = self.create_publisher(
                Image, "front_cam/raw", 10)
            
            if opt.front_params[0] != "none":
                self.frontcam_rectifiedleft_pub = self.create_publisher(
                    Image, "front_cam/rectified/left", 10)
                self.frontcam_rectifiedright_pub = self.create_publisher(
                    Image, "front_cam/rectified/right", 10)
                self.frontcam_rectified_pub = self.create_publisher(
                    Image, "front_cam/rectified", 10)

            self.front_cam_timer = self.create_timer(
                0.01, self.front_cam_timer_callback)

            self.front_sc = stereocam.StereoCamera()
            self.front_sc.device_parameter_config(device=opt.front_cam[0])
            if opt.front_params[0] != "none":
                self.front_sc.cal_parameters_init(opt.front_params[0])
                self.front_sc.rectification_init()

        if opt.down_cam[0] != "none":
            # 后置摄像头
            self.backcam_rawleft_pub = self.create_publisher(
                Image, "down_cam/raw/left", 10)
            self.backcam_rawright_pub = self.create_publisher(
                Image, "down_cam/raw/right", 10)
            self.backcam_raw_pub = self.create_publisher(
                Image, "down_cam/raw", 10)
            
            if opt.down_params[0] != "none":
                self.backcam_rectifiedleft_pub = self.create_publisher(
                    Image, "down_cam/rectified/left", 10)
                self.backcam_rectifiedright_pub = self.create_publisher(
                    Image, "down_cam/rectified/right", 10)
                self.backcam_rectified_pub = self.create_publisher(
                    Image, "down_cam/rectified", 10)
            
            self.down_cam_timer = self.create_timer(
                0.01, self.back_cam_timer_callback)

            self.down_sc = stereocam.StereoCamera()
            self.down_sc.device_parameter_config(device=opt.down_cam[0])
            if opt.down_params[0] != "none":
                self.down_sc.cal_parameters_init(opt.down_params[0])
                self.down_sc.rectification_init()

        self.opt =opt   

    def front_cam_timer_callback(self):

        s, f = self.front_sc.capture()

        if s:
            img_l, img_r = self.front_sc.get_halfimg(f)
            
            image_publish(f, self.bridge, self.frontcam_raw_pub)
            image_publish(img_l, self.bridge, self.frontcam_rawleft_pub)
            image_publish(img_r, self.bridge, self.frontcam_rawright_pub)
            
            if self.opt.down_params[0] != "none":
                r_l,r_r = self.front_sc.rectifyImage(img_l, img_r)
                full = self.front_sc.get_fullimg(r_l,r_r)
                """global cnt_front
                cnt_front = cnt_front+1
                if cnt_front == 2:
                    now = datetime.now()
                    timestamp = now.strftime("%Y%m%d_%H%M%S%f")
                    filename = os.path.join("/home/nvidia/Workspace/Cruise/img/front", f"{timestamp}.jpg")
                    cv2.imwrite(filename, full)
                    cnt_front = 0"""
                image_publish(full, self.bridge, self.frontcam_rectified_pub)
                image_publish(r_l, self.bridge, self.frontcam_rectifiedleft_pub)
                image_publish(r_r, self.bridge, self.frontcam_rectifiedright_pub)

            self.get_logger().info('发布了前视图像')
        else:
            self.get_logger().info('图像获取失败')

    def back_cam_timer_callback(self):

        s, f = self.down_sc.capture()

        if s:
            img_l, img_r = self.down_sc.get_halfimg(f)
            
            image_publish(f, self.bridge, self.backcam_raw_pub)
            image_publish(img_l, self.bridge, self.backcam_rawleft_pub)
            image_publish(img_r, self.bridge, self.backcam_rawright_pub)
            
            if self.opt.down_params[0] != "none":
                r_l,r_r = self.down_sc.rectifyImage(img_l, img_r)
                full = self.down_sc.get_fullimg(r_l,r_r)
                """global cnt_down
                cnt_down = cnt_down+1
                if cnt_down == 2:
                    now = datetime.now()
                    timestamp = now.strftime("%Y%m%d_%H%M%S%f")
                    filename = os.path.join("/home/nvidia/Workspace/Cruise/img/down", f"{timestamp}.jpg")
                    cv2.imwrite(filename, full)
                    cnt_down = 0"""
                image_publish(full, self.bridge, self.backcam_rectified_pub)
                image_publish(r_l, self.bridge, self.backcam_rectifiedleft_pub)
                image_publish(r_r, self.bridge, self.backcam_rectifiedright_pub)

            self.get_logger().info('发布了下视图像')
        else:
            self.get_logger().info('图像获取失败')

 

def main(args=None):
    # 1. 过滤ROS 2自动添加的参数（关键步骤）
    # 移除--ros-args等ROS相关参数，只保留自定义参数
    ros_args = remove_ros_args(sys.argv)
    
    # 2. 解析自定义参数（使用过滤后的参数列表）
    parser = argparse.ArgumentParser()
    parser.add_argument('--front-cam', nargs='+', type=str,
                        default=['/dev/video0'], help='前置摄像头')
    parser.add_argument('--down-cam', nargs='+', type=str,
                        default=['/dev/video2'], help='下置摄像头')
    parser.add_argument('--front-params', nargs='+', type=str,
                        default=['/home/nvidia/Workspace/Cruise/datas/front.npz'], help='前置摄像头参数存储路径')
    parser.add_argument('--down-params', nargs='+', type=str,
                        default=['/home/nvidia/Workspace/Cruise/datas/down.npz'], help='下置摄像头参数存储路径')
    
    # 解析时跳过脚本文件名（ros_args[0]），只处理自定义参数
    opt = parser.parse_args(ros_args[1:])

    # 3. 原有逻辑保持不变
    # 注意：原代码中判断条件使用了opt.back_cam，但未定义该参数，此处保留原逻辑，若实际有back-cam参数需补充
    if opt.front_cam[0] == "none" and opt.down_cam[0] == "none":  # 假设原代码中back_cam是笔误，改为down_cam
        print("No Camera opened")
    else:
        rclpy.init(args=args)
        node = CaptureNode("uv_capture", opt)
        rclpy.spin(node)
        rclpy.shutdown()
        
