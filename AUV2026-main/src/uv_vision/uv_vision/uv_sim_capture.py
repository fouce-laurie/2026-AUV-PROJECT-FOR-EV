import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image
import argparse
from datetime import datetime
import os
import sys
from pathlib import Path
from rclpy.utilities import remove_ros_args


FRONT_IMG_DIR = Path('/home/nvidia/New_Workspace/img/front')
DOWN_IMG_DIR = Path('/home/nvidia/New_Workspace/img/down')

class SimCaptureNode(Node):
    def __init__(self, name, opt):
        super().__init__(name)
        self.bridge = CvBridge()
        self.opt = opt
        
        # Make directories if they don't exist
        os.makedirs(FRONT_IMG_DIR, exist_ok=True)
        os.makedirs(DOWN_IMG_DIR, exist_ok=True)

        self.get_logger().info(f"大家好，我是{name}! 正在启动仿真图像保存...")

        self.cnt_front = 0
        self.cnt_down = 0
        
        self.front_save_interval = opt.save_interval
        self.down_save_interval = opt.save_interval

        if self.opt.enable_front:
            self.front_sub = self.create_subscription(
                Image, "front_cam/rectified", self.front_callback, 10)
            self.get_logger().info("已订阅 front_cam/rectified")
            
        if self.opt.enable_down:
            self.down_sub = self.create_subscription(
                Image, "down_cam/rectified", self.down_callback, 10)
            self.get_logger().info("已订阅 down_cam/rectified")

    def front_callback(self, msg):
        self.cnt_front += 1
        if self.cnt_front >= self.front_save_interval:
            self.cnt_front = 0
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                now = datetime.now()
                timestamp = now.strftime("%Y%m%d_%H%M%S%f")
                filename = os.path.join(str(FRONT_IMG_DIR), f"{timestamp}_sim_front.jpg")
                cv2.imwrite(filename, cv_img)
                self.get_logger().info(f'已保存前视仿真图像: {filename}')
            except Exception as e:
                self.get_logger().error(f'保存前视仿真图像失败: {e}')

    def down_callback(self, msg):
        self.cnt_down += 1
        if self.cnt_down >= self.down_save_interval:
            self.cnt_down = 0
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                now = datetime.now()
                timestamp = now.strftime("%Y%m%d_%H%M%S%f")
                filename = os.path.join(str(DOWN_IMG_DIR), f"{timestamp}_sim_down.jpg")
                cv2.imwrite(filename, cv_img)
                self.get_logger().info(f'已保存下视仿真图像: {filename}')
            except Exception as e:
                self.get_logger().error(f'保存下视仿真图像失败: {e}')

def main(args=None):
    ros_args = remove_ros_args(sys.argv)
    
    parser = argparse.ArgumentParser(description="仿真图像保存节点")
    parser.add_argument('--enable-front', action='store_true', default=True, help='启用前视图像保存')
    parser.add_argument('--enable-down', action='store_true', default=True, help='启用下视图像保存')
    parser.add_argument('--save-interval', type=int, default=3, help='保存频率，每隔 N 帧保存一张')
    
    opt = parser.parse_args(ros_args[1:])

    rclpy.init(args=args)
    node = SimCaptureNode("uv_sim_capture", opt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
