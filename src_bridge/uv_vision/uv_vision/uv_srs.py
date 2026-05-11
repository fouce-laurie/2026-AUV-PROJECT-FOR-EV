import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from sensor_msgs.msg import Image
import argparse
import subprocess


class CaptureNode(Node):
    def __init__(self, name, srs_path):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        self.path = srs_path

    def run(self):
        subprocess.run(["bash " + self.path], shell=True)


def main(args=None):
    # 加载参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', nargs='+', type=str,
                        default=['~/Workspace/Rosws/scripts/run/srs_start.sh'], help='启动文件路径')
    opt = parser.parse_args()

    # # ./objs/srs -c conf/rtc2rtmp.conf

    rclpy.init(args=args)  # 初始化rclpy
    node = CaptureNode(
        "uv_srsserver", opt.path[0])  # 新建一个节点

    node.run()

    # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.spin(node)
    # 关闭rclpy
    rclpy.shutdown()
