import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from sensor_msgs.msg import Image
import argparse
import subprocess
import socket
import json
import threading




class CaptureNode(Node):
    def __init__(self, name, opt):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)

        self.size = (int(opt.width[0]),int(opt.height[0]) )
        self.bridge = CvBridge()

        # 创建一个话题接收指定图像信息
        self.create_subscription(
            Image, opt.topic[0], self.img_rec_callback, 10)

        # 设置推流的参数
        self.web_command = ['ffmpeg',
                            '-y',
                            '-f', 'rawvideo',
                            '-vcodec', 'rawvideo',
                            '-pix_fmt', 'bgr24',
                            '-s', opt.width[0]+'*' +
                            opt.height[0],  # 根据输入视频尺寸填写
                            '-r', '25',
                            '-i', '-',
                            '-c:v', 'h264',
                            '-pix_fmt', 'yuv420p',
                            '-preset', 'ultrafast',
                            "-tune", "zerolatency",
                            "-start_time_realtime", "0",
                            '-f', 'flv',
                            "rtmp://"+opt.host[0]+"/"+opt.suffix[0]
                            ]
        # 创建、管理子进程
        self.web_pipe = subprocess.Popen(
            self.web_command, stdin=subprocess.PIPE)

    def img_rec_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

            frame_compressed = cv2.resize(
                frame, self.size, interpolation=cv2.INTER_AREA)

            # 读取尺寸、推流
            img = cv2.resize(frame_compressed, self.size)
            self.web_pipe.stdin.write(img.tobytes())

        except:
            self.get_logger().info('图像获取失败')



def main(args=None):
    # 加载参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', nargs='+', type=str,
                        default=['none'], help='订阅话题')
    parser.add_argument('--host', nargs='+', type=str,
                        default=["192.168.16.108"], help='srs服务器地址')
    parser.add_argument('--suffix', nargs='+', type=str,
                        default=['/cam/front'], help='url后缀')
    parser.add_argument('--height', nargs='+', type=str,
                        default=["360"], help='图像高度')
    parser.add_argument('--width', nargs='+', type=str,
                        default=["640"], help='图像宽度')
    opt = parser.parse_args()

    if opt.topic[0] == "none":
        print("...")
    else:
        rclpy.init(args=args)  # 初始化rclpy
        node = CaptureNode("uv_virtualwebcam", opt)  # 新建一个节点

        # 保持节点运行，检测是否收到退出指令（Ctrl+C）
        rclpy.spin(node)
        # 关闭rclpy
        rclpy.shutdown()
