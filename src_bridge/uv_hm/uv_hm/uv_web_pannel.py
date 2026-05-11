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

from datetime import datetime

from uv_msgs.msg import CabinState
from uv_msgs.msg import PropellerThrust
from uv_msgs.msg import RobotAxis
from uv_msgs.msg import WorkState

COLOR_RED = (0, 0, 255)


class CaptureNode(Node):
    def __init__(self, name, front_cam, host, height, width):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)

        self.size = (int(height), int(width))
        self.height = int(height)
        self.width = int(width)
        self.address = ("", 0)
        self.letterheight = int(16*self.height/720)
        self.letterzoom = 0.5*self.height/720

        self.cabin_state = CabinState()
        self.propeller_thrust = PropellerThrust()
        self.robot_position = RobotAxis()
        self.work_state = WorkState()
        self.robot_speed = RobotAxis()


        # 创建话题接收 cabin_state ，定义其中的消息类型为 CabinState
        self.create_subscription(
            CabinState, 'cabin_state', self.cabin_state_callback, 10)

        # 创建话题接收 propeller_thrust ，定义其中的消息类型为 PropellerThrust
        self.create_subscription(
            PropellerThrust, 'propeller_thrust', self.propeller_thrust_callback, 10)

        # 创建话题接收 robot_position ，定义其中的消息类型为 RobotAxis
        self.create_subscription(
            RobotAxis, 'robot_position', self.robot_position_callback, 10)

        # 创建话题接收 robot_speed ，定义其中的消息类型为 RobotAxis
        self.create_subscription(
            RobotAxis, 'robot_speed', self.robot_speed_callback, 10)

        # 创建话题发布 openloop_thrust ，定义其中的消息类型为 RobotAxis
        self.openloop_thrust_pub = self.create_publisher(
            RobotAxis, "openloop_thrust", 10)

        # 创建话题发布 servo_control ，定义其中的消息类型为 CabinState
        self.servo_control_pub = self.create_publisher(
            CabinState, "servo_control", 10)

        # 创建话题发布 work_state ，定义其中的消息类型为 WorkState
        self.work_state_pub = self.create_publisher(
            WorkState, "work_state", 10)

    def cabin_state_callback(self, data):
        self.cabin_state = data

    def propeller_thrust_callback(self, data):
        self.propeller_thrust = data

    def robot_position_callback(self, data):
        self.robot_position = data

    def robot_speed_callback(self, data):
        self.robot_speed = data


def controller_callback(node, port):
    openloop_thrust = RobotAxis()
    servo_state = CabinState()
    work_state = WorkState()

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    node.address = (s.getsockname()[0], port)
   
    server_socket.bind(node.address)

    # 获取当前时间，并格式化为文件名
    # current_time = datetime.now()
    # file_name = f"output_{current_time.strftime('%Y%m%d_%H%M%S')}.txt"

    # with open("/home/nvidia/Workspace/Cruise/ctrl_output.txt", 'w') as file:

    while rclpy.ok():
        receive_data, client_address = server_socket.recvfrom(1024)
        data = json.loads(receive_data.decode())

        
        openloop_thrust.x = data["x"]
        openloop_thrust.y = data["y"]
        openloop_thrust.z = data["z"]
        openloop_thrust.rz = data["yaw"]
        openloop_thrust.rx = data["roll"]
        openloop_thrust.ry = data["pitch"]

        # servo_state.servo[0] = data["servo0"]
        # servo_state.servo[1] = data["servo1"]
        
        #work_state.state = data["state"]

        # 将 JSON 对象转换为字符串并写入 .txt 文件
        # json_str = json.dumps(data, indent=None, separators=(',', ':'))
        # file.write(json_str + '\n')  # 每个 JSON 对象后添加换行符

        # node.get_logger().info(json_str)

        node.openloop_thrust_pub.publish(openloop_thrust)
        node.servo_control_pub.publish(servo_state)
        #node.work_state_pub.pubilsh(work_state)
        # node.get_logger().info(str(data))
        
        

def main(args=None):
    # 加载参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--cam', nargs='+', type=str,
                        default=['/dev/video0'], help='前置摄像头')
    parser.add_argument('--host', nargs='+', type=str,
                        default=["127.0.0.1"], help='srs服务器地址')
    parser.add_argument('--height', nargs='+', type=str,
                        default=["1080"], help='图像高度')
    parser.add_argument('--width', nargs='+', type=str,
                        default=["1920"], help='图像宽度')
    parser.add_argument('--port', nargs='+', type=str,
                        default=["10086"], help='远程控制器接入端口')
    opt = parser.parse_args()

    if opt.cam[0] == "none":
        print("No Camera opened")
    else:
        rclpy.init(args=args)  # 初始化rclpy
        node = CaptureNode(
            "uv_web_pannel", opt.cam[0], opt.host[0], opt.height[0], opt.width[0])  # 新建一个节点


        thread_controller = threading.Thread(
            target=controller_callback, args=(node, int(opt.port[0])))
        thread_controller.start()
       

        # 保持节点运行，检测是否收到退出指令（Ctrl+C）
        rclpy.spin(node)
        # 关闭rclpy
        rclpy.shutdown()
