import rclpy
from rclpy.node import Node
import time
import os
import termios
import struct
import threading
import json
import argparse; # 导入argparse模块，用于解析命令行参数
from uv_control_py import Pid
from uv_control_py.CoordinateSystem import CoordinateSystems, MotionState, Cs_Back, Cs_Move, AngleCorrect

from uv_msgs.msg import RobotDeviceManager  # 机器人设备管理器
from uv_msgs.msg import RobotMotionController  # 机器人运动控制器

from uv_msgs.msg import PidControllersState
from uv_msgs.msg import PidParams#巡线的pid参数

from uv_msgs.msg import RobotAxis
from uv_msgs.msg import ServoSet
from uv_msgs.msg import TargetPosDown
from uv_msgs.msg import Yolov8
from uv_msgs.msg import LedControllers
from uv_msgs.msg import MagnetController

from sensor_msgs.msg import Image
from uv_msgs.srv import DetectRequest
import cv2
import numpy as np
from cv_bridge import CvBridge
from filterpy.kalman import KalmanFilter


from math import sin, cos, atan2, sqrt

PI = 3.141592653589793
DEG2RAD = PI/180
RAD2DEG = 180/PI

Step = {  # 领航点拖曳速度(10hz)
    "x": 0.01,
    "y": 0.01,
    "z": 0.05,
    "rz": 5.0
}

AllowedError = {  # 位置容许误差
    "x": 0.02,
    "y": 0.02,
    "z": 0.02,
    "rz": 1.3
}


class CoreNode(Node):
    def __init__(self, name, opt):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        self.opt = opt

        self.robot = CoordinateSystems()

        self.MotionController = RobotMotionController()
        #pid参数初始化
        self.pid_parameters = PidParams()
        # self.pid_parameters.p = 0.0
        # self.pid_parameters.i = 0.0
        # self.pid_parameters.d = 0.0

        self.previous_error = 0.0
        self.i_error = 0.0
        self.dt = 1

        self.task_lock = True
        self.yolov8_data_down = Yolov8()
        self.yolov8_data_front = Yolov8()
        self.magnet = MagnetController()

        self.target = {
            "name": "none",
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        }

        self.backpoint = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "rz": 0.0
        }

        self.front_cam = CoordinateSystems()
        self.down_cam = CoordinateSystems()
        #前视相机坐标偏置
        self.front_cam.base.vector.x = 107.6/1000
        self.front_cam.base.vector.y = 320.0/1000
        self.front_cam.base.vector.z = 65.0/1000
        self.front_cam.base.vector.rx = -90.0
        self.front_cam.base.extract()
        #下视相机偏置
        self.down_cam.base.vector.x = 0.0/1000
        self.down_cam.base.vector.y = 170.0/1000
        self.down_cam.base.vector.z = 42.7/1000
        self.down_cam.base.vector.rz = 0.0
        self.down_cam.base.extract()

        self.get_logger().info(
            f"前置摄像机偏置: x: {self.front_cam.base.vector.x:.2f} y: { self.front_cam.base.vector.y : .2f} z: {self.front_cam.base.vector.z : .2f}")
        self.get_logger().info(
            f"          : rx: {self.front_cam.base.vector.rx:.2f} ry: { self.front_cam.base.vector.ry : .2f} rz: {self.front_cam.base.vector.rz : .2f}")
        self.get_logger().info(
            f"下置摄像机偏置: x: {self.down_cam.base.vector.x:.2f} y: { self.down_cam.base.vector.y : .2f} z: {self.down_cam.base.vector.z : .2f}")
        self.get_logger().info(
            f"          : rx: {self.down_cam.base.vector.rx:.2f} ry: { self.down_cam.base.vector.ry : .2f} rz: {self.down_cam.base.vector.rz : .2f}")
        
        self.start_pos = CoordinateSystems()
        
        self.front_cam_Image_data = None
        self.down_cam_Image_data = None
        self.front_cam_left_Image_data = None
        self.segment_img = None

        # 话题发布
        # 创建话题发布 target_pos_down ，定义其中的消息类型为 TargetPosDown
        self.target_pos_down_pub = self.create_publisher(
            TargetPosDown, "target_pos_down", 10)
        # 创建话题发布 servo_control ，定义其中的消息类型为 ServoSet
        self.servo_control_pub = self.create_publisher(
            ServoSet, "servo_control", 10)
        #创建话题发布  led_controllers, 定义其中的消息类型为 LedControllers
        self.led_controllers_pub = self.create_publisher(
            LedControllers, 'led_controllers', 10)
        # 创建话题发布 magnet_controller ，定义其中的消息类型为 MagnetController
        self.magnet_controller_pub = self.create_publisher(
            MagnetController, "magnet_controller", 10)
        # 创建话题发布 pid_controllers_set ，定义其中的消息类型为 PidControllersState
        self.pid_controllers_set_pub = self.create_publisher(
            PidControllersState, 'pid_controllers_set', 10)
        #创建话题发布 line_patrol_img , 定义消息类型为Image
        self.line_patrol_img_pub = self.create_publisher(
            Image, 'line_patrol_img', 10)

        self.openthrust_data_pub = self.create_publisher(
            RobotAxis, 'openloop_thrust', 10)

        # 话题接收
        # 创建话题接收 motion_controller ，定义其中的消息类型为 RobotMotionController
        self.create_subscription(
            RobotMotionController, 'motion_controller', self.motion_controller_callback, 10)
        # 创建话题接收 front_cam/rectified ，定义其中的消息类型为 Image
        self.create_subscription(
            Image, opt.front_topic[0], self.front_cam_callback, 10)
        # 创建话题接收 down_cam/rectified ，定义其中的消息类型为 Image
        self.create_subscription(
            Image, opt.down_topic[0], self.down_cam_callback, 10)
        #创建话题接收 front_cam/rectified/left,定义其中的消息类型为 Image
        self.create_subscription(
            Image, 'front_cam/rectified/left', self.front_cam_left_callback, 10)
        self.create_subscription(
            PidParams, 'track_pid_parameter', self.track_pid_parameter_callback, 10)
        # 创建话题接收 uv_detect ，定义其中的消息类型为 Yolov8
        self.create_subscription(
            Yolov8, 'uv_detect_down', self.yolov8_down_callback, 10)
        
        self.create_subscription(
            Yolov8, 'uv_detect_front', self.yolov8_front_callback, 10)
        
        #创建话题接收 binary_segment_img , 定义消息类型为Image，用于接收二值化后的分割结果
        self.create_subscription(
            Image, 'binary_segment_img', self.segment_img_callback, 10)

        # 服务请求
        # 创建服务请求 detect_request_client ，定义其中的消息类型为 RobotAxis , 请求服务 uv_detect_srv
        self.detect_request_client = self.create_client(
            DetectRequest, "uv_detect_srv")

        self.magnet.state = 1
        self.magnet_controller_pub.publish(self.magnet)

        self.get_logger().info("节点初始化完成")

    def track_pid_parameter_callback(self, data):
        pass
        # self.pid_parameters = data
    
    # 更新摄像头图像
    def front_cam_callback(self, data):
        self.front_cam_Image_data = data

    def down_cam_callback(self, data):
        self.down_cam_Image_data = data

    def front_cam_left_callback(self, data):
        self.front_cam_left_Image_data = data
    # 更新巡线图像
    def segment_img_callback(self, data):
        self.segment_img = data

    #更新目标检测结果
    def yolov8_down_callback(self, data):
        self.yolov8_data_down = data
    
    def yolov8_front_callback(self, data):
        self.yolov8_data_front = data
    
    # 更新机器人位置数据
    def motion_controller_callback(self, data):
        self.MotionController = data

        self.robot.base.vector.x = self.MotionController.pos.x
        self.robot.base.vector.y = self.MotionController.pos.y
        self.robot.base.vector.z = self.MotionController.pos.z
        self.robot.base.vector.rx = self.MotionController.pos.rx
        self.robot.base.vector.ry = self.MotionController.pos.ry
        self.robot.base.vector.rz = self.MotionController.pos.rz
        self.robot.base.extract()

    # 等待移动至指定位置
    def move_wait(self):
        cnt = 0
        time.sleep(0.5)
        while True:
            judge = 0
            if abs(self.MotionController.tpos_inbase.x) > AllowedError["x"]:
                judge += 1
            if abs(self.MotionController.tpos_inbase.y) > AllowedError["y"]:
                judge += 1
            if abs(self.MotionController.tpos_inbase.z) > AllowedError["z"]:
                judge += 1
            if abs(self.MotionController.tpos_inbase.rz) > AllowedError["rz"]:
                judge += 1

            if judge == 0:  # 进入容许范围
                return True

            if cnt > 200:  # 超时退出
                return False

            cnt += 1
            time.sleep(0.05)

    def movez(self, z):
         # 校验
        if self.robot.base.vector.z + z < -1.0:
            self.get_logger().info("Warn: 深度设置超出范围")
            z = -self.robot.base.vector.z

        self.get_logger().info("======开始移动======")

        cmd = TargetPosDown()
        cmd.cs = 2
        cmd.pos.rx = 0.0
        cmd.pos.ry = 0.0

        # 调整深度
        step_cnt = int(abs(z)/Step["z"]) + 1
        #self.get_logger().info(f"Info: 开始调整深度, 需要调整 {step_cnt:d} 次")
        cnt = 0
        while True:
            cnt += 1
            self.get_logger().info(f"Info: 开始第 {cnt:d} 次调整深度")
            if abs(z) <= Step["z"]:
                self.get_logger().info("Info: 最终深度调整开始")
                cmd.pos.z = z
                cmd.pos.x = cmd.pos.y = cmd.pos.rz = 0.0
                self.target_pos_down_pub.publish(cmd)
                re = self.move_wait()
                if re:
                    self.get_logger().info("Info: 最终深度调整完成")
                else:
                    self.get_logger().info("Warn: 最终深度调整超时！")
                break
            else:
                if z > 0:
                    z -= Step["z"]
                    cmd.pos.z = Step["z"]
                else:
                    z += Step["z"]
                    cmd.pos.z = -Step["z"]
                cmd.pos.x = cmd.pos.y = cmd.pos.rz = 0.0
                self.target_pos_down_pub.publish(cmd)
                #self.get_logger().info(f"Info: 第 {cnt:d} 次深度调整完成")
                time.sleep(0.2)

        self.get_logger().info("======移动结束======")
    
        # 移动 z 的相对位移
    def fast_movez(self, z):
        # 直接调整
        if self.robot.base.vector.z + z < 0:
            self.get_logger().info("Warn: 深度设置超出范围")
        else:
            self.get_logger().info("======开始移动======")
            cmd = TargetPosDown()
            cmd.cs = 2
            cmd.pos.rx = 0.0
            cmd.pos.ry = 0.0
            self.get_logger().info("Info: 深度快速微调开始")
            cmd.pos.z = z
            cmd.pos.x = cmd.pos.y = cmd.pos.rz = 0.0
            self.target_pos_down_pub.publish(cmd)
            self.get_logger().info("======移动结束======")

        # 渐进调整
        # # 校验
        # if self.robot.base.vector.z + z < 0.2:
        #     self.get_logger().info("Warn: 深度设置超出范围")
        # else:
        #     self.get_logger().info("======开始移动======")

        #     cmd = TargetPosDown()
        #     cmd.cs = 2
        #     cmd.pos.rx = 0.0
        #     cmd.pos.ry = 0.0

        #     # 调整深度
        #     step_cnt = int(abs(z)/Step["z"]) + 1
        #     self.get_logger().info(f"Info: 开始调整深度, 需要调整 {step_cnt:d} 次")
        #     cnt = 0
        #     while True:
        #         cnt += 1
        #         self.get_logger().info(f"Info: 开始第 {cnt:d} 次调整深度")
        #         if abs(z) <= Step["z"]:
        #             self.get_logger().info("Info: 最终深度调整开始")
        #             cmd.pos.z = z
        #             cmd.pos.x = cmd.pos.y = cmd.pos.rz = 0.0
        #             self.target_pos_down_pub.publish(cmd)
        #             re = self.move_wait()
        #             if re:
        #                 self.get_logger().info("Info: 最终深度调整完成")
        #             else:
        #                 self.get_logger().info("Warn: 最终深度调整超时！")
        #             break
        #         else:
        #             if z > 0:
        #                 z -= Step["z"]
        #                 cmd.pos.z = Step["z"]
        #             else:
        #                 z += Step["z"]
        #                 cmd.pos.z = -Step["z"]
        #             cmd.pos.x = cmd.pos.y = cmd.pos.rz = 0.0
        #             self.target_pos_down_pub.publish(cmd)
        #             self.get_logger().info(f"Info: 第 {cnt:d} 次深度调整完成")
        #             time.sleep(0.2)

        #     self.get_logger().info("======移动结束======")

    # 移动 x 的相对位移
    def movex(self, x):
        if x > 10:
            self.get_logger().info("Warn: 横向位置设置超出范围！")
        else:
            self.get_logger().info("======开始移动======")

            cmd = TargetPosDown()
            cmd.cs = 2
            cmd.pos.rx = 0.0
            cmd.pos.ry = 0.0

            # 调整横向位置
            step_cnt = int(abs(x)/Step["x"]) + 1
            self.get_logger().info(f"Info: 开始调整横向位置, 需要调整 {step_cnt:d} 次")
            cnt = 0
            while True:
                cnt += 1
                #self.get_logger().info(f"Info: 开始第 {cnt:d} 次调整横向位置")
                if abs(x) <= Step["x"]:
                    self.get_logger().info("Info: 最终横向位置调整开始")
                    cmd.pos.x = x
                    cmd.pos.y = cmd.pos.z = cmd.pos.rz = 0.0
                    self.target_pos_down_pub.publish(cmd)
                    s = self.move_wait()
                    if s:
                        self.get_logger().info("Info: 最终横向位置调整完成")
                    else:
                        self.get_logger().info("Warn: 最终横向位置调整超时！")
                    break
                else:
                    if x > 0:
                        x -= Step["x"]
                        cmd.pos.x = Step["x"]
                    else:
                        x += Step["x"]
                        cmd.pos.x = -Step["x"]
                    cmd.pos.y = cmd.pos.z = cmd.pos.rz = 0.0
                    self.target_pos_down_pub.publish(cmd)
                    #self.get_logger().info(f"Info: 第 {cnt:d} 次横向位置调整完成")
                    time.sleep(0.1)
            self.get_logger().info("======移动结束======")

    # 移动 x 的相对位移
    def fast_movex(self, x):
        if x > 10:
            self.get_logger().info("Warn: 横向位置设置超出范围！")
        else:
            self.get_logger().info("======开始移动======")

            cmd = TargetPosDown()
            cmd.cs = 2
            cmd.pos.rx = 0.0
            cmd.pos.ry = 0.0
            self.get_logger().info("Info: 横向位置快速微调开始")
            cmd.pos.x = x
            cmd.pos.y = cmd.pos.z = cmd.pos.rz = 0.0
            self.target_pos_down_pub.publish(cmd)
            self.get_logger().info("======移动结束======")
    


    # 移动 y 的相对位移
    def movey(self, y):
        if y > 10:
            self.get_logger().info("Warn: 前后位置设置超出范围！")
        else:
            self.get_logger().info("======开始移动======")

            cmd = TargetPosDown()
            cmd.cs = 2
            cmd.pos.rx = 0.0
            cmd.pos.ry = 0.0

            # 调整
            step_cnt = int(abs(y)/Step["y"]) + 1
            self.get_logger().info(f"Info: 开始调整前后位置, 需要调整 {step_cnt:d} 次")
            cnt = 0
            while True:
                cnt += 1
                #self.get_logger().info(f"Info: 开始第 {cnt:d} 次调整前后位置")
                if abs(y) <= Step["y"]:
                    self.get_logger().info("Info: 最终前后位置调整开始")
                    cmd.pos.y = y
                    cmd.pos.x = cmd.pos.z = cmd.pos.rz = 0.0
                    self.target_pos_down_pub.publish(cmd)
                    s = self.move_wait()
                    if s:
                        self.get_logger().info("Info: 最终前后位置调整完成")
                    else:
                        self.get_logger().info("Warn: 最终前后位置调整超时！")
                    break
                else:
                    if y > 0:
                        y -= Step["y"]
                        cmd.pos.y = Step["y"]
                    else:
                        y += Step["y"]
                        cmd.pos.y = -Step["y"]
                    cmd.pos.x = cmd.pos.z = cmd.pos.rz = 0.0
                    self.target_pos_down_pub.publish(cmd)
                    #self.get_logger().info(f"Info: 第 {cnt:d} 次前后位置调整完成")
                    time.sleep(0.1)
            self.get_logger().info("======移动结束======")

                # 移动 y 的相对位移
    def fast_movey(self, y):
        if y > 10:
            self.get_logger().info("Warn: 前后位置设置超出范围！")
        else:
            self.get_logger().info("======开始移动======")

            cmd = TargetPosDown()
            cmd.cs = 2
            cmd.pos.rx = 0.0
            cmd.pos.ry = 0.0
            self.get_logger().info("Info: 前后位置快速微整开始")
            cmd.pos.y = y
            cmd.pos.x = cmd.pos.z = cmd.pos.rz = 0.0
            self.target_pos_down_pub.publish(cmd)
            self.get_logger().info("======移动结束======")

    # 移动 rz 的相对位移
    def moverz(self, rz):
        if rz > 180 or rz < -180:
            self.get_logger().info("Warn: 角度设置超出范围！")
            rz = AngleCorrect(rz)
            self.get_logger().info(f"Info: 角度等效为 {rz:.2f}°")

        self.get_logger().info("======开始旋转======")

        cmd = TargetPosDown()
        cmd.cs = 2
        cmd.pos.rx = 0.0
        cmd.pos.ry = 0.0

        # 调整
        step_cnt = int(abs(rz)/Step["rz"]) + 1
        self.get_logger().info(f"Info: 开始调整角度, 需要调整 {step_cnt:d} 次")
        cnt = 0
        while True:
            cnt += 1
            #self.get_logger().info(f"Info: 开始第 {cnt:d} 次调整角度")
            if abs(rz) <= Step["rz"]:
                self.get_logger().info("Info: 最终角度调整开始")
                cmd.pos.rz = rz
                cmd.pos.x = cmd.pos.z = cmd.pos.y = 0.0
                self.target_pos_down_pub.publish(cmd)
                s = self.move_wait()
                if s:
                    self.get_logger().info("Info: 最终角度调整完成")
                else:
                    self.get_logger().info("Warn: 最终角度调整超时！")
                break
            else:
                if rz > 0:
                    rz -= Step["rz"]
                    cmd.pos.rz = Step["rz"]
                else:
                    rz += Step["rz"]
                    cmd.pos.rz = -Step["rz"]
                cmd.pos.x = cmd.pos.z = cmd.pos.y = 0.0
                self.target_pos_down_pub.publish(cmd)
                #self.get_logger().info(f"Info: 第 {cnt:d} 次角度调整完成")
                time.sleep(0.1)
        self.get_logger().info("======旋转结束======")
        
        
    def fast_moverz(self, rz):
        if rz > 180 or rz < -180:
            self.get_logger().info("Warn: 角度设置超出范围！")
            rz = AngleCorrect(rz)
            self.get_logger().info(f"Info: 角度等效为 {rz:.2f}°")

        self.get_logger().info("======开始旋转======")

        cmd = TargetPosDown()
        cmd.cs = 2
        cmd.pos.rx = 0.0
        cmd.pos.ry = 0.0

        self.get_logger().info("Info: 前后位置快速微整开始")
        cmd.pos.rz = rz
        cmd.pos.x = cmd.pos.z = cmd.pos.y = 0.0
        self.target_pos_down_pub.publish(cmd)
        self.get_logger().info("======移动结束======")

    # 移动x,y相对位移
    def movexy(self, x, y):
        if x == 0 and y == 0:
            self.get_logger().info("Warn: 未设置合法位移！")
        else:
            rz = atan2(y, x)*RAD2DEG - 90
            d = sqrt(x*x + y*y)
            self.get_logger().info(f"Info: ======转向目标点,旋转{rz:.2f}°======")
            self.moverz(rz)
            self.get_logger().info("Info: ======转向结束======")
            time.sleep(0.1)
            self.get_logger().info(f"Info: ======移动指定距离,移动{d:.2f}m======")
            self.movey(d)
            self.get_logger().info("Info: ======移动结束======")
            time.sleep(0.1)
            #self.get_logger().info(f"Info: ======朝向回正,旋转{-rz:.2f}°======")
            #self.moverz(-rz)
            #self.get_logger().info("Info: ======转向结束======")

    # 移动x,y,z相对位移
    def movexyz(self, x, y, z):
        if z == 0:
            self.get_logger().info("Warn: 未设置合法深度位移！")
        else:
            self.movez(z)
        time.sleep(0.1)
        self.movexy(x, y)

    # 设置 z
    def setz(self, z):
        if z >= -1.0:
            self.get_logger().info("======开始调整深度======")
            cmd = TargetPosDown()
            cmd.cs = 0
            cmd.pos.rx = self.MotionController.pos.rx
            cmd.pos.ry = self.MotionController.pos.ry
            cmd.pos.x = self.MotionController.pos.x
            cmd.pos.y = self.MotionController.pos.y
            cmd.pos.rz = self.MotionController.pos.rz
            cmd.pos.z = z
            self.target_pos_down_pub.publish(cmd)
            s = self.move_wait()
            if s:
                self.get_logger().info("Info: 深度调整完成")
            else:
                self.get_logger().info("Warn: 深度调整超时！")
            self.get_logger().info("======移动结束======")
        else:
            self.get_logger().info("Warn: 深度设置错误！")

    # 设置rz
    def setrz(self, rz):
        if rz > 180 or rz < -180:
            self.get_logger().info("Warn: 角度设置超出范围！")
            rz = AngleCorrect(rz)
            self.get_logger().info(f"Info: 角度等效为 {rz:.2f}°")

        self.get_logger().info("======开始调整角度======")
        cmd = TargetPosDown()
        cmd.cs = 0
        cmd.pos.rx = self.MotionController.pos.rx
        cmd.pos.ry = self.MotionController.pos.ry
        cmd.pos.x = self.MotionController.pos.x
        cmd.pos.y = self.MotionController.pos.y
        cmd.pos.z = self.MotionController.pos.z
        cmd.pos.rz = rz
        self.target_pos_down_pub.publish(cmd)
        s = self.move_wait()
        if s:
            self.get_logger().info("Info: 角度调整完成")
        else:
            self.get_logger().info("Warn: 角度调整超时！")
        self.get_logger().info("======移动结束======")

    # 设置路径点
    def setp(self):
        self.backpoint["x"] = self.MotionController.pos.x
        self.backpoint["y"] = self.MotionController.pos.y
        self.backpoint["z"] = self.MotionController.pos.z
        self.backpoint["rz"] = self.MotionController.pos.rz
        self.get_logger().info(
            f'Info:已保存当前位置 x: {self.backpoint["x"]:.2f} y: {self.backpoint["y"]:.2f} z: {self.backpoint["z"]:.2f} rz : {self.backpoint["rz"]:.2f}')

    # 回到路径点
    def back(self):
        self.robot.target_inworld.vector.x = self.backpoint["x"]
        self.robot.target_inworld.vector.y = self.backpoint["y"]
        self.robot.target_inworld.vector.z = self.backpoint["z"]
        self.robot.target_inworld.vector.rz = self.backpoint["rz"]
        self.robot.world2base()

        x = self.robot.target_inbase.vector.x
        y = self.robot.target_inbase.vector.y
        z = self.robot.target_inbase.vector.z
        rz = self.robot.target_inbase.vector.rz

        self.get_logger().info(
            f"Info:需要调整的位移 x: {x:.2f} y: {y:.2f} z: {z:.2f} rz : {rz:.2f}")

        if z == 0:
            self.get_logger().info("Warn: 深度位移非法！")
        else:
            self.movez(z)
        time.sleep(0.1)

        if x == 0 and y == 0:
            self.get_logger().info("Warn: 非法位移！")
            self.get_logger().info(f"Info: ======调整姿态,旋转{rz_:.2f}°======")
            self.moverz(rz)
            self.get_logger().info("Info: ======转向结束======")
        else:
            rz_ = atan2(y, x)*RAD2DEG - 270
            d = sqrt(x*x + y*y)
            self.get_logger().info(f"Info: ======转向目标点,旋转{rz_:.2f}°======")
            self.moverz(rz_)
            self.get_logger().info("Info: ======转向结束======")
            time.sleep(0.1)
            self.get_logger().info(f"Info: ======移动指定距离,移动{rz:.2f}m======")
            self.movey(-d)
            self.get_logger().info("Info: ======移动结束======")
            self.get_logger().info(f"Info: ======调整姿态,旋转{rz_:.2f}°======")
            self.moverz(rz-rz_)
            self.get_logger().info("Info: ======转向结束======")
            
    def fast_back(self):
        self.robot.target_inworld.vector.x = self.backpoint["x"]
        self.robot.target_inworld.vector.y = self.backpoint["y"]
        self.robot.target_inworld.vector.z = self.backpoint["z"]
        self.robot.target_inworld.vector.rz = self.backpoint["rz"]
        self.robot.world2base()

        x = self.robot.target_inbase.vector.x
        y = self.robot.target_inbase.vector.y
        z = self.robot.target_inbase.vector.z
        rz = self.robot.target_inbase.vector.rz

        self.get_logger().info(
            f"Info:需要调整的位移 x: {x:.2f} y: {y:.2f} z: {z:.2f} rz : {rz:.2f}")

        if z == 0:
            self.get_logger().info("Warn: 深度位移非法！")
        else:
            self.fast_movez(z)
        time.sleep(2)

        if x == 0 and y == 0:
            self.get_logger().info("Warn: 非法位移！")
            self.get_logger().info(f"Info: ======调整姿态,旋转{rz_:.2f}°======")
            self.fast_moverz(rz)
            self.get_logger().info("Info: ======转向结束======")
        else:
            rz_ = atan2(y, x)*RAD2DEG - 270
            d = sqrt(x*x + y*y)
            self.get_logger().info(f"Info: ======转向目标点,旋转{rz_:.2f}°======")
            self.fast_moverz(rz_)
            self.get_logger().info("Info: ======转向结束======")
            time.sleep(5.0)
            self.get_logger().info(f"Info: ======移动指定距离,移动{rz:.2f}m======")
            self.fast_movey(-d)
            time.sleep(5.0)
            self.get_logger().info("Info: ======移动结束======")
            self.get_logger().info(f"Info: ======调整姿态,旋转{rz_:.2f}°======")
            self.fast_moverz(rz-rz_)
            time.sleep(2.0)
            self.get_logger().info("Info: ======转向结束======")
            
    # 回到路径点
    def backy(self):
        self.robot.target_inworld.vector.x = self.backpoint["x"]
        self.robot.target_inworld.vector.y = self.backpoint["y"]
        self.robot.target_inworld.vector.z = self.backpoint["z"]
        self.robot.target_inworld.vector.rz = self.backpoint["rz"]
        self.robot.world2base()

        x = self.robot.target_inbase.vector.x
        y = self.robot.target_inbase.vector.y
        z = self.robot.target_inbase.vector.z
        rz = self.robot.target_inbase.vector.rz

        self.get_logger().info(
            f"Info:需要调整的位移 x: {x:.2f} y: {y:.2f} z: {z:.2f} rz : {rz:.2f}")


        if x == 0 and y == 0:
            self.get_logger().info("Warn: 非法位移！")
            self.get_logger().info(f"Info: ======调整姿态,旋转{rz_:.2f}°======")
            self.moverz(rz)
            self.get_logger().info("Info: ======转向结束======")
        else:
            rz_ = atan2(y, x)*RAD2DEG - 270
            d = sqrt(x*x + y*y)
            self.get_logger().info(f"Info: ======转向目标点,旋转{rz_:.2f}°======")
            self.moverz(rz_)
            self.get_logger().info("Info: ======转向结束======")
            time.sleep(0.1)
            self.get_logger().info(f"Info: ======移动指定距离,移动{rz:.2f}m======")
            self.movey(-d)
            self.get_logger().info("Info: ======移动结束======")
            self.get_logger().info(f"Info: ======调整姿态,旋转{rz_:.2f}°======")
            self.moverz(rz-rz_)
            self.get_logger().info("Info: ======转向结束======")

        if z == 0:
            self.get_logger().info("Warn: 深度位移非法！")
        else:
            self.movez(z)
        time.sleep(0.1)

    # 移动至寄存器 self.target 所指定的位置
    def mtty(self, dy, dz):

        self.robot.target_inworld.vector.x = self.target["x"]
        self.robot.target_inworld.vector.y = self.target["y"]
        self.robot.target_inworld.vector.z = self.target["z"]
        self.robot.world2base()

        x = self.robot.target_inbase.vector.x
        y = self.robot.target_inbase.vector.y
        z = self.robot.target_inbase.vector.z

        z -= dz

        self.get_logger().info(
            f"Info:需要调整的位移 x: {x:.2f} y: {y:.2f} z: {z:.2f}")

        if z == 0:
            self.get_logger().info("Warn: 深度位移非法！")
        else:
            self.movez(z)
        time.sleep(0.1)

        if x == 0 and y == 0:
            self.get_logger().info("Warn: 非法位移！")
        else:
            rz = atan2(y, x)*RAD2DEG - 90
            d = sqrt(x*x + y*y) - dy
            self.get_logger().info(f"Info: ======转向目标点,旋转{rz:.2f}°======")
            self.moverz(rz)
            self.get_logger().info("Info: ======转向结束======")
            time.sleep(0.1)
            self.get_logger().info(f"Info: ======移动指定距离,移动{d:.2f}m======")
            self.movey(d)
            self.get_logger().info("Info: ======移动结束======")

    def mttzxy(self, dz, dx, dy):
        self.robot.target_inworld.vector.x = self.target["x"]
        self.robot.target_inworld.vector.y = self.target["y"]
        self.robot.target_inworld.vector.z = self.target["z"]
        self.robot.world2base()

        x = self.robot.target_inbase.vector.x
        y = self.robot.target_inbase.vector.y
        z = self.robot.target_inbase.vector.z

        x += dx
        y += dy
        z += dz

        self.get_logger().info(
            f"Info:需要调整的位移 x: {x:.2f} y: {y:.2f} z: {z:.2f}")
        
        if z == 0:
            self.get_logger().info("Warn: 深度位移非法！")
        else:
            self.get_logger().info(f"Info: ======深度移动指定距离,移动{z:.2f}m======")
            self.movez(z)
            self.get_logger().info("Info: ======移动结束======")
        time.sleep(0.1)

        if x == 0:
            self.get_logger().info("Warn: 水平位移非法！")
        else:
            self.get_logger().info(f"Info: ======横移指定距离,移动{x:.2f}m======")
            self.movex(x)
            self.get_logger().info("Info: ======移动结束======")
        time.sleep(0.1)

        if y == 0:
            self.get_logger().info("Warn: 前后位移非法！")
        else:
            self.get_logger().info(f"Info: ======前后指定距离,移动{y:.2f}m======")
            self.movey(y)
        time.sleep(0.1)

        self.get_logger().info(f"Info: ======移动结束======")


    def mttxf(self,dx,dy):
        self.robot.target_inworld.vector.x = self.target["x"]
        self.robot.target_inworld.vector.y = self.target["y"]
        self.robot.target_inworld.vector.z = self.target["z"]
        self.robot.world2base()

        x = self.robot.target_inbase.vector.x
        y = self.robot.target_inbase.vector.y
        z = self.robot.target_inbase.vector.z

        self.get_logger().info(
            f"Info:需要调整的位移 x: {x:.2f} y: {y:.2f} z: {z:.2f}")
        x_target = x - dx
        self.get_logger().info(f"Info: ======横移指定距离,移动{x:.2f}m======")
        self.movex(x_target)
        self.get_logger().info("Info: ======移动结束======")
        time.sleep(0.1)

        if y == 0:
            self.get_logger().info("Warn: 非法位移！")
        else:
            d = y - dy
            time.sleep(0.1)
            self.get_logger().info(f"Info: ======移动指定距离,移动{d:.2f}m======")
            self.movey(d)
            self.get_logger().info("Info: ======移动结束======")
        time.sleep(0.1)


    # 移动至寄存器 self.target 所指定的位置
    def mttz(self, dy, dz):

        self.robot.target_inworld.vector.x = self.target["x"]
        self.robot.target_inworld.vector.y = self.target["y"]
        self.robot.target_inworld.vector.z = self.target["z"]
        self.robot.world2base()

        x = self.robot.target_inbase.vector.x
        y = self.robot.target_inbase.vector.y
        z = self.robot.target_inbase.vector.z

        z -= dz

        self.get_logger().info(
            f"Info:需要调整的位移 x: {x:.2f} y: {y:.2f} z: {z:.2f}")

        if x == 0 and y == 0:
            self.get_logger().info("Warn: 非法位移！")
        else:
            rz = atan2(y, x)*RAD2DEG - 90
            d = sqrt(x*x + y*y) - dy
            self.get_logger().info(f"Info: ======转向目标点,旋转{rz:.2f}°======")
            self.moverz(rz)
            self.get_logger().info("Info: ======转向结束======")
            time.sleep(0.1)
            self.get_logger().info(f"Info: ======移动指定距离,移动{rz:.2f}m======")
            self.movey(d)
            self.get_logger().info("Info: ======移动结束======")

        if z == 0:
            self.get_logger().info("Warn: 深度位移非法！")
        else:
            self.movez(z)
        time.sleep(0.1)
    
    #移动至指定世界坐标位置
    def mttpos(self, x, y, z, rz, dy):
        self.target["x"] = x
        self.target["y"] = y
        self.target["z"] = z

        Step["y"] = 0.04
        self.mttz(dy, 0.0)
        self.setrz(rz)
        Step["y"] = 0.02
    
    #移动至指定世界坐标位置，但最后不旋转
    def mttzpos(self, x, y, z, dy):
        self.target["x"] = x
        self.target["y"] = y
        self.target["z"] = z
        Step["z"] = 0.2
        Step["y"] = 0.04
        self.mttz(dy, 0.0)
        Step["y"] = 0.02
        Step["z"] = 0.05
    
    #移动至指定世界坐标位置，以比赛开始位置为坐标
    def mttpos_amend(self, x, y, z, rz, dy):
        self.start_pos.target_inbase.vector.x = x
        self.start_pos.target_inbase.vector.y = y
        self.start_pos.target_inbase.vector.z = z
        self.start_pos.target_inbase.vector.rz = rz
        self.start_pos.target_inbase.vector.rx = self.start_pos.target_inbase.vector.ry= 0.0
        self.start_pos.base2world()
        self.target["x"] = self.start_pos.target_inworld.vector.x
        self.target["y"] = self.start_pos.target_inworld.vector.y
        self.target["z"] = self.start_pos.target_inworld.vector.z
        self.target["rz"] = self.start_pos.target_inworld.vector.rz
        
        self.get_logger().info(
            f"x: {self.target['x']:.2f} y: {self.target['y']:.2f} z: {self.target['z']:.2f}")
              
        Step["y"] = 0.04
        self.mttz(dy, 0.0)
        self.setrz(rz+self.start_pos.base.vector.rz)
        Step["y"] = 0.02
    
    #移动至指定世界坐标位置，以比赛开始位置为坐标
    def mttzpos_amend(self, x, y, z, dy):
        self.start_pos.target_inbase.vector.x = x
        self.start_pos.target_inbase.vector.y = y
        self.start_pos.target_inbase.vector.z = z
        self.start_pos.target_inbase.vector.rx = self.start_pos.target_inbase.vector.ry= 0.0
        self.start_pos.base2world()
        self.target["x"] = self.start_pos.target_inworld.vector.x
        self.target["y"] = self.start_pos.target_inworld.vector.y
        self.target["z"] = self.start_pos.target_inworld.vector.z
        Step["z"] = 0.2
        Step["y"] = 0.04
        self.mttz(dy, 0.0)
        Step["y"] = 0.02
        Step["z"] = 0.05
    
    # 搜寻目标
    def search(self, name, cam):
        # 定位目标
        # 等待服务段上线
        wait = True
        while rclpy.ok() and self.detect_request_client.wait_for_service(0.1) == False:
            if wait:
                wait = False
                self.get_logger().info("Info:等待服务端上线....")
        self.get_logger().info("Info:服务端已启动")

        request = DetectRequest.Request()
        request.stero = cam
        request.target = name
        responce = None

        pos_list = []
        fail_cnt = 0

        image = False

        while True:
            if cam == "front" and self.front_cam_Image_data != None:
                request.imagein = self.front_cam_Image_data
                image = True
            if cam == "down" and self.down_cam_Image_data != None:
                request.imagein = self.down_cam_Image_data
                image = True

            if image == True:
                responce = self.detect_request_client.call(request)
                if responce.s == 1:
                    if cam == "front":
                        self.front_cam.target_inbase.vector.x = responce.x
                        self.front_cam.target_inbase.vector.y = responce.y
                        self.front_cam.target_inbase.vector.z = responce.z
                        #相机坐标系到机器人坐标系
                        self.front_cam.base2world()

                        self.robot.target_inbase.vector.x = self.front_cam.target_inworld.vector.x
                        self.robot.target_inbase.vector.y = self.front_cam.target_inworld.vector.y
                        self.robot.target_inbase.vector.z = self.front_cam.target_inworld.vector.z
                        self.robot.target_inbase.vector.rx = self.front_cam.target_inworld.vector.rx
                        self.robot.target_inbase.vector.ry = self.front_cam.target_inworld.vector.ry
                        self.robot.target_inbase.vector.rz = self.front_cam.target_inworld.vector.rz
                        #机器人坐标系到世界坐标系
                        self.robot.base2world()

                    if cam == "down":
                        self.down_cam.target_inbase.vector.x = responce.x
                        self.down_cam.target_inbase.vector.y = responce.y
                        self.down_cam.target_inbase.vector.z = responce.z
                        self.down_cam.base2world()

                        self.robot.target_inbase.vector.x = self.down_cam.target_inworld.vector.x
                        self.robot.target_inbase.vector.y = self.down_cam.target_inworld.vector.y
                        self.robot.target_inbase.vector.z = self.down_cam.target_inworld.vector.z
                        self.robot.target_inbase.vector.rx = self.down_cam.target_inworld.vector.rx
                        self.robot.target_inbase.vector.ry = self.down_cam.target_inworld.vector.ry
                        self.robot.target_inbase.vector.rz = self.down_cam.target_inworld.vector.rz
                        self.robot.base2world()

                    # self.get_logger().info(
                    #     f"Info:目标在机器人坐标系中的位置: x: {self.robot.target_inbase.vector.x: .3f} y: {self.robot.target_inbase.vector.y: .3f} z: {self.robot.target_inbase.vector.z: .3f}")
                    self.get_logger().info(
                        f"Info:目标在相机坐标系中的位置: x: {responce.x: .3f} y: {responce.y: .3f} z: {responce.z: .3f}")

                    pos_list.append([self.robot.target_inworld.vector.x,
                                    self.robot.target_inworld.vector.y, self.robot.target_inworld.vector.z])

                    if len(pos_list) >= 5:
                        break
                else:
                    fail_cnt += 1
                    time
                    if fail_cnt >= 1000 & len(pos_list) < 5:
                        self.get_logger().info(f"确认无目标，退出函数")
                        return responce.s

            else:
                self.get_logger().info("无图像传入")

            image = False
        x = y = z = 0
        for i in pos_list:
            x += i[0]
            y += i[1]
            z += i[2]
        x /= len(pos_list)
        y /= len(pos_list)
        z /= len(pos_list)

        self.robot.base2world()
        self.target["name"] = name
        self.target["x"] = x
        self.target["y"] = y
        self.target["z"] = z

        self.get_logger().info(
            f"目标位置: x : {self.target['x']:.2f} y: {self.target['y']:.2f} z: {self.target['z']:.2f}")
        return responce.s

    def search2(self, name, cam,dy,dz,z_target,times):
        # 定位目标
        # 等待服务段上线
        wait = True
        while rclpy.ok() and self.detect_request_client.wait_for_service(0.1) == False:
            if wait:
                wait = False
                self.get_logger().info("Info:等待服务端上线....")
        self.get_logger().info("Info:服务端已启动")

        request = DetectRequest.Request()
        request.stero = cam
        request.target = name

        pos_list = []

        image = False
        times_none = 0 #
        flag_to_next = 0 #

        while True:
            if cam == "front" and self.front_cam_Image_data != None:
                request.imagein = self.front_cam_Image_data
                image = True
            if cam == "down" and self.down_cam_Image_data != None:
                request.imagein = self.down_cam_Image_data
                image = True

            if image == True:
                responce = self.detect_request_client.call(request)
                if responce.s == 1:
                    if cam == "front":
                        self.front_cam.target_inbase.vector.x = responce.x
                        self.front_cam.target_inbase.vector.y = responce.y
                        self.front_cam.target_inbase.vector.z = responce.z
                        #相机坐标系到机器人坐标系
                        self.front_cam.base2world()

                        self.robot.target_inbase.vector.x = self.front_cam.target_inworld.vector.x
                        self.robot.target_inbase.vector.y = self.front_cam.target_inworld.vector.y
                        self.robot.target_inbase.vector.z = self.front_cam.target_inworld.vector.z
                        self.robot.target_inbase.vector.rx = self.front_cam.target_inworld.vector.rx
                        self.robot.target_inbase.vector.ry = self.front_cam.target_inworld.vector.ry
                        self.robot.target_inbase.vector.rz = self.front_cam.target_inworld.vector.rz
                        #机器人坐标系到世界坐标系
                        self.robot.base2world()

                    if cam == "down":
                        self.down_cam.target_inbase.vector.x = responce.x
                        self.down_cam.target_inbase.vector.y = responce.y
                        self.down_cam.target_inbase.vector.z = responce.z
                        self.down_cam.base2world()

                        self.robot.target_inbase.vector.x = self.down_cam.target_inworld.vector.x
                        self.robot.target_inbase.vector.y = self.down_cam.target_inworld.vector.y
                        self.robot.target_inbase.vector.z = self.down_cam.target_inworld.vector.z
                        self.robot.target_inbase.vector.rx = self.down_cam.target_inworld.vector.rx
                        self.robot.target_inbase.vector.ry = self.down_cam.target_inworld.vector.ry
                        self.robot.target_inbase.vector.rz = self.down_cam.target_inworld.vector.rz
                        self.robot.base2world()

                    self.get_logger().info(
                        f"Info:目标在世界坐标系中的位置: x: {self.robot.target_inworld.vector.x: .3f} y: {self.robot.target_inworld.vector.y: .3f} z: {self.robot.target_inworld.vector.z: .3f}")

                    pos_list.append([self.robot.target_inworld.vector.x,
                                    self.robot.target_inworld.vector.y, self.robot.target_inworld.vector.z])

                    if len(pos_list) >= 100:
                        break
                else:
                    if (times > 0):
                        times_none += 1
                        self.get_logger().info(f"第{times_none}次无目标")
                        if (times_none >= 50) & (len(pos_list) < 10):
                            flag_to_next = 1
                            self.get_logger().info(f"确认无目标，进行下一步")
                            break
            else:
                self.get_logger().info("无图像传入")

            image = False

        times += 1
        if flag_to_next == 0:
            x = y = z = 0
            for i in pos_list:
                x += i[0]
                y += i[1]
                z += i[2]
            x /= len(pos_list)
            y /= len(pos_list)
            z /= len(pos_list)

            self.robot.base2world()
            self.target["name"] = name
            self.target["x"] = x
            self.target["y"] = y
            self.target["z"] = z

            self.get_logger().info(
                f"目标位置: x : {self.target['x']:.2f} y: {self.target['y']:.2f} z: {self.target['z']:.2f}")
            
            self.mttz(dy,dz)
            self.setz(z_target)
            self.get_logger().info(
                f"开始第{times}次搜查是否抓球成功")            
            self.search2(self, name, cam,dy,dz,z_target,times)
    
    def search4(self, name1, name2,dx,dy,dz,z_target,rz_target,times,distance):#1为圈，2为T插
       
        times_none = 0
        flag_to_next = 0
        
        T_cnt = 0
        while True:
            
            s = self.search(self, "scaffolding", "front")
            if s == 0:
                self.get_logger().info("======无目标======")
                self.led(0, 0)
                self.get_logger().info("======旋转寻找目标目标======")
                self.moverz(40)
                T_cnt += 1
            else :
                self.get_logger().info("======发现基架======")  
                self.target["y"] -= 0.50  
                self.mttz(dy,dz)
                self.setrz(rz_target)
                break
            if T_cnt > 10:
                self.get_logger().info("======未找到目标======")
                break
        
        """
        计算法线方向
        """
        # 解包坐标
        x1, y1, z1 = self.cam2robot(2, 4,"front")
        x2, y2, z2 = self.cam2robot(2, 4,"front")
        x3, y3, z3 = self.cam2robot(2, 4,"front")
            
            
        # 计算向量AC的分量
        a = x1 - x3
        b = y1 - y3
            
        # 计算向量模长的平方
        length_squared = a**2 + b**2
            
            
        # 计算u和v的可能值
        if  abs(b) >= 0.001:
            # 一般情况
            factor = distance * abs(b) / (length_squared ** 0.5)
            u1 = factor
            v1 = -a * u1 / b
                
            u2 = -factor
            v2 = -a * u2 / b
        else:
            # b为0的特殊情况（AC垂直于x轴）
            u1 = 0
            v1 = distance
                
            u2 = 0
            v2 = -distance
            
            # 计算点D的坐标
        d1 = (x2 + u1, y2 + v1, z3)
        d2 = (x2 + u2, y2 + v2, z3)
            
        real_d = min(d1, d2)    
             
        
              
    # 机械爪控制
    def pow(self, s):
        servo = ServoSet()
        servo.num = 0
        if s == 1:
            servo.angle = 0.35
            self.servo_control_pub.publish(servo)
        if s == 0:
            servo.angle = 0.92#推球，夹T插
            self.servo_control_pub.publish(servo)
        time.sleep(0.1)


    # 任务启动
    def start(self):
        time.sleep(5)
        #  载入当前目标值
        tpos = TargetPosDown()
        tpos.cs = 0
        self.start_pos.base.vector.x = tpos.pos.x = self.MotionController.pos.x
        self.start_pos.base.vector.y = tpos.pos.y = self.MotionController.pos.y
        tpos.pos.z = self.MotionController.pos.z
        self.start_pos.base.vector.z = 0.0
        self.start_pos.base.vector.rz = tpos.pos.rz = self.MotionController.pos.rz
        self.start_pos.base.vector.ry = self.start_pos.base.vector.rx = tpos.pos.rx = tpos.pos.ry =  0.0
        self.start_pos.base.extract()
        self.get_logger().info(f"载入当前世界坐标 x:{tpos.pos.x:.3f} y:{tpos.pos.y:.3f} z:{tpos.pos.z:.3f} rz:{tpos.pos.rz:.3f}")
        self.led(1,0)#绿灯
        time.sleep(3.0)
        self.led(0,1)#黄灯
        time.sleep(3.0)
        self.led(1,1)#红灯
        time.sleep(3.0)
        self.led(0,0)
        self.target_pos_down_pub.publish(tpos)
        # 机械爪加紧T插
        self.pow(0) 
        
        # 开启PID控制器
        pid = PidControllersState()
        pid.x = pid.y = pid.z = pid.rz = 1
        pid.rx = pid.ry = 0
        self.pid_controllers_set_pub.publish(pid)
        time.sleep(0.1)

    # 任务结束
    def end(self):
        # 关闭PID控制器
        pid = PidControllersState()
        pid.x = pid.y = pid.z = pid.rz = 0
        pid.rx = pid.ry = 0
        self.pid_controllers_set_pub.publish(pid)
        time.sleep(0.1)

    def run(self, task: dict):
        if task["name"] == "movexyz":
            self.movexyz(task["params"]["x"], task["params"]
                         ["y"], task["params"]["z"])

        elif task["name"] == "throw_golf":
            self.throw_golf(task["params"]["dy"],
                            task["params"]["depth"])
            
        elif task["name"] == "movexy":
            self.movexy(task["params"]["x"], task["params"]["y"])

            
        elif task["name"] == "movex":
            self.movex(task["params"]["x"])

        elif task["name"] == "movey":
            self.movey(task["params"]["y"])

        elif task["name"] == "movez":
            self.movez(task["params"]["z"])

        elif task["name"] == "moverz":
            self.moverz(task["params"]["rz"])

        elif task["name"] == "setz":
            self.setz(task["params"]["z"])

        elif task["name"] == "setrz":
            self.setrz(task["params"]["rz"])

        elif task["name"] == "search":
            self.search(task["params"]["name"], task["params"]["cam"])

        elif task["name"] == "mtty":
            self.mtty(task["params"]["y"], task["params"]["z"])
        elif task["name"] == "mttxf":
            self.mttxf(task["params"]["dx"],task["params"]["dy"])
        elif task["name"] == "mttz":
            self.mttz(task["params"]["y"], task["params"]["z"])
        elif task["name"] == "mttzxy":
            self.mttzxy(task["params"]["dz"], task["params"]["dx"], task["params"]["dy"])
        elif task["name"] == "setp":
            self.setp()

        elif task["name"] == "back":
            self.back()
            
        elif task["name"] == "backy":
            self.backy()
            
        elif task["name"] == "pow":
            self.pow(task["params"])
        
        elif task["name"] == "line":
            self.line(task["params"]["ys_dep"])

        elif task["name"] == "graball":
            self.graball(task["params"]["color"], task["params"]["depth"], task["params"]["timeout"],task["params"]["pr"],task["params"]["k"],task["params"]["step_time"])

        elif task["name"] == "thrball":
            self.thrball(task["params"]["pr"], task["params"]["timeout"],task["params"]["k"],task["params"]["step_time"])
        
        elif task["name"] == "pass_door":
            self.pass_door(task["params"]["num"])

        elif task["name"] == "mttpos":
            self.mttpos(task["params"]["x"], task["params"]["y"], task["params"]["z"], task["params"]["rz"], task["params"]["dy"])
            
        elif task["name"] == "mttzpos_amend":
            self.mttzpos_amend(task["params"]["x"], task["params"]["y"], task["params"]["z"], task["params"]["dy"])
        
        elif task["name"] == "mttpos_amend":
            self.mttpos_amend(task["params"]["x"], task["params"]["y"], task["params"]["z"], task["params"]["rz"], task["params"]["dy"])
            
        elif task["name"] == "mttzpos_":
            self.mttzpos(task["params"]["x"], task["params"]["y"], task["params"]["z"], task["params"]["dy"])
        
        elif task["name"] == "delay":
            self.delay(task["params"])

        elif task["name"] == "led":
            self.led(task["params"]["led0"],
                     task["params"]["led1"])

        elif task["name"] == "grab_golf":
            self.grab_golf(task["params"]["kind"],
                           task["params"]["dx"],
                           task["params"]["dy"],
                           task["params"]["down_depth"],
                           task["params"]["up_depth"])
        
        elif task["name"] == "put_t":
            self.put_t(task["params"]["num"],
                           task["params"]["dy"],
                           task["params"]["dz"])
        
        elif task["name"] == "strike_ball":
            self.strike_ball(task["params"]["num"])
        
        elif task["name"] == "strike_ball3":
            self.strike_ball3(task["params"]["num1"],
                       task["params"]["num2"],
                       task["params"]["num3"],)
        
        elif task["name"] == "line_qd":
            # 无参数，直接调用line_qd方法
            self.line_qd()
            
        elif task["name"] == "endfloat":
            # 无参数，直接调用line_qd方法
            self.endfloat()

        else:
            self.get_logger().info("Info:非法任务名:  " + task["name"])
    
    def pid_update(self, error):
        self.dt = self.dt + 1 if abs(error-self.previous_error) < 2 else 1
        p_value = self.pid_parameters.p * error
        self.i_error += error * self.dt
        i_value = self.pid_parameters.i * self.i_error
        d_value = self.pid_parameters.d * (error - self.previous_error) / self.dt
        self.previous_error = error
        output = p_value + i_value + d_value
        if output > self.pid_parameters.output_limit:
            output = self.pid_parameters.output_limit
        elif output < -self.pid_parameters.output_limit:
            output = -self.pid_parameters.output_limit
        return output
    

    def cam2robot(self, clas, timeout,cam):
        pos_list = []
        start_time = time.time()

        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # 检查超时
            if elapsed_time >= timeout:
                self.get_logger().info("超时退出")
                return 0.0, 0.0, 0.0
            if cam == "down":
                if self.yolov8_data_down.targets[clas].tpos_inworld.x != 0 or self.yolov8_data_down.targets[clas].tpos_inworld.y != 0 or self.yolov8_data_down.targets[clas].tpos_inworld.z != 0:
                    self.down_cam.target_inbase.vector.x = self.yolov8_data_down.targets[clas].tpos_inworld.x
                    self.down_cam.target_inbase.vector.y = self.yolov8_data_down.targets[clas].tpos_inworld.y
                    self.down_cam.target_inbase.vector.z = self.yolov8_data_down.targets[clas].tpos_inworld.z
                    #相机坐标系到机器人坐标系
                    self.down_cam.base2world()

                    t_x = self.robot.target_inbase.vector.x = self.down_cam.target_inworld.vector.x
                    t_y = self.robot.target_inbase.vector.y = self.down_cam.target_inworld.vector.y
                    t_z = self.robot.target_inbase.vector.z = self.down_cam.target_inworld.vector.z
                    t_rx = self.robot.target_inbase.vector.rx = self.down_cam.target_inworld.vector.rx
                    t_ry = self.robot.target_inbase.vector.ry = self.down_cam.target_inworld.vector.ry
                    t_rz = self.robot.target_inbase.vector.rz = self.down_cam.target_inworld.vector.rz
                    self.robot.base2world()
                    time.sleep(0.015)
                                    # 检查返回的坐标是否为 0
                    if t_x != 0 or t_y != 0 or t_z != 0:
                        pos_list.append([t_x, t_y, t_z])
                        self.get_logger().info(f"t_x : {t_x:.2f} t_y: {t_y:.2f} t_z: {t_z:.2f}")
            if cam == "front":
                if self.yolov8_data_front.targets[clas].tpos_inworld.x != 0 or self.yolov8_data_front.targets[clas].tpos_inworld.y != 0 or self.yolov8_data_front.targets[clas].tpos_inworld.z != 0:
                    self.front_cam.target_inbase.vector.x = self.yolov8_data_front.targets[clas].tpos_inworld.x
                    self.front_cam.target_inbase.vector.y = self.yolov8_data_front.targets[clas].tpos_inworld.y
                    self.front_cam.target_inbase.vector.z = self.yolov8_data_front.targets[clas].tpos_inworld.z
                    #相机坐标系到机器人坐标系
                    self.front_cam.base2world()
                    t_x = self.robot.target_inbase.vector.x = self.front_cam.target_inworld.vector.x
                    t_y = self.robot.target_inbase.vector.y = self.front_cam.target_inworld.vector.y
                    t_z = self.robot.target_inbase.vector.z = self.front_cam.target_inworld.vector.z
                    t_rx = self.robot.target_inbase.vector.rx = self.front_cam.target_inworld.vector.rx
                    t_ry = self.robot.target_inbase.vector.ry = self.front_cam.target_inworld.vector.ry
                    t_rz = self.robot.target_inbase.vector.rz = self.front_cam.target_inworld.vector.rz
                        #机器人坐标系到世界坐标系
                    self.robot.base2world()
                    time.sleep(0.015)
                    if t_x != 0 or t_y != 0 or t_z != 0:
                        pos_list.append([t_x, t_y, t_z])
                        self.get_logger().info(f"t_x : {t_x:.2f} t_y: {t_y:.2f} t_z: {t_z:.2f}")

            if len(pos_list) >= 80:
                break
        x = y = z = 0.0
        for i in pos_list:
            x += i[0]
            y += i[1]
            z += i[2]
        x /= len(pos_list)
        y /= len(pos_list)
        z /= len(pos_list)
        self.get_logger().info(
            f"目标在机器人坐标系下的位置: x : {x:.2f} y: {y:.2f} z: {z:.2f}")

        return x, y, z
    

    def cam2robot_fast(self, clas, timeout,cam):
        pos_list = []
        start_time = time.time()

        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # 检查超时
            if elapsed_time >= timeout:
                self.get_logger().info("超时退出")
                return 0.0, 0.0, 0.0
            if cam == "down":
                if self.yolov8_data_down.targets[clas].tpos_inworld.x != 0 or self.yolov8_data_down.targets[clas].tpos_inworld.y != 0 or self.yolov8_data_down.targets[clas].tpos_inworld.z != 0:
                    self.down_cam.target_inbase.vector.x = self.yolov8_data_down.targets[clas].tpos_inworld.x
                    self.down_cam.target_inbase.vector.y = self.yolov8_data_down.targets[clas].tpos_inworld.y
                    self.down_cam.target_inbase.vector.z = self.yolov8_data_down.targets[clas].tpos_inworld.z
                    #相机坐标系到机器人坐标系
                    self.down_cam.base2world()

                    t_x = self.robot.target_inbase.vector.x = self.down_cam.target_inworld.vector.x
                    t_y = self.robot.target_inbase.vector.y = self.down_cam.target_inworld.vector.y
                    t_z =  self.robot.target_inbase.vector.z = self.down_cam.target_inworld.vector.z
                    t_rx = self.robot.target_inbase.vector.rx = self.down_cam.target_inworld.vector.rx
                    t_ry = self.robot.target_inbase.vector.ry = self.down_cam.target_inworld.vector.ry
                    t_rz = self.robot.target_inbase.vector.rz = self.down_cam.target_inworld.vector.rz
                    self.robot.base2world()
                                    # 检查返回的坐标是否为 0
                    if t_x != 0 or t_y != 0 or t_z != 0:
                        pos_list.append([t_x, t_y, t_z])
                        self.get_logger().info(f"t_x : {t_x:.2f} t_y: {t_y:.2f} t_z: {t_z:.2f}")
            if cam == "front":
                if self.yolov8_data_front.targets[clas].tpos_inworld.x != 0 or self.yolov8_data_front.targets[clas].tpos_inworld.y != 0 or self.yolov8_data_front.targets[clas].tpos_inworld.z != 0:
                    self.front_cam.target_inbase.vector.x = self.yolov8_data_front.targets[clas].tpos_inworld.x
                    self.front_cam.target_inbase.vector.y = self.yolov8_data_front.targets[clas].tpos_inworld.y
                    self.front_cam.target_inbase.vector.z = self.yolov8_data_front.targets[clas].tpos_inworld.z
                    #相机坐标系到机器人坐标系
                    self.front_cam.base2world()
                    t_x = self.robot.target_inbase.vector.x = self.front_cam.target_inworld.vector.x
                    t_y = self.robot.target_inbase.vector.y = self.front_cam.target_inworld.vector.y
                    t_z = self.robot.target_inbase.vector.z = self.front_cam.target_inworld.vector.z
                    t_rx = self.robot.target_inbase.vector.rx = self.front_cam.target_inworld.vector.rx
                    t_ry = self.robot.target_inbase.vector.ry = self.front_cam.target_inworld.vector.ry
                    t_rz = self.robot.target_inbase.vector.rz = self.front_cam.target_inworld.vector.rz
                        #机器人坐标系到世界坐标系
                    self.robot.base2world()
                    if t_x != 0 or t_y != 0 or t_z != 0:
                        pos_list.append([t_x, t_y, t_z])
                        self.get_logger().info(f"t_x : {t_x:.2f} t_y: {t_y:.2f} t_z: {t_z:.2f}")
            if len(pos_list) >= 50:
                break
        x = y = z = 0.0
        for i in pos_list:
            x += i[0]
            y += i[1]
            z += i[2]
        x /= len(pos_list)
        y /= len(pos_list)
        z /= len(pos_list)
        self.get_logger().info(
            f"目标在机器人坐标系下的位置: x : {x:.2f} y: {y:.2f} z: {z:.2f}")

        return x, y, z
    
    #抓球 pr为百分比距离
    def graball(self,color,depth,timeout,pr,k,step_time):
        led = LedControllers()
        start_time = time.time()
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time >= timeout:
                self.get_logger().info("超时退出")
                break
            if all(x==0 for x in self.yolov8_data_down.state):
                self.led(0,0)
                self.get_logger().info(
                f"没有找到任何物体")
            elif self.yolov8_data_down.state[5] == 1 and self.yolov8_data_down.state[4] == 1: #5是黄色，4是pink
                self.led(1,0)
                self.get_logger().info(
                f"发现高尔夫球")
                if color == "pink":
                    a, b, c = self.cam2robot(4, 5,"down")
                    self.movex(a)
                    self.movey(b-0.175)
                    self.movez(depth)
                    self.movez(-depth)
                    break
                elif color == "yellow":
                    a, b, c = self.cam2robot(5, 5,"down")
                    self.movex(a)
                    self.movey(b-0.175)
                    self.movez(depth)
                    self.movez(-depth)
                    break
            elif self.yolov8_data_down.state[6] == 1:
                led.led0 = 0
                led.led1 = 1
                r = sqrt((self.yolov8_data_down.targets[6].tpos_inpic.y -  480)**2 + (self.yolov8_data_down.targets[6].tpos_inpic.x -  640)**2)/sqrt(480**2+640**2)
                a, b, c = self.cam2robot_fast(6, 5,"down")
                if r >= pr:
                    self.led_controllers_pub.publish(led)
                    self.get_logger().info(
                    f"只发现陈列框")
                    vx = k*a
                    vy = k*b
                    if abs(vx) < 0.01:
                        vx = 0.01 if vx >= 0 else -0.01
                    if abs(vy) < 0.01:
                        vy = 0.01 if vy >= 0 else -0.01
                    if abs(vx) > 0.04:
                        vx = 0.04 if vx >= 0 else -0.04
                    if abs(vy) > 0.04:
                        vy = 0.04 if vy >= 0 else -0.04
                    self.fast_movex(vx)
                    self.fast_movey(vy)
                    time.sleep(step_time)
                else:
                    self.get_logger().info(
                    f"陈列框已经进入视野中心")
            else:
                self.led(1,1)
                self.get_logger().info(
                f"检测到非必要物体")       
        self.led(0,0)
    #投球
    def thrball(self,pr,timeout,k,step_time):
        led = LedControllers()
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(self.down_cam_Image_data,"bgr8")
        row_index = img.shape[0] // 2
        column_index = img.shape[1] // 4
        self.get_logger().info(f"row: {row_index} , column: {column_index}")
        start_time = time.time()
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time >= timeout:
                self.get_logger().info("超时退出")
                break
            if all(x==0 for x in self.yolov8_data_down.state):
                led.led0 = 0
                led.led1 = 0
                self.led_controllers_pub.publish(led)
                self.get_logger().info(
                f"没有找到任何物体")
            elif self.yolov8_data_down.state[5] == 1:
                r = sqrt((self.yolov8_data_down.targets[5].tpos_inpic.y -  row_index)**2 + (self.yolov8_data_down.targets[5].tpos_inpic.x -  column_index)**2)/sqrt(480**2+640**2)
                led.led0 = 1
                led.led1 = 0
                a, b, c = self.cam2robot_fast(5, 5,"down")
                if r >= pr:
                    self.led_controllers_pub.publish(led)
                    self.get_logger().info(
                                            f"发现收集框")
                    vx = k*a
                    vy = k*b
                    if abs(vx) < 0.01:
                        vx = 0.01 if vx >= 0 else -0.01
                    if abs(vy) < 0.01:
                        vy = 0.01 if vy >= 0 else -0.01
                    if abs(vx) > 0.04:
                        vx = 0.04 if vx >= 0 else -0.04
                    if abs(vy) > 0.04:
                        vy = 0.04 if vy >= 0 else -0.04
                    self.fast_movex(vx)
                    self.fast_movey(vy)
                    time.sleep(step_time)
                else:
                    self.get_logger().info(f"收集框已进入视野中心")
                    break
            else:
                led.led0 = 1
                led.led1 = 1
                self.led_controllers_pub.publish(led)
                self.get_logger().info(
                f"检测到非必要物体")
        a, b, c = self.cam2robot(5, 5,"down")
        led.led0 = 1
        led.led1 = 0
        self.led_controllers_pub.publish(led)
        self.movex(a)
        self.movey(b-0.175)   
        self.pow(0) 
        time.sleep(1)      
        led.led0 = 0
        led.led1 = 0
        self.led_controllers_pub.publish(led)

    def pass_door(self,num):
        self.setz(0.5)
        #self.setrz(0)
        start_time = time.time()
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time >= 30:
                self.get_logger().info("超时,采用planB")
                self.movey(3.5)
                break
            if self.yolov8_data_front.state[num] == 1:
                self.get_logger().info(f"检测到资格门")
                time.sleep(2)
                a, b, c = self.cam2robot_fast(num, 5,"front")
                b = b 
                rz = atan2(b, a)*RAD2DEG - 90
                self.get_logger().info(f"Info: ======转向目标点,旋转{rz:.2f}°======")
                self.moverz(rz)
                cnt = 0
                cnt2 = 0
                cnt3 = 0
                cnt4 = 0
                cmd = TargetPosDown()
                cmd.cs = 2
                cmd.pos.rx = 0.0
                cmd.pos.ry = 0.0
                cmd.pos.x = 0.0
                cmd.pos.y = 0.01
                cmd.pos.z = 0.0
                cmd.pos.rz = 1.5
                time.sleep(0.5)
                while True:
                    cnt+=1
                    cnt4 += 1#0.1秒后再进行一次检测
                    if cnt > 100 * b:             
                        break
                    self.target_pos_down_pub.publish(cmd)
                    time.sleep(0.05)
                break

        time.sleep(5.0)
        self.movey(0.5)
        self.get_logger().info(f"过门任务完成")

    def led(self, led0, led1):
        led = LedControllers()
        led.led0 = led0
        led.led1 = led1
        self.led_controllers_pub.publish(led)
    
    def delay(self, t):
        time.sleep(t)
    
    def grab_golf(self, kind, dx, dy, down_depth, up_depth):
        
        num = 0
        
        if kind == "blue_golf":
            num = 4
        if kind == "red_golf":
            num = 3

        
        cmd = TargetPosDown()
        cmd.cs = 2
        cmd.pos.rx = 0.0
        cmd.pos.ry = 0.0
        cmd.pos.x = 0.0
        cmd.pos.y = 0.0
        cmd.pos.z = 0.0
        cmd.pos.rz = 0.1
        
        sample_flag = 0
        golf_cnt = 0
        
        self.pow(1)
        self.led(0, 0)
        s = 1
        while True:
            if all(x == 0 for x in self.yolov8_data_front.state) and \
               all(y == 0 for y in self.yolov8_data_down.state):
                # pass
                self.target_pos_down_pub.publish(cmd)
                time.sleep(0.01)
                golf_cnt+=1
                
            else:
                if self.yolov8_data_front.state[6]!= 0 :
                    self.get_logger().info("前视发现置物台")
                    time.sleep(2)
                    a, b, c = self.cam2robot(6, 5,"front")
                    a = a + 0.1
                    b = b - 0.1                    
                    self.movex(a)
                    self.movey(b)
                    self.setz(0.2)
                    time.sleep(2)
                    break
                elif self.yolov8_data_down.state[6]!= 0 :
                    self.get_logger().info("下视发现置物台")
                    time.sleep(2)
                    a, b, c = self.cam2robot(6, 5,"down")
                    a = a + 0.1
                    b = b - 0.1                      
                    self.movex(a)
                    self.movey(b)
                    self.setz(0.2)
                    time.sleep(2)
                    break
                else:
                    self.target_pos_down_pub.publish(cmd)
                    time.sleep(0.01)
                    golf_cnt+=1 
            if  golf_cnt>4000 :
                self.get_logger().info("任务失败")
                self.get_logger().info("======上浮======")
                self.setz(-0.5)
                return
                     
        '''s = self.search(kind, "down")
        if s == 0:
            self.get_logger().info("======下无目标======")
            self.get_logger().info("======第一次平移寻找目标目标======")
            self.movex(0.2)
            self.movey(0.2)
            s = self.search( kind, "down")
            if s == 0:
                self.get_logger().info("======下无目标======")
                self.get_logger().info("======第二次平移寻找目标目标======")
                self.movey(-0.4)
                s = self.search( kind, "down")
                if s == 0:
                    self.get_logger().info("======下无目标======")
                    self.get_logger().info("======第三次平移寻找目标目标======")
                    self.movex(-0.4)
                    s = self.search( kind, "down")
                    if s == 0:
                        self.get_logger().info("======下无目标======")
                        self.get_logger().info("======第四次平移寻找目标目标======")
                        self.movey(0.4)
                        s = self.search(kind, "down")
                        if s == 0:
                            self.get_logger().info("======下无目标======")
                            self.get_logger().info("======任务失败======")
                            return
        if s != 0 :
            self.get_logger().info("======发现球======")  
            self.get_logger().info("======正在移向球======")   
            self.mttxf(0,0) '''
        golf_cnt = 0
        while True: 
            if all(x == 0 for x in self.yolov8_data_front.state) and \
               all(y == 0 for y in self.yolov8_data_down.state):
                golf_cnt+=1
                time.sleep(0.01)
                if golf_cnt>2000:
                    self.get_logger().info("======确认无目标======")
                    self.get_logger().info("======上浮======")
                    self.setz(-0.5)
                    time.sleep(5.0)
                    self.get_logger().info("=====上浮结束======")
                    return                  
            elif self.yolov8_data_down.state[num] !=0:
                self.get_logger().info("======发现目标球=====")
                a, b, c = self.cam2robot(num, 5,"down")
                a = a - dx
                b = b - dy 
                self.movex(a)
                self.movey(b)

                self.delay(1)
                self.led(1, 0)
                self.movez(down_depth)
                self.get_logger().info("=====抓球结束，开始上浮======")
                self.setz(-0.5)
                time.sleep(5.0)
                self.led(0, 0)
                self.get_logger().info("=====上浮结束======")
                return
            else:
                golf_cnt+=1
                time.sleep(0.01)
                if golf_cnt>2000:
                    self.get_logger().info("======确认无目标======")
                    self.get_logger().info("======上浮======")
                    self.setz(-0.5)
                    time.sleep(5)
                    self.get_logger().info("=====上浮结束======")
                    return  
    
    
    def put_t(self, num,dy,dz):
        T_cnt = 0
        T_cnt2 = 0
        T_cnt3 = 0
        cmd = TargetPosDown()
        cmd.cs = 2
        cmd.pos.rx = 0.0
        cmd.pos.ry = 0.0
        cmd.pos.x = 0.0
        cmd.pos.y = 0.0
        cmd.pos.z = 0.0
        cmd.pos.rz = 0.01

        while True:
            if self.yolov8_data_front.state[num] == 0: 
                self.target_pos_down_pub.publish(cmd)
                time.sleep(0.001)
                T_cnt+=1
                

            elif  self.yolov8_data_front.state[num] != 0:
                time.sleep(2)
                T_cnt = 0
                self.get_logger().info("发现目标")
                a, b, c = self.cam2robot(num, 5,"front")
                a=a-dy
                c=c-dz
                self.movex(a)
                self.movez(c)
                time.sleep(1)
                cmd.pos.rz = 0.0
                while True:
                    T_cnt2+=1
                    if self.yolov8_data_front.state[num] != 0 and T_cnt2 % 600 == 0: 
                        time.sleep(2)
                        a, b, c = self.cam2robot(num, 5,"front")
                        a=a-dy
                        c=c-dz
                        if (abs(a) > 0.03 or abs(c) > 0.03) and b > 0.25:
                            self.get_logger().info("======目标偏移======")
                            cmd.pos.x = float(a)
                            cmd.pos.z = float(c)
                            cmd.pos.y = 0.0
                            self.target_pos_down_pub.publish(cmd)
                            time.sleep(2)
                        else:
                            cmd.pos.y = 0.001
                            cmd.pos.x = 0.0
                            cmd.pos.z = 0.0
                            self.target_pos_down_pub.publish(cmd)
                            self.get_logger().info("======正在前进======")
                            T_cnt+=1
                            time.sleep(0.01)

                    else:
                        cmd.pos.y = 0.001
                        cmd.pos.x = 0.0
                        cmd.pos.z = 0.0
                        self.target_pos_down_pub.publish(cmd)
                        T_cnt+=1
                        time.sleep(0.01)
                    if T_cnt > 1000*(b+0.7):
                            self.pow(1)
                            self.get_logger().info("======任务完成======")
                            return
            if T_cnt>4000:
                self.get_logger().info("======未发现目标 任务失败======")            
                
    def endfloat(self):
   
        num = 5
        cmd = TargetPosDown()
        cmd.cs = 2
        cmd.pos.rx = 0.0
        cmd.pos.ry = 0.0
        cmd.pos.x = 0.0
        cmd.pos.y = 0.0
        cmd.pos.z = 0.0
        cmd.pos.rz = 0.2
        plate_cnt = 0
        while True:
            if self.yolov8_data_front.state[num] ==0 and \
               self.yolov8_data_down.state[num] ==0 :
                # pass
                self.target_pos_down_pub.publish(cmd)
                time.sleep(0.01)
                golf_cnt+=1
                
            else:
                if self.yolov8_data_front.state[num] ==1:
                    self.get_logger().info("前视发现目标")
                    time.sleep(2)
                    a, b, c = self.cam2robot(num, 5,"front")
                    a = a - 0.3
                    b = b - 0.5                    
                    self.movex(a)
                    self.movey(b)
                    time.sleep(1)
                    self.setz(-0.1)
                    self.get_logger().info("任务成功")
                    break
                else:
                    self.get_logger().info("下视发现目标")
                    a, b, c = self.cam2robot(num, 5,"down")
                    a = a - 0.3
                    b = b - 0.5                    
                    self.movex(a)
                    self.movey(b)
                    time.sleep(1)
                    self.setz(-0.1)
                    self.get_logger().info("任务成功")
                    break
            if  plate_cnt>2000 :
                self.get_logger().info("任务失败")
                return


    def throw_golf(self, dy, depth):
        self.get_logger().info("======开始投球======")
        self.led(0, 1)
        self.search('col_basket', 'down')
        self.led(1, 1)
        self.mttxf(0, dy)
        self.movez(depth)
        self.led(1, 0)
        self.pow(0)
        self.delay(1)
        self.led(0, 0)
        self.get_logger().info("======投球结束======")
        
    def strike_ball(self,num):
        
        ball_cnt = 0
        cmd = TargetPosDown()
        cmd.cs = 2
        cmd.pos.rx = 0.0
        cmd.pos.ry = 0.0
        cmd.pos.x = 0.0
        cmd.pos.y = 0.0
        cmd.pos.z = 0.0
        cmd.pos.rz = 0.05

        while True:
            if self.yolov8_data_front.state[num] == 0: 
                self.target_pos_down_pub.publish(cmd)
                time.sleep(0.01)
                ball_cnt+=1
            elif  self.yolov8_data_front.state[num] != 0:
                time.sleep(2)
                ball_cnt = 0
                self.get_logger().info("发现目标")
                a, b, c = self.cam2robot(num, 5,"front")
                #a = a 
                c = c - 0.32
                b = b - 0.2
                self.fast_movez(c)
                time.sleep(1)
                self.movexy(a,b)
                time.sleep(1)
                self.get_logger().info("======任务完成 正在返回======")
                self.fast_movey(-b)
                self.move_wait()
                time.sleep(4)
                self.get_logger().info("======返回完成======")
                               
                return
            if ball_cnt>75000:
                self.get_logger().info("======未发现目标 任务失败======")
                return
                
    def strike_ball3(self,num1,num2,num3):
        self.get_logger().info("======撞球任务开始======")
        #self.setp()
        self.get_logger().info("======撞第一个球======")
        self.strike_ball(num1)
        #self.fast_moverz(180.0)
        #self.get_logger().info("======正在返回======")
        #self.fast_back()
        #time.sleep(4)
        self.get_logger().info("======撞第二个球======")
        self.strike_ball(num2)
        #self.fast_moverz(180.0)
        #self.get_logger().info("======正在返回======")
        #self.fast_back()
        #time.sleep(4)
        self.get_logger().info("======撞第三个球======")
        self.strike_ball(num3)
        #self.get_logger().info("======正在返回======")
        #self.fast_back()
        
    def line_qd(self):
        
            
        self.led(0,0)
        
        self.pid_parameters.p = 0.0009
        self.pid_parameters.i = 0.0
        self.pid_parameters.d = 0.08
        self.pid_parameters.output_limit = 5.0

        cmd = TargetPosDown()
        cmd.cs = 2
        cmd.pos.rx = 0.0
        cmd.pos.ry = 0.0

        drz = 0.0

        timesleep = 0.025
        
        self.task_lock = [0,0,0]
        
        magnet = MagnetController()
        led = LedControllers()
        bridge = CvBridge()
    
        window_size = 3  # 滑动窗口的大小
        last_positions = []  # 使用列表来存储最近的位置
        cnt = 0
        cnt_1 = 0
        #等待准备就绪
        time.sleep(1.0)
        while True:
            #try:
            # 转化为opencv图像
            img1 = bridge.imgmsg_to_cv2(self.down_cam_Image_data, "mono8")
            img = bridge.imgmsg_to_cv2(self.segment_img, "mono8")
            if img is None:
                self.get_logger().error("bridge.imgmsg_to_cv2 returned None")
                return
            img = cv2.flip(img, -1)

            row_index = img.shape[0] // 3
            row_index1 = img.shape[0] // 2
            column_index = img.shape[1] // 2
            row = img[row_index, :]
            white_pixels = np.where(row == 255)[0]

            # 计算白点的平均位置作为线的中心
            if white_pixels.size > 0:
                line_center = np.mean(white_pixels).astype(int)
                last_positions.append(line_center)
                if len(last_positions) > window_size:  # 如果列表长度超过窗口大小，删除最旧的元素
                    last_positions.pop(0)
            else:
                if last_positions:
                    line_center = last_positions[-1]  # 如果没有检测到新的，则使用上一个

            # 计算滑动平均
            if len(last_positions) > 1:
                smoothed_center = int(np.mean(last_positions))
                cv2.circle(img, (smoothed_center, row_index), 10, (0, 0, 0), -1)  # 标记平滑后的中心点
            else:
                smoothed_center = img.shape[1] // 2
                cv2.circle(img, (smoothed_center, row_index), 10, (0, 0, 0), -1)

            # 在图像上添加文字
            cv2.putText(img, "output: {:.2f}".format(drz), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)

            img_msg = bridge.cv2_to_imgmsg(img, "mono8")
            self.line_patrol_img_pub.publish(img_msg)

            angle_error = smoothed_center - column_index
            drz = self.pid_update(angle_error)

            if all(x==0 for x in self.yolov8_data_down.state):
                # pass
                cmd.pos.x = 0.0
                cmd.pos.y = 0.002
                cmd.pos.z = 0.0
                cmd.pos.rz = drz
                self.target_pos_down_pub.publish(cmd)
                time.sleep(timesleep)
                cnt_1 += 1
                
            elif not all(x==0 for x in self.yolov8_data_down.state):
                if  self.yolov8_data_down.state[0] != 0 and  self.yolov8_data_down.targets[0].tpos_inpic.y > row_index1 :
                    if self.task_lock[0] == 0 :
                        self.get_logger().info("Info:检测A，开始执行任务") 
                        #self.setp()
                        #a, b, c = self.cam2robot(1, 5,"down")
                        #self.movex(a)
                        #self.movey(b)
                    
                        # 保存任务结束时的图片
                        task1_end_img_path = f"A.png"
                        cv2.imwrite(task1_end_img_path, img1)
                        self.get_logger().info(f"Info:保存图片 {task1_end_img_path}")

                        self.get_logger().info("Info:任务执行完毕")
                        #self.back()
                        self.task_lock[0] = 1
                    else:
                        #pass
                        cmd.pos.x = 0.0
                        cmd.pos.y = 0.002
                        cmd.pos.z = 0.0
                        cmd.pos.rz = drz
                        self.target_pos_down_pub.publish(cmd)
                        cnt_1 += 1
                        time.sleep(timesleep)
                elif self.yolov8_data_down.state[1] != 0 and self.yolov8_data_down.targets[1].tpos_inpic.y > row_index1:
                    if self.task_lock[1] == 0 :
                        self.get_logger().info("Info:检测B，开始执行任务")
                        #self.setp()
                        #a, b, c = self.cam2robot(2, 4,"down")
                        #self.movex(a)
                        #self.movey(b)
                        
                        # 保存任务结束时的图片
                        task2_end_img_path = f"B.png"
                        cv2.imwrite(task2_end_img_path, img1)
                        self.get_logger().info(f"Info:保存图片 {task2_end_img_path}")

                        #self.back()
                        self.get_logger().info("Info:任务执行完毕")
                        self.task_lock[1] = 1
                    else:
                        #pass
                        cmd.pos.x = 0.0
                        cmd.pos.y = 0.002
                        cmd.pos.z = 0.0
                        cmd.pos.rz = drz
                        self.target_pos_down_pub.publish(cmd)
                        cnt_1 += 1
                        time.sleep(timesleep)
                elif self.yolov8_data_down.state[2] != 0 and self.yolov8_data_down.targets[2].tpos_inpic.y > row_index1:
                    if self.task_lock[2] == 0 :
                        self.get_logger().info("Info:检测C，开始执行任务") 
                        #self.setp()
                        #a, b, c = self.cam2robot(0, 4,"down")
                        #self.movex(a)
                       # self.movey(b)
                        # 保存任务结束时的图片
                        task3_end_img_path = f"C.png"
                        cv2.imwrite(task3_end_img_path, img1)
                        self.get_logger().info(f"Info:保存图片 {task3_end_img_path}")

                        self.get_logger().info("Info:任务执行完毕")
                        self.task_lock[2] = 1
                    else:
                        #pass
                        cmd.pos.x = 0.0
                        cmd.pos.y = 0.002
                        cmd.pos.z = 0.0
                        cmd.pos.rz = drz
                        self.target_pos_down_pub.publish(cmd)
                        cnt_1 += 1
                        time.sleep(timesleep)

                else:
                    #pass
                    cmd.pos.x = 0.0
                    cmd.pos.y = 0.002
                    cmd.pos.z = 0.0
                    cmd.pos.rz = drz
                    self.target_pos_down_pub.publish(cmd)
                    cnt_1 += 1
                    time.sleep(timesleep)
            if cnt_1 > 1200:
                self.get_logger().info("Info:任务全部执行完毕")
                return
            if all(x == 1 for x in self.task_lock):
                cmd.pos.x = 0.0
                cmd.pos.y = 0.004
                cmd.pos.z = 0.0
                cmd.pos.rz = drz
                self.target_pos_down_pub.publish(cmd)
                time.sleep(timesleep)
                cnt+=1
                if cnt > 200:
                    self.get_logger().info("Info:任务全部执行完毕")
                    return

          
    #巡线
    def line(self, ys_dep):
        self.led(0, 0)
        
        self.pid_parameters.p = 0.0009
        self.pid_parameters.i = 0.0
        self.pid_parameters.d = 0.08
        self.pid_parameters.output_limit = 5.0

        cmd = TargetPosDown()
        cmd.cs = 2
        cmd.pos.rx = 0.0
        cmd.pos.ry = 0.0

        drz = 0.0
        depth = ys_dep
        timesleep = 0.025
        
        self.task_lock = [0, 0, 0]
        
        magnet = MagnetController()
        bridge = CvBridge()
        
        # 初始化1D卡尔曼滤波器（仅跟踪位置）
        kf = KalmanFilter(dim_x=1, dim_z=1)
        kf.x = np.array([320.])  # 初始位置（图像中心附近）
        kf.F = np.array([[1.]])  # 状态转移矩阵（简单恒速模型）
        kf.H = np.array([[1.]])  # 观测矩阵
        kf.P *= 100.  # 初始协方差
        kf.R = 3.0    # 观测噪声
        kf.Q = np.array([[1.0]])  # 过程噪声
        
        # 图像状态跟踪变量
        last_valid_image_time = None
        has_valid_image = False
        last_filtered_center = 320  # 初始中心位置
        img_shape = (480, 640)  # 预设图像尺寸，根据实际情况调整
        row_index = img_shape[0] // 3  # 预设行索引
        row_index1 = img_shape[0] // 2  # 预设任务检测行索引
        column_index = img_shape[1] // 2  # 预设列中心
        
        # 新增：图像初始化等待机制
        init_wait_time = 0.1  # 初始化等待0.1秒
        start_time = time.time()
        self.get_logger().info(f"等待图像初始化，最多等待{init_wait_time}秒...")
        
        # 等待图像或超时
        while not has_valid_image and (time.time() - start_time) < init_wait_time:
            try:
                if self.segment_img is not None:
                    img = bridge.imgmsg_to_cv2(self.segment_img, "mono8")
                    img_shape = img.shape
                    row_index = img_shape[0] // 3
                    row_index1 = img_shape[0] // 2
                    column_index = img_shape[1] // 2
                    has_valid_image = True
                    last_valid_image_time = time.time()
                    self.get_logger().info("成功获取初始图像")
            except Exception as e:
                self.get_logger().debug(f"初始化阶段图像获取失败: {str(e)}")
            time.sleep(0.1)  # 短时间等待后重试
        
        if not has_valid_image:
            self.get_logger().warn(f"初始化超时，未获取到图像，将使用默认参数运行")
            
        cnt = 0
        cnt_1 = 0
        while True:
            cnt_1 = cnt_1 + 1
            if cnt_1 > 2000:
                self.get_logger().warn(f"时间结束自动退出")
                return
                
            # 尝试获取和处理图像
            try:
                if self.segment_img is not None:
                    img = bridge.imgmsg_to_cv2(self.segment_img, "mono8")
                    img = cv2.flip(img, -1)
                    img_shape = img.shape
                    row_index = img_shape[0] // 3
                    row_index1 = img_shape[0] // 2
                    column_index = img_shape[1] // 2
                    last_valid_image_time = time.time()
                    has_valid_image = True
                    
                    # 处理图像获取线中心
                    row = img[row_index, :]
                    white_pixels = np.where(row == 255)[0]

                    # 卡尔曼滤波处理
                    kf.predict()
                    
                    # 有观测值时更新
                    if white_pixels.size > 0:
                        line_center = np.mean(white_pixels).astype(int)
                        kf.update(line_center)
                    
                    # 使用滤波后的值
                    filtered_center = int(kf.x[0])
                    last_filtered_center = filtered_center
                    
                    # 绘制标记并发布图像
                    cv2.circle(img, (filtered_center, row_index), 10, (0, 0, 0), -1)
                    cv2.putText(img, "output: {:.2f}".format(drz), (50, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
                    img_msg = bridge.cv2_to_imgmsg(img, "mono8")
                    self.line_patrol_img_pub.publish(img_msg)
                    

                        
            except Exception as e:
                # 图像处理失败时的处理
                self.get_logger().warn(f"无分割图像: {str(e)}")
                
                # 仅使用预测
                kf.predict()
                filtered_center = int(kf.x[0])
                last_filtered_center = filtered_center
                
                # 状态判断与警示
                if not has_valid_image:
                    self.get_logger().warn("持续未接收到图像，使用默认控制策略")
                elif (time.time() - last_valid_image_time) > 1.0:
                    self.get_logger().warn("图像已丢失超过1秒，使用预测值控制")
    
            
            # 计算控制量
            angle_error = filtered_center - column_index
            # 无图像时减小控制增益，采用更保守的控制
            if not has_valid_image:
                drz = self.pid_update(angle_error) * 0.5  # 降低控制强度
                dy = 0.004  # 降低前进速度
            else:
                drz = self.pid_update(angle_error)
                dy = 0.005 - 0.0004 * abs(drz)
            
            # 控制逻辑
            if all(x == 0 for x in self.yolov8_data_down.state):
                cmd.pos.x = 0.0
                cmd.pos.y = dy
                cmd.pos.z = 0.0
                cmd.pos.rz = drz
                self.target_pos_down_pub.publish(cmd)
                time.sleep(timesleep)
                
            elif not all(x == 0 for x in self.yolov8_data_down.state):
                if self.yolov8_data_down.state[1] != 0 and self.yolov8_data_down.targets[1].tpos_inpic.y > row_index1:
                        if self.task_lock[0] == 0:
                            self.get_logger().info("检测到黑色方块，开始执行任务")
                            time.sleep(2)
                            magnet.state = 0
                            self.magnet_controller_pub.publish(magnet)
                            self.led(1, 1)
                            time.sleep(1)
                            self.get_logger().info("任务执行完毕")
                            self.task_lock[0] = 1
                        else:
                            cmd.pos.x = 0.0
                            cmd.pos.y = dy
                            cmd.pos.z = 0.0
                            cmd.pos.rz = drz
                            self.target_pos_down_pub.publish(cmd)
                            time.sleep(timesleep)
                elif self.yolov8_data_down.state[2] != 0 and self.yolov8_data_down.targets[2].tpos_inpic.y > row_index1:
                    if self.task_lock[1] == 0:
                        self.get_logger().info("检测到绿色圆形，开始执行任务")
                        time.sleep(2)
                        a, b, c = self.cam2robot(2, 4, "down")
                        b = b - 0.4
                        self.movex(a)
                        self.movey(b)
                        self.movez(depth)
                        self.led(1,0)
                        self.movez(-depth)
                        self.led(0,0)
                        self.get_logger().info("任务执行完毕")
                        self.task_lock[1] = 1
                    else:
                        cmd.pos.x = 0.0
                        cmd.pos.y = dy
                        cmd.pos.z = 0.0
                        cmd.pos.rz = drz
                        self.target_pos_down_pub.publish(cmd)
                        time.sleep(timesleep)
                elif self.yolov8_data_down.state[0] != 0 and self.yolov8_data_down.targets[0].tpos_inpic.y > row_index1:
                    if self.task_lock[2] == 0:
                        self.get_logger().info("检测到黄色三角，开始执行任务")
                        time.sleep(2)
                        self.moverz(179.9)
                        time.sleep(1)
                        self.moverz(179.9)
                        time.sleep(1)
                        self.get_logger().info("任务执行完毕")
                        self.task_lock[2] = 1
                    else:
                        cmd.pos.x = 0.0
                        cmd.pos.y = dy
                        cmd.pos.z = 0.0
                        cmd.pos.rz = drz
                        self.target_pos_down_pub.publish(cmd)
                        time.sleep(timesleep)
                else:
                    cmd.pos.x = 0.0
                    cmd.pos.y = dy
                    cmd.pos.z = 0.0
                    cmd.pos.rz = drz
                    self.target_pos_down_pub.publish(cmd)
                    time.sleep(timesleep)
                    
            if all(x == 1 for x in self.task_lock):
                cmd.pos.x = 0.0
                cmd.pos.y = dy
                cmd.pos.z = 0.0
                cmd.pos.rz = drz
                self.target_pos_down_pub.publish(cmd)
                time.sleep(timesleep)
                cnt += 1
                if cnt > 60:
                    return
    

            # except Exception as e:
            #     self.get_logger().error(f"图像处理过程中出现错误 {e}")
            #     break
                # except Exception as e:
                #     self.get_logger().error(f"图像处理过程中出现错误 {e}")
                #     break
 


# 从命令行获取任务
def get_act():
    command = input("请输入任务: \n")
    parts = command.split()
    if parts[0] == 'movexyz':
        args = parts[1:]
        if len(args) == 3:
            x = float(args[0])
            y = float(args[1])
            z = float(args[2])
            dic = {
                "name": "movexyz",
                "params": {
                    "x": x,
                    "y": y,
                    "z": z
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None

    elif parts[0] == 'throw_golf':
        args = parts[1:]
        if len(args) == 2:
            dy = float(args[0])
            depth = float(args[1])
            dic = {
                "name": "throw_golf",
                "params": {
                    "dy": dy,
                    "depth": depth,
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None

    elif parts[0] == 'movexy':
        args = parts[1:]
        if len(args) == 2:
            x = float(args[0])
            y = float(args[1])
            dic = {
                "name": "movexy",
                "params": {
                    "x": x,
                    "y": y,
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'movex':
        args = parts[1:]
        if len(args) == 1:
            x = float(args[0])
            dic = {
                "name": "movex",
                "params": {
                    "x": x,
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'movey':
        args = parts[1:]
        if len(args) == 1:
            y = float(args[0])
            dic = {
                "name": "movey",
                "params": {
                    "y": y,
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'movez':
        args = parts[1:]
        if len(args) == 1:
            z = float(args[0])
            dic = {
                "name": "movez",
                "params": {
                    "z": z,
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'moverz':
        args = parts[1:]
        if len(args) == 1:
            rz = float(args[0])
            dic = {
                "name": "moverz",
                "params": {
                    "rz": rz,
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'setz':
        args = parts[1:]
        if len(args) == 1:
            z = float(args[0])
            dic = {
                "name": "setz",
                "params": {
                    "z": z,
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'setrz':
        args = parts[1:]
        if len(args) == 1:
            rz = float(args[0])
            dic = {
                "name": "setrz",
                "params": {
                    "rz": rz,
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'pow':
        args = parts[1:]
        if len(args) == 1:
            a = int(args[0])
            dic = {
                "name": "pow",
                "params": a
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'mtty':
        args = parts[1:]
        if len(args) == 2:
            y = float(args[0])
            z = float(args[1])
            dic = {
                "name": "mtty",
                "params": {
                    "y": y,
                    "z": z
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'mttxf':
        args = parts[1:]
        if len(args) == 2:
            dx = float(args[0])
            dy = float(args[1])
            dic = {
                "name": "mttxf",
                "params": {
                    "dx": dx,
                    "dy": dy
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'mttz':
        args = parts[1:]
        if len(args) == 2:
            y = float(args[0])
            z = float(args[1])
            dic = {
                "name": "mttz",
                "params": {
                    "y": y,
                    "z": z
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'mttzxy':
        args = parts[1:]
        if len(args) == 3:
            dz = float(args[0])
            dx = float(args[1])
            dy = float(args[2])
            dic = {
                "name": "mttzxy",
                "params": {
                    "dz": dz,
                    "dx": dx,
                    "dy": dy
                }
            }
            return dic
        else:
            print("指令格式不正确")
    elif parts[0] == 'setp':
        if len(parts) == 1:
            dic = {
                "name": "setp",
                "params": {}
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'back':
        if len(parts) == 1:
            dic = {
                "name": "back",
                "params": {}
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'backy':
        if len(parts) == 1:
            dic = {
                "name": "backy",
                "params": {}
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'search':
        args = parts[1:]
        if len(args) == 2:
            name = args[0]
            cam = args[1]
            dic = {
                "name": "search",
                "params": {
                    "name": name,
                    "cam": cam
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'line':
        args = parts[1:]
        if len(args) == 1:
            ys_dep = float(args[0])
            dic = {
                "name": "line",
                "params": {
                    "ys_dep": ys_dep
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None 
        
    elif parts[0] == 'mttpos':
        args = parts[1:]
        if len(args) == 5:
            x = float(args[0])
            y = float(args[1])
            z = float(args[2])
            rz = float(args[3])
            dy = float(args[4])
            dic = {
                "name": "mttpos",
                "params": {
                    "x": x,
                    "y": y,
                    "z": z,
                    "rz": rz,
                    "dy": dy
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
        
    elif parts[0] == 'mttzpos':
        args = parts[1:]
        if len(args) == 4:
            x = float(args[0])
            y = float(args[1])
            z = float(args[2])
            dy = float(args[3])
            dic = {
                "name": "mttzpos",
                "params": {
                    "x": x,
                    "y": y,
                    "z": z,
                    "dy": dy
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None

    elif parts[0] == 'mttpos_amend':
        args = parts[1:]
        if len(args) == 5:
            x = float(args[0])
            y = float(args[1])
            z = float(args[2])
            rz = float(args[3])
            dy = float(args[4])
            dic = {
                "name": "mttpos_amend",
                "params": {
                    "x": x,
                    "y": y,
                    "z": z,
                    "rz": rz,
                    "dy": dy
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
        
    elif parts[0] == 'mttzpos_amend':
        args = parts[1:]
        if len(args) == 4:
            x = float(args[0])
            y = float(args[1])
            z = float(args[2])
            dy = float(args[3])
            dic = {
                "name": "mttzpos_amend",
                "params": {
                    "x": x,
                    "y": y,
                    "z": z,
                    "dy": dy
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    
    elif parts[0] == 'graball':
        args = parts[1:]
        if len(args) == 6:
            color       =   str(args[0])
            depth       =   float(args[1])
            timeout     =   float(args[2])
            pr          =   float(args[3])
            k           =   float(args[4])
            step_time   =   float(args[5])           
            dic = {
                "name": "graball",
                "params": {
                    "color": color,
                    "depth": depth,
                    "timeout": timeout,
                    "pr": pr,
                    "k" : k,
                    "step_time":step_time
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'thrball':
        args = parts[1:]
        if len(args) == 4:
            pr        =   float(args[0])
            timeout   =   float(args[1])
            k         =   float(args[2])
            step_time =   float(args[3])  
            dic = {
                "name": "thrball",
                "params": {
                    "pr": pr,
                    "timeout": timeout,
                    "k" : k,
                    "step_time":step_time
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None  
    elif parts[0] == 'pass_door':
        args = parts[1:]
        if len(args) == 1:
            num       =   int(args[0])
            dic = {
                "name": "pass_door",
                "params": {
                    "num":num
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'led':
        args = parts[1:]
        if len(args) == 2:
            led0 = int(args[0])
            led1 = int(args[1])
            dic = {
                "name": "led",
                "params": {
                    "led0": led0,
                    "led1": led1
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    
    elif parts[0] == 'delay':
        args = parts[1:]
        if len(args) == 1:
            t = float(args[0])
            dic = {
                "name": "delay",
                "params": t
            }
            return dic
        else:
            print("指令格式不正确")
            return None

    elif parts[0] == 'grab_golf':
        args = parts[1:]
        if len(args) == 5:
            kind = str(args[0])
            dx = float(args[1])
            dy = float(args[2])
            down_depth = float(args[3])
            up_depth = float(args[4])
            dic = {
                "name": "grab_golf",
                "params": {
                    "kind": kind,
                    "dx": dx,
                    "dy": dy,
                    "down_depth": down_depth,
                    "up_depth": up_depth
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    
    elif parts[0] == 'line_qd':
        args = parts[1:]
        if len(args) == 0:
            dic = {
                "name": "line_qd",
                "params": {}  # 无参数时为空字典
            }
            return dic
        else:
            print("指令格式不正确，line_qd不需要参数")
            return None 
        
    elif parts[0] == 'endfloat':
        args = parts[1:]
        if len(args) == 0:
            dic = {
                "name": "endfloat",
                "params": {}  # 无参数时为空字典
            }
            return dic
        else:
            print("指令格式不正确，line_qd不需要参数")
            return None 
    
    elif parts[0] == 'put_t':
        args = parts[1:]
        if len(args) == 3:
            num = int(args[0])
            dy = float(args[1])
            dz = float(args[2])
            dic = {
                "name": "put_t",
                "params": {
                    "num": num,
                    "dy": dy,
                    "dz": dz
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
        
    elif parts[0] == 'strike_ball':
        args = parts[1:]
        if len(args) == 1:
            num = int(args[0])
            dic = {
                "name": "strike_ball",
                "params": {
                    "num": num
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    
    elif parts[0] == 'strike_ball3':
        args = parts[1:]
        if len(args) == 3:
            num1 = int(args[0])
            num2 = int(args[1])
            num3 = int(args[2])
            dic = {
                "name": "strike_ball3",
                "params": {
                    "num1": num1,
                    "num2": num2,
                    "num3": num3
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None

    else:
        print("无效指令")
        return None
    
    


# 读取并从文件中加载任务队列
def load_actions(path):
    with open(path, 'r', encoding='utf-8') as f:
        dic = json.load(f)
    return dic

# 保存任务队列


def save_actions(dic, path):
    print("所保存的任务:\n"+str(dic)+"\n")
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(dic, f)

# 普通模式


def commom_loop(node, opt):
    list = load_actions(opt.data_path[0]+"uv_tasks.json")

    node.start()
    num = 0

    for act in list["tasks"]:
        node.get_logger().info(
            f"Info:=========执行第 {num:d} 步任务: " + act["name"] + "=========")
        node.run(act)
        node.get_logger().info(
            f"Info:=========执行第 {num:d} 步任务: " + act["name"] + "=========")
        num += 1
    
    node.end()

# 调试模式


def debug_loop(node: CoreNode, opt):
    node.get_logger().info("Info:进入调试模式")

    tasklist = load_actions(opt.data_path[0]+"uv_tasks.json")
     
    help_txt = ""
    with open(opt.data_path[0]+"help.txt", 'r', encoding='utf-8') as f:
        s = f.read()
        help_txt = s

    node.start()

    while True:
        node.get_logger().info(f"Info:等待指令输入")
        command = input("请输入调试指令: \n")
        parts = command.split()
        if parts[0] == 'help':
            print(help_txt)
        elif parts[0] == 'act':
            act = get_act()

            if act == None:
                node.get_logger().info(f"Warn:非法任务")
            else:
                node.get_logger().info(
                    f"Info:=========执行任务: " + act["name"] + "=========")
                node.run(act)
                node.get_logger().info(
                    f"Info:=========完成任务: " + act["name"] + "=========")
        elif parts[0] == "task":
            if parts[1] == "run":
                if len(parts[1:]) == 1:
                    num = 0
                    for act in tasklist["tasks"]:
                        node.get_logger().info(
                            f"Info:=========执行第 {num:d} 步任务: " + act["name"] + "=========")
                        node.run(act)
                        node.get_logger().info(
                            f"Info:=========完成第 {num:d} 步任务: " + act["name"] + "=========")
                        num += 1
                elif len(parts[1:]) == 2:
                    num = int(parts[parts.index('run') + 1])
                    if num > len(tasklist["tasks"]) - 1 or num < 0:
                        node.get_logger().info("Warn: 所请求任务不在列表范围内！")
                    else:
                        n = num
                        for act in tasklist["tasks"][num:]:
                            node.get_logger().info(
                                f"Info:=========执行第 {num:d} 步任务: " + act["name"] + "=========")
                            node.run(act)
                            node.get_logger().info(
                                f"Info:=========完成第 {num:d} 步任务: " + act["name"] + "=========")
                            n += 1
                else:
                    node.get_logger().info("Warn:非法指令")
            elif parts[1] == "runonly":
                if len(parts[1:]) == 2:
                    num = int(parts[parts.index('runonly') + 1])
                    if num > len(tasklist["tasks"]) - 1 or num < 0:
                        node.get_logger().info("Warn: 所请求任务不在列表范围内！")
                    else:
                        act = tasklist["tasks"][num]
                        node.get_logger().info(
                            "Info:=========执行任务: " + act["name"] + "=========")
                        node.run(act)
                        node.get_logger().info(
                            "Info:=========完成任务: " + act["name"] + "=========")
                else:
                    node.get_logger().info("Warn:非法指令")
            elif parts[1] == "add":
                if len(parts[1:]) == 2:
                    num = int(parts[parts.index('add') + 1])
                    if num > len(tasklist["tasks"]) - 1 or num < 0:
                        node.get_logger().info("Warn: 所请求任务不在列表范围内！")
                    else:
                        act = get_act()
                        if act == None:
                            node.get_logger().info("非法任务")
                        else:
                            tasklist["tasks"].insert(num, act)
                            save_actions(
                                tasklist, opt.data_path[0]+"uv_tasks.json")
                elif len(parts[1:]) == 1:
                    act = get_act()
                    if act == None:
                        node.get_logger().info("非法任务")
                    else:
                        tasklist["tasks"].append(act)
                        save_actions(
                            tasklist, opt.data_path[0]+"uv_tasks.json")
                else:
                    node.get_logger().info("Warn:非法指令")
            elif parts[1] == "del":
                if len(parts[1:]) == 2:
                    num = int(parts[parts.index('del') + 1])
                    if num > len(tasklist["tasks"]) - 1 or num < 0:
                        node.get_logger().info("Warn: 所请求任务不在列表范围内！")
                    else:
                        tasklist["tasks"].pop(num)
                        save_actions(
                            tasklist, opt.data_path[0]+"uv_tasks.json")
                else:
                    node.get_logger().info("Warn:非法指令")
            elif parts[1] == "clear":
                tasklist["tasks"] = []
                save_actions(tasklist, opt.data_path[0]+"uv_tasks.json")
            elif parts[1] == "list":
                node.get_logger().info("Info: 打印任务列表")
                num = 0
                for i in tasklist["tasks"]:
                    print("编号:   " + str(num))
                    print("任务名: "+i["name"])
                    print("参数:   " + str(i["params"]))
                    num += 1
            elif parts[1] == "mod":
                if len(parts[1:]) == 2:
                    num = int(parts[parts.index('mod') + 1])
                    if num > len(tasklist["tasks"]) - 1 or num < 0:
                        node.get_logger().info("Warn: 所请求任务不在列表范围内！")
                    else:
                        act = get_act()
                        if act == None:
                            node.get_logger().info("非法任务")
                        else:
                            tasklist["tasks"][num] = act
                            save_actions(
                                tasklist, opt.data_path[0]+"uv_tasks.json")
                else:
                    node.get_logger().info("Warn:非法指令")
            else:
                node.get_logger().info("Warn:非法指令")


def main(args=None):

    # 加载参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--front-topic', nargs='+', type=str, default=[
                        'front_cam/rectified'], help='前视摄像头')
    parser.add_argument('--down-topic', nargs='+', type=str, default=[
                        'down_cam/rectified'], help='下视摄像头')
    parser.add_argument('--data-path', nargs='+', type=str, default=[
                        '/home/nvidia/Workspace/Cruise/datas/'], help='PID参数路径')
    parser.add_argument('--debug', nargs='+', type=bool,
                        default=False, help='PID参数路径')

    opt = parser.parse_args()

    rclpy.init(args=args)  # 初始化rclpy

    if opt.debug:
        node = CoreNode("uv_automaton_debug", opt)  # 新建一个节点
        thread_debug = threading.Thread(
            target=debug_loop, args=(node, opt))  # 创建调度线程
        thread_debug.start()
    else:
        node = CoreNode("uv_automaton", opt)  # 新建一个节点
        thread_common = threading.Thread(
            target=commom_loop, args=(node, opt))    # 创建调度线程
        thread_common.start()

    node.get_logger().info("节点与调度线程成功启动")

    rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+Z）
    rclpy.shutdown()  # 关闭rclpy
