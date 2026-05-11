# -*- coding: utf-8 -*-
"""
水下机器人硬件管理单元 (无串口版本)
Underwater Vehicle Hardware Management Unit (No Serial - for micro-ROS bridge)

功能：原有上位机节点的无串口版本，配合 micro-ROS bridge 使用

主要职责：
1. 参数管理：加载、保存和广播 PID 参数和推力曲线
2. 话题转发：接收上层应用的控制命令，发布到标准话题
3. 状态接收：接收来自桥接节点的反馈数据
4. 兼容性：保持与原有系统相同的话题接口

与原版本的区别：
- 移除了串口通信部分（Serial.TtyReader/TtyWriter）
- 不再直接发送串口帧到单片机
- 所有通信通过 ROS2 话题，由 microros_bridge 节点转发
"""

import rclpy
from rclpy.node import Node
import time
import threading
import argparse
import sys

from rclpy.utilities import remove_ros_args

# 消息类型定义
from uv_msgs.msg import RobotDeviceManager      # 机器人设备管理器
from uv_msgs.msg import RobotMotionController   # 机器人运动控制器
from uv_msgs.msg import PidParams               # PID参数
from uv_msgs.msg import PidControllers          # PID控制器集合
from uv_msgs.msg import PidControllersState     # PID控制器状态
from uv_msgs.msg import ThrustCurve             # 推力曲线
from uv_msgs.msg import ThrustCurves            # 推力曲线集合
from uv_msgs.msg import RobotAxis               # 机器人轴控制
from uv_msgs.msg import ServoSet                # 舵机设置
from uv_msgs.msg import TargetPosDown           # 目标位置下发
from uv_msgs.msg import ImuData                 # IMU数据
from uv_msgs.msg import LedControllers          # LED控制器
from uv_msgs.msg import MagnetController        # 电磁铁控制器

# 控制算法模块
from uv_control_py import Pid    # PID参数管理
from uv_control_py import Curve  # 推力曲线管理


class CoreNodeNoSerial(Node):
    """
    核心节点 - 无串口版本

    作用：
    1. 保持与原有系统相同的话题接口，确保上层应用无需修改
    2. 管理 PID 参数和推力曲线的文件存储
    3. 定期广播参数供其他节点使用
    4. 接收来自 microros_bridge 的反馈数据

    工作流程：
    上层应用 → 本节点 → microros_bridge → micro-ROS (单片机)
                ↑                              ↓
                └──────── 状态反馈 ←───────────┘
    """

    def __init__(self, name, pid_path, curve_path):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s (无串口版本)!" % name)

        # ========== 数据结构初始化 ==========
        self.MotionController = RobotMotionController()  # 运动控制器数据
        self.DeviceManager = RobotDeviceManager()        # 设备管理器数据

        # ========== 话题发布者 ==========
        # 发布运动控制器状态（来自 micro-ROS 的反馈）
        self.motion_controller_pub = self.create_publisher(
            RobotMotionController, "motion_controller", 10)

        # 发布设备管理器状态（来自 micro-ROS 的反馈）
        self.device_manager_pub = self.create_publisher(
            RobotDeviceManager, "device_manager", 10)

        # 发布 PID 控制器参数（定期广播）
        self.pid_controllers_pub = self.create_publisher(
            PidControllers, "pid_controllers", 10)

        # 发布推力曲线参数（定期广播）
        self.curves_pub = self.create_publisher(
            ThrustCurves, "curves", 10)

        # ========== 话题订阅者（接收上层应用的命令）==========
        # 这些话题会被 microros_bridge 节点转发到 micro-ROS

        # 开环推力控制：直接控制推进器输出
        self.create_subscription(
            RobotAxis, 'openloop_thrust', self.openloop_thrust_callback, 10)

        # 舵机控制：控制舵机角度
        self.create_subscription(
            ServoSet, 'servo_control', self.servo_control_callback, 10)

        # PID参数设置：调整PID参数
        self.create_subscription(
            PidParams, 'pid_params_set', self.pid_params_set_callback, 10)

        # PID控制器状态设置：开关PID控制器
        self.create_subscription(
            PidControllersState, 'pid_controllers_set', self.pid_controllers_set_callback, 10)

        # 目标位置设置：位置环控制
        self.create_subscription(
            TargetPosDown, 'target_pos_down', self.target_pos_down_callback, 10)

        # DVL设置：控制DVL开关
        self.create_subscription(
            ImuData, 'dvl_set', self.dvl_set_callback, 10)

        # 推力曲线设置：更新推力曲线
        self.create_subscription(
            ThrustCurve, 'thrust_curve_set', self.thrust_curve_set_callback, 10)

        # 目标速度设置：速度环控制
        self.create_subscription(
            TargetPosDown, 'Target_speed_down', self.target_speed_down_callback, 10)

        # LED控制：控制LED灯
        self.create_subscription(
            LedControllers, 'led_controllers', self.led_controllers_callback, 10)

        # 电磁铁控制：控制电磁铁开关
        self.create_subscription(
            MagnetController, 'magnet_controller', self.magnet_controller_callback, 10)

        # ========== 订阅来自 bridge 的反馈数据 ==========
        # 接收 microros_bridge 转换后的状态数据

        # 运动控制器反馈
        self.create_subscription(
            RobotMotionController, 'motion_controller', self.motion_controller_feedback_callback, 10)

        # 设备管理器反馈
        self.create_subscription(
            RobotDeviceManager, 'device_manager', self.device_manager_feedback_callback, 10)

        # ========== 参数管理器初始化 ==========
        # 注意：不传入 writer（串口写入器），因为不再使用串口通信
        self.pid = Pid.PID(None, pid_path)      # PID参数管理器
        self.curve = Curve.CURVE(None, curve_path)  # 推力曲线管理器

        self.get_logger().info("节点初始化完成，等待 micro-ROS bridge 连接...")

    # ========== 命令回调函数（接收上层应用的命令）==========
    # 这些回调函数只记录日志，实际转发由 microros_bridge 完成

    def openloop_thrust_callback(self, data):
        """
        开环推力命令回调
        接收上层应用的推力命令，发布到话题供 bridge 转发
        """
        self.get_logger().debug(f"收到开环推力命令: x={data.x:.2f}, y={data.y:.2f}, z={data.z:.2f}")
        # 不做处理，由 microros_bridge 节点订阅并转发

    def servo_control_callback(self, data):
        """
        舵机控制命令回调
        接收舵机角度设置命令
        """
        self.get_logger().debug(f"收到舵机控制命令: angle={data.angle:.2f}")
        # 由 microros_bridge 转发

    def led_controllers_callback(self, data):
        """
        LED控制命令回调
        接收LED灯光控制命令
        """
        self.get_logger().debug(f"收到LED控制命令: led0={data.led0}, led1={data.led1}")
        # 由 microros_bridge 转发

    def magnet_controller_callback(self, data):
        """
        电磁铁控制命令回调
        接收电磁铁开关控制命令
        """
        self.get_logger().debug(f"收到电磁铁控制命令: state={data.state}")
        # 由 microros_bridge 转发

    def pid_params_set_callback(self, data):
        """
        PID参数设置回调
        接收PID参数设置命令，保存到文件

        工作流程：
        1. 接收新的PID参数
        2. 保存到本地文件（持久化）
        3. 发布到话题供 microros_bridge 转发到单片机
        """
        # 更新内存中的PID参数
        self.pid.topicrec(data)

        # 保存到文件（JSON格式）
        self.pid.filesave()
        self.get_logger().info("已保存PID参数信息到文件")

        # 参数会通过话题发布，由 microros_bridge 转发到单片机

    def thrust_curve_set_callback(self, data):
        """
        推力曲线设置回调
        接收推力曲线设置命令，保存到文件

        推力曲线：描述推进器输入（PWM）与输出（推力）的关系
        """
        # 更新内存中的推力曲线
        self.curve.topicrec(data)

        # 保存到文件
        self.curve.filesave()
        self.get_logger().info("已保存推力曲线到文件")

    def target_pos_down_callback(self, data):
        """
        目标位置设置回调
        接收目标位置命令（位置环控制）
        """
        self.get_logger().debug(f"收到目标位置: x={data.pos.x:.2f}, y={data.pos.y:.2f}, z={data.pos.z:.2f}")
        # 由 microros_bridge 转发

    def pid_controllers_set_callback(self, data):
        """
        PID控制器状态设置回调
        接收PID控制器开关命令
        """
        self.get_logger().debug("收到PID控制器状态设置")
        # 由 microros_bridge 转发

    def dvl_set_callback(self, data):
        """
        DVL设置回调
        接收DVL（多普勒测速仪）开关命令
        """
        self.get_logger().debug(f"收到DVL设置: dvl={data.dvl}")
        # 由 microros_bridge 转发

    def target_speed_down_callback(self, data):
        """
        目标速度设置回调
        接收目标速度命令（速度环控制）
        """
        self.get_logger().debug(f"收到目标速度: x={data.pos.x:.2f}, y={data.pos.y:.2f}")
        # 由 microros_bridge 转发

    # ========== 反馈回调函数（接收来自 bridge 的状态）==========

    def motion_controller_feedback_callback(self, data):
        """
        运动控制器反馈回调
        接收来自 microros_bridge 的运动控制器状态

        包含：
        - 位置信息（pos）
        - 速度信息（imu.spd）
        - 推力信息（thrust）
        - PID状态（pidstate）
        """
        self.MotionController = data
        # 可以在这里添加额外的处理逻辑
        # 例如：数据记录、异常检测等

    def device_manager_feedback_callback(self, data):
        """
        设备管理器反馈回调
        接收来自 microros_bridge 的设备管理器状态

        包含：
        - 漏水检测
        - 电池电压
        - 温度、湿度
        - 舵机角度
        - LED状态
        """
        self.DeviceManager = data
        # 可以在这里添加额外的处理逻辑

    def parameters_init(self):
        """
        参数初始化
        从文件加载PID参数和推力曲线

        注意：这里只加载参数，不下发到硬件
        参数下发由 microros_bridge 在收到参数设置命令时完成
        """
        self.get_logger().info("PID参数已从文件加载")
        self.get_logger().info("推力曲线已从文件加载")


def ParameterBroadcast(node):
    """
    参数广播线程函数
    定期广播PID参数和推力曲线供其他节点使用

    广播频率：
    - PID参数：20Hz
    - 推力曲线：20Hz
    - 交替发送，总频率约10Hz

    用途：
    - 供上位机界面显示当前参数
    - 供其他节点获取参数信息
    - 确保参数同步
    """
    while rclpy.ok():
        # 发布PID控制器参数
        node.pid_controllers_pub.publish(node.pid.pid)
        time.sleep(0.05)  # 50ms

        # 发布推力曲线参数
        node.curves_pub.publish(node.curve.curves)
        time.sleep(0.05)  # 50ms


def main(args=None):
    """
    主函数
    初始化节点并启动参数广播线程
    """
    # 过滤ROS参数
    ros_args = remove_ros_args(sys.argv)

    # 解析命令行参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--pid-path', nargs='+', type=str, default=[
                        '/home/nvidia/Workspace/Cruise/datas/pid_parameters.json'],
                        help='PID参数文件路径')
    parser.add_argument('--curve-path', nargs='+', type=str, default=[
                        '/home/nvidia/Workspace/Cruise/datas/thrust_cureves.json'],
                        help='推力曲线文件路径')
    opt = parser.parse_args(ros_args[1:])

    # 初始化ROS2
    rclpy.init(args=args)

    # 创建节点
    node = CoreNodeNoSerial("uv_core_no_serial",
                            opt.pid_path[0], opt.curve_path[0])

    # 加载参数
    node.parameters_init()

    # 创建参数广播线程
    thread_parameter_broadcast = threading.Thread(
        target=ParameterBroadcast, args=(node,))
    thread_parameter_broadcast.daemon = True  # 设置为守护线程
    thread_parameter_broadcast.start()

    # 运行节点
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
