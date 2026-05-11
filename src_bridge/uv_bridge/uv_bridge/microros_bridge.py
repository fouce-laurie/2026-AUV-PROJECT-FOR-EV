#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Micro-ROS 桥接节点 (传话筒节点)
功能：在原始的 uv_msgs 话题和新的 micro-ROS zit6_interfaces 话题之间进行双向转换

主要职责：
1. 上行转换：将上位机的控制命令（uv_msgs）转换为 micro-ROS 命令（zit6_interfaces）
2. 下行转换：将 micro-ROS 的状态反馈转换为上位机可识别的格式
3. 心跳管理：维持与 micro-ROS 的心跳连接，监控系统状态
4. 启动协调：处理系统启动、解锁流程
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

# 原始消息类型（上位机使用）
from uv_msgs.msg import (
    RobotMotionController,    # 机器人运动控制器
    RobotDeviceManager,       # 机器人设备管理器
    RobotAxis,                # 机器人轴控制（6-DOF）
    PidParams,                # PID参数
    PidControllers,           # PID控制器集合
    PidControllersState,      # PID控制器状态
    TargetPosDown,            # 目标位置下发
    ServoSet,                 # 舵机设置
    LedControllers,           # LED控制器
    ImuData                   # IMU数据
)

# Micro-ROS 消息类型（单片机使用）
from zit6_interfaces.msg import ZitSetpoint, ZitStatus, ZitPid, ZitPidStatus
from std_msgs.msg import Float32MultiArray, UInt32, UInt8, Float32, Bool


class MicroRosBridge(Node):
    """
    Micro-ROS 桥接节点类

    作用：充当"传话筒"，在两种不同的通信协议之间进行翻译
    - 原始协议：基于串口的自定义帧格式（已废弃）
    - 新协议：基于 ROS2 话题的标准通信
    """

    def __init__(self):
        super().__init__('microros_bridge')

        # ========== 状态变量 ==========
        self.last_agx_heartbeat_time = time.time()  # 上次发送心跳的时间
        self.heartbeat_seq = 0                       # 心跳序列号，用于消息追踪
        self.arm_mode = 0                            # 解锁模式：0=DEFAULT（需要导航就绪），3=REMOTE（遥控模式）
        self.is_armed = False                        # 系统是否已解锁（允许推力输出）
        self.last_motion_data = None                 # 最后一次的运动控制数据，用于状态合并

        # ========== QoS 配置 ==========
        # 可靠传输：用于关键命令和状态，保证消息不丢失
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 尽力传输：用于高频数据，允许丢包以降低延迟
        self.best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ========== 初始化发布者和订阅者 ==========
        self._setup_publishers()
        self._setup_subscribers()

        # ========== 定时器 ==========
        # 心跳定时器：10Hz，向 micro-ROS 发送心跳
        self.heartbeat_timer = self.create_timer(0.1, self.heartbeat_callback)

        # 状态监控定时器：1Hz，监控系统状态
        self.status_timer = self.create_timer(1.0, self.status_monitor_callback)

        self.get_logger().info('Micro-ROS 桥接节点已初始化')

    def _setup_publishers(self):
        """
        设置所有发布者
        分为两类：
        1. 发送到 micro-ROS 的命令话题
        2. 发送到上位机的状态话题
        """
        # ========== 命令发布者（发送到 micro-ROS）==========
        # 控制设定点：位置/速度/推力控制命令
        self.zit_setpoint_pub = self.create_publisher(
            ZitSetpoint, '/zit6/cmd/setpoint', self.reliable_qos)

        # PID参数配置：在线调整PID参数
        self.zit_pid_pub = self.create_publisher(
            ZitPid, '/zit6/cmd/pid', self.reliable_qos)

        # AGX心跳：告诉单片机上位机还活着
        self.agx_heartbeat_pub = self.create_publisher(
            UInt32, '/zit6/cmd/agxhbt', self.reliable_qos)

        # 惯导命令：控制DVL开关、惯导重启等
        self.ins_cmd_pub = self.create_publisher(
            UInt8, '/zit6/cmd/ins', self.reliable_qos)

        # 舵机命令：控制舵机角度
        self.servo_cmd_pub = self.create_publisher(
            Float32, '/zit6/cmd/servo', self.reliable_qos)

        # 灯光命令：控制LED灯
        self.light_cmd_pub = self.create_publisher(
            UInt8, '/zit6/cmd/light', self.reliable_qos)

        # ========== 状态发布者（发送到上位机）==========
        # 运动控制器状态：包含位置、速度、推力等综合信息
        self.motion_controller_pub = self.create_publisher(
            RobotMotionController, 'motion_controller', 10)

        # 设备管理器状态：包含传感器、电池等设备信息
        self.device_manager_pub = self.create_publisher(
            RobotDeviceManager, 'device_manager', 10)

    def _setup_subscribers(self):
        """
        设置所有订阅者
        分为两类：
        1. 订阅原始话题（来自上位机）
        2. 订阅 micro-ROS 状态话题（来自单片机）
        """
        # ========== 订阅原始话题（来自上位机）==========
        # 开环推力控制：直接控制推进器输出
        self.create_subscription(
            RobotAxis, 'openloop_thrust', self.openloop_thrust_callback, 10)

        # 目标位置控制：位置环控制
        self.create_subscription(
            TargetPosDown, 'target_pos_down', self.target_pos_down_callback, 10)

        # 目标速度控制：速度环控制
        self.create_subscription(
            TargetPosDown, 'Target_speed_down', self.target_speed_down_callback, 10)

        # PID参数设置：调整PID参数
        self.create_subscription(
            PidParams, 'pid_params_set', self.pid_params_set_callback, 10)

        # PID控制器状态设置：开关PID控制器
        self.create_subscription(
            PidControllersState, 'pid_controllers_set', self.pid_controllers_set_callback, 10)

        # 舵机控制
        self.create_subscription(
            ServoSet, 'servo_control', self.servo_control_callback, 10)

        # LED控制
        self.create_subscription(
            LedControllers, 'led_controllers', self.led_controllers_callback, 10)

        # DVL设置
        self.create_subscription(
            ImuData, 'dvl_set', self.dvl_set_callback, 10)

        # ========== 订阅 micro-ROS 状态话题（来自单片机）==========
        # 系统状态：解锁状态、控制层级、错误标志等
        self.create_subscription(
            ZitStatus, '/zit6/state/status', self.zit_status_callback, self.reliable_qos)

        # PID状态：当前PID参数反馈
        self.create_subscription(
            ZitPidStatus, '/zit6/state/pid_status', self.zit_pid_status_callback, self.reliable_qos)

        # 位置反馈：[x, y, z, yaw]
        self.create_subscription(
            Float32MultiArray, '/zit6/state/pos', self.zit_pos_callback, self.best_effort_qos)

        # 速度反馈：[vx, vy, vz, vyaw]
        self.create_subscription(
            Float32MultiArray, '/zit6/state/vel', self.zit_vel_callback, self.best_effort_qos)

        # 推力反馈：[f0, f1, f2, f3]
        self.create_subscription(
            Float32MultiArray, '/zit6/state/thr', self.zit_thr_callback, self.best_effort_qos)

        # micro-ROS心跳：确认单片机还活着
        self.create_subscription(
            UInt32, '/zit6/state/zithbt', self.zit_heartbeat_callback, self.reliable_qos)

    # ========== 原始话题回调函数（上位机 → micro-ROS）==========

    def openloop_thrust_callback(self, msg):
        """
        开环推力控制回调
        将 6-DOF 推力命令转换为 4-DOF ZitSetpoint（FORCE模式）

        原始：RobotAxis (x, y, z, rx, ry, rz)
        转换：ZitSetpoint (x, y, z, yaw) with control_key=FORCE

        注意：原系统支持6自由度，新系统只支持4自由度（X,Y,Z,Yaw）
        """
        setpoint = ZitSetpoint()
        setpoint.control_key = 0x02  # FORCE模式（bits 0-1 = 2）
        setpoint.type_mask = 0x0F    # 控制所有轴（X, Y, Z, Yaw）
        setpoint.x = msg.x           # X轴推力
        setpoint.y = msg.y           # Y轴推力
        setpoint.z = msg.z           # Z轴推力
        setpoint.yaw = msg.rz        # Yaw力矩（使用rz）
        setpoint.seq = self.heartbeat_seq  # 序列号

        self.zit_setpoint_pub.publish(setpoint)
        self.get_logger().debug(f'发布开环推力: [{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}, {msg.rz:.2f}]')

    def target_pos_down_callback(self, msg):
        """
        目标位置控制回调
        将目标位置转换为 ZitSetpoint（POS模式）

        支持坐标系选择：
        - cs=0: 世界坐标系（world frame）
        - cs=1: 机体坐标系（body frame）
        """
        setpoint = ZitSetpoint()
        setpoint.control_key = 0x00  # POS模式（bits 0-1 = 0）

        # 根据坐标系设置标志位
        if msg.cs == 1:
            setpoint.control_key |= 0x10  # 设置机体坐标系标志（bit 4）

        setpoint.type_mask = 0x0F  # 控制所有轴
        setpoint.x = msg.pos.x     # 目标X位置
        setpoint.y = msg.pos.y     # 目标Y位置
        setpoint.z = msg.pos.z     # 目标Z位置
        setpoint.yaw = msg.pos.rz  # 目标Yaw角度
        setpoint.seq = self.heartbeat_seq

        self.zit_setpoint_pub.publish(setpoint)
        self.get_logger().debug(f'发布目标位置: [{msg.pos.x:.2f}, {msg.pos.y:.2f}, {msg.pos.z:.2f}, {msg.pos.rz:.2f}]')

    def target_speed_down_callback(self, msg):
        """
        目标速度控制回调
        将目标速度转换为 ZitSetpoint（VEL模式）

        注意：原系统可能发送6个速度分量，但这里只使用X和Y
        """
        setpoint = ZitSetpoint()
        setpoint.control_key = 0x01  # VEL模式（bits 0-1 = 1）
        setpoint.type_mask = 0x03    # 只控制X和Y轴
        setpoint.x = msg.pos.x       # X轴速度
        setpoint.y = msg.pos.y       # Y轴速度
        setpoint.z = 0.0             # Z轴速度（不使用）
        setpoint.yaw = 0.0           # Yaw角速度（不使用）
        setpoint.seq = self.heartbeat_seq

        self.zit_setpoint_pub.publish(setpoint)
        self.get_logger().debug(f'发布目标速度: [{msg.pos.x:.2f}, {msg.pos.y:.2f}]')

    def pid_params_set_callback(self, msg):
        """
        PID参数设置回调
        将原始的PID参数转换为 ZitPid 消息

        轴映射：
        - 原系统：0:X, 1:Y, 2:Z, 3:Yaw, 4:Vx, 5:Vy（6个轴）
        - 新系统：0:X, 1:Y, 2:Z, 3:Yaw（4个轴，每个轴有位置环和速度环）
        """
        # 轴索引映射表
        axis_map = {0: 0, 1: 1, 2: 2, 3: 3, 4: 0, 5: 1}

        if msg.axis in axis_map:
            pid_msg = ZitPid()
            pid_msg.axis = axis_map[msg.axis]

            # 判断是位置环还是速度环
            # 原系统：0-3是位置环，4-5是速度环
            pid_msg.is_pos_ring = (msg.axis < 4)

            # PID参数
            pid_msg.kp = msg.kp
            pid_msg.ki = msg.ki
            pid_msg.kd = msg.kd
            pid_msg.i_limit = msg.i_limit if hasattr(msg, 'i_limit') else 0.0
            pid_msg.out_limit = msg.out_limit if hasattr(msg, 'out_limit') else 0.0

            # 规划器参数（仅位置环使用）
            pid_msg.max_v = 0.0
            pid_msg.max_a = 0.0

            self.zit_pid_pub.publish(pid_msg)
            self.get_logger().info(f'发布PID参数: 轴{pid_msg.axis}, 位置环={pid_msg.is_pos_ring}')

    def pid_controllers_set_callback(self, msg):
        """
        PID控制器状态设置回调
        原系统可以单独开关每个轴的PID，新系统通过control_level统一切换
        这里仅记录日志，实际控制通过解锁机制实现
        """
        self.get_logger().debug('收到PID控制器状态变更请求')

    def servo_control_callback(self, msg):
        """
        舵机控制回调
        直接转发舵机角度到 micro-ROS
        """
        servo_msg = Float32()
        servo_msg.data = msg.angle
        self.servo_cmd_pub.publish(servo_msg)
        self.get_logger().debug(f'发布舵机角度: {msg.angle:.2f}')

    def led_controllers_callback(self, msg):
        """
        LED控制回调
        原系统有两个LED（led0, led1），新系统只有一个控制字节
        这里使用led0作为主控制
        """
        led_msg = UInt8()
        led_msg.data = msg.led0  # 使用led0作为主LED控制
        self.light_cmd_pub.publish(led_msg)
        self.get_logger().debug(f'发布LED状态: {msg.led0}')

    def dvl_set_callback(self, msg):
        """
        DVL设置回调
        将DVL开关状态转换为INS命令

        INS命令定义：
        1: 开启DVL
        2: 关闭DVL
        3: 重启惯导
        4: 重置位置
        5: 设置初始位置
        """
        ins_msg = UInt8()
        ins_msg.data = 1 if msg.dvl else 2  # DVL开启=1，关闭=2
        self.ins_cmd_pub.publish(ins_msg)
        self.get_logger().debug(f'发布INS命令: {ins_msg.data}')

    # ========== micro-ROS 状态回调函数（micro-ROS → 上位机）==========

    def zit_status_callback(self, msg):
        """
        系统状态回调
        接收 micro-ROS 的系统状态，更新本地状态并转换为原始格式

        ZitStatus 包含：
        - is_armed: 是否解锁
        - arm_mode: 解锁模式
        - control_level: 控制层级（NONE/POS/VEL/FORCE）
        - ins_state: 惯导状态
        - navigation_ready: 导航是否就绪
        - forces: 推力输出
        - error_flags: 错误标志
        """
        # 更新本地状态
        self.is_armed = msg.is_armed
        self.arm_mode = msg.arm_mode

        # 初始化运动控制数据（如果还没有）
        if self.last_motion_data is None:
            self.last_motion_data = RobotMotionController()

        # 根据控制层级更新PID状态
        if msg.control_level == ZitStatus.LEVEL_POS:
            # 位置环模式：X, Y, Z轴PID开启
            self.last_motion_data.pidstate.x = 1
            self.last_motion_data.pidstate.y = 1
            self.last_motion_data.pidstate.z = 1
        elif msg.control_level == ZitStatus.LEVEL_VEL:
            # 速度环模式：Vx, Vy PID开启
            self.last_motion_data.pidstate.vx = 1
            self.last_motion_data.pidstate.vy = 1
        else:
            # 其他模式：关闭所有PID
            self.last_motion_data.pidstate.x = 0
            self.last_motion_data.pidstate.y = 0
            self.last_motion_data.pidstate.z = 0
            self.last_motion_data.pidstate.vx = 0
            self.last_motion_data.pidstate.vy = 0

        # 更新推力数据（4个推进器）
        for i in range(min(4, len(msg.forces))):
            if i < len(self.last_motion_data.thrust.thrust):
                self.last_motion_data.thrust.thrust[i] = msg.forces[i]

        # 更新IMU状态
        self.last_motion_data.imu.mode = msg.ins_state  # 惯导状态
        self.last_motion_data.imu.dvl = 1 if msg.navigation_ready else 0  # DVL状态

        self.get_logger().debug(f'收到状态: 解锁={msg.is_armed}, 控制层级={msg.control_level}')

    def zit_pid_status_callback(self, msg):
        """
        PID状态回调
        接收 micro-ROS 的PID参数反馈
        这里仅记录日志，实际参数由上位机的参数管理节点处理
        """
        self.get_logger().debug('收到PID状态反馈')

    def zit_pos_callback(self, msg):
        """
        位置反馈回调
        接收 micro-ROS 的位置数据 [x, y, z, yaw]
        更新到运动控制器消息并发布
        """
        if self.last_motion_data is None:
            self.last_motion_data = RobotMotionController()

        if len(msg.data) >= 4:
            # 更新机器人位置
            self.last_motion_data.pos.x = msg.data[0]
            self.last_motion_data.pos.y = msg.data[1]
            self.last_motion_data.pos.z = msg.data[2]
            self.last_motion_data.pos.rz = msg.data[3]

            # 同时更新IMU位置（保持一致性）
            self.last_motion_data.imu.pos.x = msg.data[0]
            self.last_motion_data.imu.pos.y = msg.data[1]
            self.last_motion_data.imu.pos.z = msg.data[2]
            self.last_motion_data.imu.pos.rz = msg.data[3]

            # 发布更新后的运动控制器数据
            self.motion_controller_pub.publish(self.last_motion_data)

    def zit_vel_callback(self, msg):
        """
        速度反馈回调
        接收 micro-ROS 的速度数据 [vx, vy, vz, vyaw]
        更新到IMU速度字段
        """
        if self.last_motion_data is None:
            self.last_motion_data = RobotMotionController()

        if len(msg.data) >= 4:
            self.last_motion_data.imu.spd.x = msg.data[0]
            self.last_motion_data.imu.spd.y = msg.data[1]
            self.last_motion_data.imu.spd.z = msg.data[2]
            self.last_motion_data.imu.spd.rz = msg.data[3]

    def zit_thr_callback(self, msg):
        """
        推力反馈回调
        接收 micro-ROS 的推力数据
        更新到推力数组
        """
        if self.last_motion_data is None:
            self.last_motion_data = RobotMotionController()

        for i in range(min(len(msg.data), len(self.last_motion_data.thrust.thrust))):
            self.last_motion_data.thrust.thrust[i] = msg.data[i]

    def zit_heartbeat_callback(self, msg):
        """
        micro-ROS 心跳回调
        接收单片机的心跳，确认通信正常
        """
        self.get_logger().debug(f'收到 micro-ROS 心跳: {msg.data}')

    # ========== 定时器回调函数 ==========

    def heartbeat_callback(self):
        """
        心跳定时器回调（10Hz）
        定期向 micro-ROS 发送心跳，维持连接

        心跳数据包含解锁模式，用于告诉单片机当前的工作模式
        """
        hb_msg = UInt32()
        hb_msg.data = self.arm_mode  # 发送解锁模式作为心跳数据
        self.agx_heartbeat_pub.publish(hb_msg)
        self.heartbeat_seq += 1  # 序列号递增

        # 自动解锁逻辑：心跳建立3秒后尝试解锁
        if not self.is_armed and (time.time() - self.last_agx_heartbeat_time) > 3.0:
            self.get_logger().info('心跳建立完成，系统准备解锁')
            self.last_agx_heartbeat_time = time.time()

    def status_monitor_callback(self):
        """
        状态监控定时器回调（1Hz）
        定期输出系统状态，便于监控
        """
        if self.is_armed:
            self.get_logger().info(f'系统已解锁 (模式={self.arm_mode})', throttle_duration_sec=5.0)
        else:
            self.get_logger().info('系统未解锁', throttle_duration_sec=5.0)


def main(args=None):
    """
    主函数
    初始化ROS2，创建桥接节点并运行
    """
    rclpy.init(args=args)
    node = MicroRosBridge()

    try:
        rclpy.spin(node)  # 保持节点运行
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
