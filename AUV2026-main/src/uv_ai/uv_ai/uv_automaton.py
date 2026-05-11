import rclpy
from rclpy.node import Node
import time
import os
import math
import heapq
import termios
import struct
import threading
import json
import argparse
import datetime
import traceback
from pathlib import Path
import math
import heapq
try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None
from uv_control_py import Pid
from uv_control_py.CoordinateSystem import CoordinateSystems, MotionState, Cs_Back, Cs_Move, AngleCorrect

from uv_msgs.msg import RobotDeviceManager  # 机器人设备管理器
from uv_msgs.msg import RobotMotionController  # 机器人运动控制器

from uv_msgs.msg import PidParams#巡线的pid参数

from uv_msgs.msg import Yolov8
from uv_msgs.msg import LedControllers
from uv_msgs.msg import MagnetController

from zit6_interfaces.msg import ZitSetpoint, ZitStatus, ZitPid
from std_msgs.msg import UInt32, Float32, UInt8, Float32MultiArray

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
    "x": 0.2,
    "y": 0.05,
    "z": 0.2,
    "rz": 9.0
}

AllowedError = {  # 位置容许误差
    "x": 0.1,
    "y": 0.1,
    "z": 0.1,
    "rz": 5.0
}

# Yaw-to-distance normalization: at the tolerance boundary, rz degrees of
# yaw error contributes the same scalar as x meters of position error.
_YAW_TO_DIST = AllowedError["rz"] / AllowedError["x"]  # ~16.67

class KalmanFilter:
    def __init__(self, dt=0.1, process_noise=0.1, measurement_noise=1.0):
        """
        初始化卡尔曼滤波器（恒定加速度模型，适应非匀速运动）
        :param dt: 采样时间间隔（秒）
        :param process_noise: 过程噪声协方差（越大越信任测量值）
        :param measurement_noise: 测量噪声协方差（越大越信任预测值）
        """
        # 状态向量 [位置, 速度, 加速度]
        self.x = np.array([[0.0], [0.0], [0.0]])  # 初始状态（x方向）
        self.y = np.array([[0.0], [0.0], [0.0]])  # 初始状态（y方向）
        
        # 状态转移矩阵（3x3，基于恒定加速度模型）
        # 位置 = 前一位置 + 速度*dt + 0.5*加速度*dt²
        # 速度 = 前一速度 + 加速度*dt
        # 加速度 = 前一加速度（假设加速度变化率为噪声）
        self.F = np.array([
            [1, dt, 0.5 * dt**2],
            [0, 1, dt],
            [0, 0, 1]
        ])
        
        # 观测矩阵（1x3，仅观测位置）
        self.H = np.array([[1, 0, 0]])
        
        # 过程噪声协方差矩阵Q（反映模型不确定性，非匀速时增大加速度项）
        self.Q = process_noise * np.array([
            [dt**4/4, dt**3/2, dt**2/2],
            [dt**3/2, dt**2, dt],
            [dt**2/2, dt, 1]
        ])
        
        # 测量噪声协方差矩阵R（反映观测噪声）
        self.R = np.array([[measurement_noise]])
        
        # 状态协方差矩阵P（初始不确定性）
        self.Px = np.eye(3) * 1000  # x方向
        self.Py = np.eye(3) * 1000  # y方向

    def predict(self):
        """预测步骤：基于当前状态和运动模型预测下一状态"""
        # 预测状态
        self.x = self.F @ self.x
        self.y = self.F @ self.y
        
        # 更新状态协方差（预测不确定性）
        self.Px = self.F @ self.Px @ self.F.T + self.Q
        self.Py = self.F @ self.Py @ self.F.T + self.Q

    def update(self, z_x, z_y):
        """更新步骤：用观测值修正预测值"""
        # 处理x方向观测
        y_x = z_x - self.H @ self.x  # 残差（观测-预测）
        S_x = self.H @ self.Px @ self.H.T + self.R  # 残差协方差
        K_x = self.Px @ self.H.T @ np.linalg.inv(S_x)  # 卡尔曼增益
        
        self.x = self.x + K_x @ y_x  # 更新状态
        self.Px = (np.eye(3) - K_x @ self.H) @ self.Px  # 更新协方差
        
        # 处理y方向观测
        y_y = z_y - self.H @ self.y
        S_y = self.H @ self.Py @ self.H.T + self.R
        K_y = self.Py @ self.H.T @ np.linalg.inv(S_y)
        
        self.y = self.y + K_y @ y_y
        self.Py = (np.eye(3) - K_y @ self.H) @ self.Py

    def get_filtered_position(self):
        """返回滤波后的位置（x和y）"""
        return self.x[0, 0], self.y[0, 0]


class AStarPlanner:
    def __init__(self, resolution=0.5, safe_radius=2.0):
        self.resolution = resolution
        self.safe_radius = safe_radius

    def plan(self, start_x, start_y, goal_x, goal_y, obstacles):
        def to_grid(x, y):
            return int(round(x / self.resolution)), int(round(y / self.resolution))
            
        def to_world(gx, gy):
            return gx * self.resolution, gy * self.resolution

        sgx, sgy = to_grid(start_x, start_y)
        ggx, ggy = to_grid(goal_x, goal_y)
        
        # 限制搜索范围为起点终点包围盒加上15米的余量，防止过于庞大
        min_x = min(start_x, goal_x) - 15.0
        max_x = max(start_x, goal_x) + 15.0
        min_y = min(start_y, goal_y) - 15.0
        max_y = max(start_y, goal_y) + 15.0
        
        min_gx, min_gy = to_grid(min_x, min_y)
        max_gx, max_gy = to_grid(max_x, max_y)
        
        obs_grids = set()
        padding = int(math.ceil(self.safe_radius / self.resolution))
        for (ox, oy) in obstacles:
            ogx, ogy = to_grid(ox, oy)
            # 添加以障碍物点为中心的膨胀区块
            for dx in range(-padding, padding + 1):
                for dy in range(-padding, padding + 1):
                    if math.hypot(dx*self.resolution, dy*self.resolution) < self.safe_radius:
                        obs_grids.add((ogx + dx, ogy + dy))
                        
        if (sgx, sgy) in obs_grids:
            # 如果起点已经在膨胀区，可能需要增加容错
            pass

        open_set = []
        heapq.heappush(open_set, (0.0, sgx, sgy))
        came_from = {}
        g_score = {(sgx, sgy): 0.0}
        
        def heuristic(x1, y1, x2, y2):
            return math.hypot(x2 - x1, y2 - y1) * self.resolution
            
        while open_set:
            _, cx, cy = heapq.heappop(open_set)
            
            # 如果进入终点小范围内认为到达
            if math.hypot(cx - ggx, cy - ggy) <= 2: 
                path = [(cx, cy)]
                while (cx, cy) in came_from:
                    cx, cy = came_from[(cx, cy)]
                    path.append((cx, cy))
                path.reverse()
                
                # 找到前视距离大于一定值的路径点作为下一个临时路点
                for px, py in path:
                    if math.hypot(px - sgx, py - sgy) * self.resolution > 1.5:
                        return to_world(px, py)
                if len(path) > 1:
                    return to_world(path[-1][0], path[-1][1])
                return goal_x, goal_y
                
            for dx, dy in [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
                nx, ny = cx + dx, cy + dy
                if min_gx <= nx <= max_gx and min_gy <= ny <= max_gy:
                    # 如果不是障碍物
                    if (nx, ny) in obs_grids:
                        continue
                    
                    # 距离计算：对角线是 1.414，直行是 1.0
                    move_cost = math.hypot(dx, dy) * self.resolution
                    tentative_g = g_score[(cx, cy)] + move_cost
                    
                    if tentative_g < g_score.get((nx, ny), float('inf')):
                        came_from[(nx, ny)] = (cx, cy)
                        g_score[(nx, ny)] = tentative_g
                        f_score = tentative_g + heuristic(nx, ny, ggx, ggy)
                        heapq.heappush(open_set, (f_score, nx, ny))
        
        return goal_x, goal_y

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
        self.yolov8_data_front_pos = Yolov8()
        self.yolov8_data_down_pos = Yolov8()
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
        #前视相机坐标偏置 (NED body: X前, Y右, Z下)
        self.front_cam.base.vector.x = 320.0/1000
        self.front_cam.base.vector.y = 0/1000
        self.front_cam.base.vector.z = 0.0/1000
        self.front_cam.base.vector.rx = -90.0
        self.front_cam.base.extract()
        #下视相机偏置 (NED body: X前, Y右, Z下)
        self.down_cam.base.vector.x = 0.0/1000
        self.down_cam.base.vector.y = 0.0/1000
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
        
        # 初始化日志功能
        self.init_logger()
        
        self.start_pos = CoordinateSystems()
        
        self.front_cam_Image_data = None
        self.down_cam_Image_data = None
        self.front_cam_left_Image_data = None
        self.front_cam_right_Image_data = None
        self.down_cam_left_Image_data = None
        self.down_cam_right_Image_data = None
        self.segment_img = None

        # 话题发布 (ZIT6)
        self.setpoint_pub = self.create_publisher(ZitSetpoint, '/zit6/cmd/setpoint', 10)
        self.pid_pub = self.create_publisher(ZitPid, '/zit6/cmd/pid', 10)
        self.servo_pub = self.create_publisher(Float32, '/zit6/cmd/servo', 10)
        self.light_pub = self.create_publisher(UInt8, '/zit6/cmd/light', 10)
        #创建话题发布 line_patrol_img , 定义消息类型为Image
        self.line_patrol_img_pub = self.create_publisher(
            Image, 'line_patrol_img', 10)

        self._last_setpoint = None  # 记录最后发送的 setpoint（度制）
        self._target_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}  # 目标位置（绝对坐标）

        # 话题接收 (ZIT6 状态)
        self.create_subscription(ZitStatus, '/zit6/state/status', self._on_zit6_status, 10)
        self.create_subscription(Float32MultiArray, '/zit6/state/pos', self._on_zit6_pos, 10)
        self.create_subscription(Float32MultiArray, '/zit6/state/vel', self._on_zit6_vel, 10)
        self.create_subscription(Float32MultiArray, '/zit6/state/thr', self._on_zit6_thr, 10)
        
        # 创建话题接收 front_cam/rectified ，定义其中的消息类型为 Image
        self.create_subscription(
            Image, opt.front_topic[0], self.front_cam_callback, 10)
        # 创建话题接收 down_cam/rectified ，定义其中的消息类型为 Image
        self.create_subscription(
            Image, opt.down_topic[0], self.down_cam_callback, 10)
        #创建话题接收 front_cam/rectified/left,定义其中的消息类型为 Image
        self.create_subscription(
            Image, 'front_cam/rectified/left', self.front_cam_left_callback, 10)
        #创建话题接收 front_cam/rectified/right,定义其中的消息类型为 Image
        self.create_subscription(
            Image, 'front_cam/rectified/right', self.front_cam_right_callback, 10)
        #创建话题接收 down_cam/rectified/left,定义其中的消息类型为 Image
        self.create_subscription(
            Image, 'down_cam/rectified/left', self.down_cam_left_callback, 10)
        #创建话题接收 down_cam/rectified/right,定义其中的消息类型为 Image
        self.create_subscription(
            Image, 'down_cam/rectified/right', self.down_cam_right_callback, 10)
        
        self.create_subscription(
            PidParams, 'track_pid_parameter', self.track_pid_parameter_callback, 10)
        
        # 创建话题接收 uv_detect ，定义其中的消息类型为 Yolov8
        self.create_subscription(
            Yolov8, 'uv_detect_down', self.yolov8_down_callback, 10)
        
        self.create_subscription(
            Yolov8, 'uv_detect_front', self.yolov8_front_callback, 10)
        
        # 新增订阅 uv_position 发送的定位消息
        self.create_subscription(
            Yolov8, 'uv_detect_front_pos', self.yolov8_front_pos_callback, 10)
        self.create_subscription(
            Yolov8, 'uv_detect_down_pos', self.yolov8_down_pos_callback, 10)
        
        #创建话题接收 binary_segment_img , 定义消息类型为Image，用于接收二值化后的分割结果
        self.create_subscription(
            Image, 'binary_segment_img', self.segment_img_callback, 10)

        # 服务请求
        # 创建服务请求 detect_request_client ，定义其中的消息类型为 RobotAxis , 请求服务 uv_detect_srv
        self.detect_request_client = self.create_client(
            DetectRequest, "uv_detect_srv")

        self.magnet.state = 1
        self._publish_magnet(self.magnet.state)

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
    
    def front_cam_right_callback(self, data):
        self.front_cam_right_Image_data = data
    
    def down_cam_left_callback(self, data):
        self.down_cam_left_Image_data = data
    
    def down_cam_right_callback(self, data):
        self.down_cam_right_Image_data = data
    
    # 更新巡线图像
    def segment_img_callback(self, data):
        self.segment_img = data

    #更新目标检测结果
    def yolov8_down_callback(self, data):
        self.yolov8_data_down = data
    
    def yolov8_front_callback(self, data):
        self.yolov8_data_front = data
    
    def yolov8_front_pos_callback(self, data):
        self.yolov8_data_front_pos = data
        
    def yolov8_down_pos_callback(self, data):
        self.yolov8_data_down_pos = data

    def init_logger(self):
        """初始化日志记录器，创建日志文件夹和文件"""
        try:
            # 日志目录：datas/logs/
            log_dir = os.path.join(self.opt.data_path[0], "logs")
            if not os.path.exists(log_dir):
                os.makedirs(log_dir)

            # 文件名：automaton_YYYYMMDD_HHMMSS.jsonl
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_file_path = os.path.join(log_dir, f"automaton_{timestamp}.jsonl")

            # 创建定时器：以 1Hz 频率记录状态快照
            self.log_timer = self.create_timer(1.0, self.log_state)

            # 包装 logger.info，让每条 info 自动写入日志文件并附带 pose
            logger = self.get_logger()
            original_info = logger.info

            def info_with_log(msg):
                original_info(msg)
                self.log_write(str(msg))

            logger.info = info_with_log

            logger.info(f"日志系统初始化成功，文件路径: {self.log_file_path}")
        except Exception as e:
            self.get_logger().error(f"日志系统初始化失败: {str(e)}")

    def log_write(self, msg):
        """将一条消息写入日志文件，附带当前 pose"""
        if not hasattr(self, 'log_file_path'):
            return
        try:
            entry = {
                "type": "log",
                "timestamp": time.time(),
                "datetime": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                "msg": msg,
                "pose": {
                    "x": round(self.robot.base.vector.x, 3),
                    "y": round(self.robot.base.vector.y, 3),
                    "z": round(self.robot.base.vector.z, 3),
                    "rx": round(self.robot.base.vector.rx, 2),
                    "ry": round(self.robot.base.vector.ry, 2),
                    "rz": round(self.robot.base.vector.rz, 2)
                }
            }
            with open(self.log_file_path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(entry, ensure_ascii=False) + "\n")
        except Exception:
            pass

    def log_state(self, current_task=None):
        """记录当前系统状态到日志文件"""
        if not hasattr(self, 'log_file_path'):
            return
            
        try:
            state = {
                "type": "state",
                "timestamp": time.time(),
                "datetime": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                "task_info": {
                    "current_task": current_task if current_task else (getattr(self, 'current_task_name', 'idle')),
                },
                "pose": {
                    "x": round(self.robot.base.vector.x, 3),
                    "y": round(self.robot.base.vector.y, 3),
                    "z": round(self.robot.base.vector.z, 3),
                    "rx": round(self.robot.base.vector.rx, 2),
                    "ry": round(self.robot.base.vector.ry, 2),
                    "rz": round(self.robot.base.vector.rz, 2)
                },
                "target_pose": {
                    "x": round(self.MotionController.tpos.x, 3),
                    "y": round(self.MotionController.tpos.y, 3),
                    "z": round(self.MotionController.tpos.z, 3),
                    "rz": round(self.MotionController.tpos.rz, 2)
                },
                "objects": {
                    "front": [],
                    "down": [],
                    "front_pos": [],
                    "down_pos": []
                }
            }

            # 辅助函数：提取 Yolov8 消息中的物体信息
            def extract_objects(msg):
                objs = []
                for idx, count in enumerate(msg.state):
                    if count > 0 and idx < len(msg.targets):
                        t = msg.targets[idx]
                        objs.append({
                            "class_id": idx,
                            "count": int(count),
                            "x": round(t.tpos_inworld.x, 3),
                            "y": round(t.tpos_inworld.y, 3),
                            "z": round(t.tpos_inworld.z, 3),
                            "px": t.tpos_inpic.x,
                            "py": t.tpos_inpic.y
                        })
                return objs

            state["objects"]["front"] = extract_objects(self.yolov8_data_front)
            state["objects"]["down"] = extract_objects(self.yolov8_data_down)
            state["objects"]["front_pos"] = extract_objects(self.yolov8_data_front_pos)
            state["objects"]["down_pos"] = extract_objects(self.yolov8_data_down_pos)

            with open(self.log_file_path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(state, ensure_ascii=False) + "\n")
        except Exception as e:
            self.get_logger().debug(f"log_state error: {e}")
    
    # 更新机器人位置数据
    # ==================== ZIT6 状态回调 ====================

    def _on_zit6_status(self, s: ZitStatus):
        ins_map = {0: 0x00, 1: 0x01, 2: 0x02, 3: 0x04, 4: 0x04, 5: 0x05}
        self.MotionController.imu.mode = ins_map.get(s.ins_state, 0x00)
        self.MotionController.thrust.thrust[0] = s.forces[0]
        self.MotionController.thrust.thrust[1] = s.forces[1]
        self.MotionController.thrust.thrust[2] = s.forces[2]
        self.MotionController.thrust.thrust[5] = s.forces[3]
        self._update_tpos_inbase()

    def _on_zit6_pos(self, msg: Float32MultiArray):
        self.MotionController.pos.x = msg.data[0]
        self.MotionController.pos.y = msg.data[1]
        self.MotionController.pos.z = msg.data[2]
        self.MotionController.pos.rz = math.degrees(msg.data[3])
        self.robot.base.vector.x = msg.data[0]
        self.robot.base.vector.y = msg.data[1]
        self.robot.base.vector.z = msg.data[2]
        self.robot.base.vector.rz = self.MotionController.pos.rz
        self.robot.base.extract()
        # 首次收到位置时，用当前位置初始化目标
        if self._last_setpoint is None:
            self._target_pos = {
                'x': msg.data[0], 'y': msg.data[1],
                'z': msg.data[2], 'rz': self.MotionController.pos.rz}
        self._update_tpos_inbase()

    def _on_zit6_vel(self, msg: Float32MultiArray):
        self.MotionController.imu.spd.x = msg.data[0]
        self.MotionController.imu.spd.y = msg.data[1]
        self.MotionController.imu.spd.z = msg.data[2]
        self.MotionController.imu.spd.rz = math.degrees(msg.data[3])

    def _on_zit6_thr(self, msg: Float32MultiArray):
        self.MotionController.thrust.thrust[0] = msg.data[0]
        self.MotionController.thrust.thrust[1] = msg.data[1]
        self.MotionController.thrust.thrust[2] = msg.data[2]
        self.MotionController.thrust.thrust[5] = msg.data[3]

    def _update_tpos_inbase(self):
        if self._last_setpoint is None:
            return
        tgt = self._target_pos
        dx = tgt['x'] - self.MotionController.pos.x
        dy = tgt['y'] - self.MotionController.pos.y
        dz = tgt['z'] - self.MotionController.pos.z
        dyaw = tgt['rz'] - self.MotionController.pos.rz
        yaw_rad = math.radians(self.MotionController.pos.rz)
        self.MotionController.tpos_inbase.x = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
        self.MotionController.tpos_inbase.y = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
        self.MotionController.tpos_inbase.z = dz
        self.MotionController.tpos_inbase.rz = dyaw

    def _send_setpoint(self, control_key, type_mask, x, y, z, rz_deg):
        msg = ZitSetpoint()
        msg.control_key = control_key
        msg.type_mask = type_mask
        msg.x = x
        msg.y = y
        msg.z = z
        msg.yaw = math.radians(rz_deg)
        self.setpoint_pub.publish(msg)
        self._last_setpoint = {'x': x, 'y': y, 'z': z, 'rz': rz_deg}

        # 维护绝对目标位置（供 move_wait 使用）
        if control_key & 0x20:
            # 增量模式：bridge 会做 body→world 旋转，这里用旋转后的值维护目标
            if type_mask & 0x03:
                yaw_rad = math.radians(self.MotionController.pos.rz)
                cy = math.cos(yaw_rad)
                sy = math.sin(yaw_rad)
                wx = cy * x - sy * y
                wy = sy * x + cy * y
            else:
                wx, wy = x, y
            if type_mask & 0x01:
                self._target_pos['x'] += wx
            if type_mask & 0x02:
                self._target_pos['y'] += wy
            if type_mask & 0x04:
                self._target_pos['z'] += z
            if type_mask & 0x08:
                self._target_pos['rz'] += rz_deg
        else:
            # 绝对模式：直接设置
            if type_mask & 0x01:
                self._target_pos['x'] = x
            if type_mask & 0x02:
                self._target_pos['y'] = y
            if type_mask & 0x04:
                self._target_pos['z'] = z
            if type_mask & 0x08:
                self._target_pos['rz'] = rz_deg

    def _publish_servo(self, angle):
        msg = Float32()
        msg.data = max(0.0, min(1.0, float(angle)))
        self.servo_pub.publish(msg)

    def _publish_led(self, led0, led1=0):
        msg = UInt8()
        msg.data = int(led0)
        self.light_pub.publish(msg)

    def _publish_magnet(self, state):
        self.get_logger().warn('MagnetController not supported by ZIT6, ignoring')

    # 等待移动至指定位置
    def move_wait(self):
        sleeptime = 0.1
        # 在 settle delay 之前捕获指令位移（世界系，L2 范数旋转不变）
        dx = self._target_pos['x'] - self.MotionController.pos.x
        dy = self._target_pos['y'] - self.MotionController.pos.y
        dz = self._target_pos['z'] - self.MotionController.pos.z
        drz = self._target_pos['rz'] - self.MotionController.pos.rz
        initial_dist = math.sqrt(dx**2 + dy**2 + dz**2 + (drz / _YAW_TO_DIST)**2)
        time.sleep(0.5)  # Wait for coordinates to update properly
        cnt = 0
        limit_cnt = math.sqrt(initial_dist) * 5 / sleeptime
        if limit_cnt < 5/sleeptime:
            limit_cnt = 5/sleeptime
        elif limit_cnt > 300/sleeptime:
            limit_cnt = 300/sleeptime
        time.sleep(sleeptime)
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

            if cnt > limit_cnt:  # 超时退出
                return False

            cnt += 1
            time.sleep(sleeptime)

    def _do_swing_cycle(self, angle, speed):
        """执行一个完整的摆头周期: 原位 -> 左 -> 右 -> 原位"""
        base_rz = self.MotionController.pos.rz
        rel_targets = [angle, -angle, 0.0]
        for rel_rz in rel_targets:
            self._send_setpoint(0x00, 0x0F,
                self.MotionController.pos.x,
                self.MotionController.pos.y,
                self.MotionController.pos.z,
                base_rz + rel_rz)
            a = self.move_wait()
            if not a:
                self.get_logger().info("Warn: 摆头移动超时！")
            time.sleep(0.2)
            
    def stop(self):
        self.get_logger().info("======机器人停止======")
        self._send_setpoint(0x00, 0x0F,
            self.MotionController.pos.x,
            self.MotionController.pos.y,
            self.MotionController.pos.z + 0.06,
            self.MotionController.pos.rz)
        time.sleep(5)
        re = self.move_wait()
        if re:
            self.get_logger().info("Info: 停止完成")
        else:
            self.get_logger().info("Warn: 停止超时！")

    def movez(self, z):
        if self.robot.base.vector.z + z < -1.0:
            self.get_logger().info("Warn: 深度设置超出范围")
            z = -self.robot.base.vector.z

        self.get_logger().info("======开始移动======")
        step_cnt = int(abs(z)/Step["z"]) + 1
        self.get_logger().info(f"Info: 开始调整深度, 需要调整 {step_cnt:d} 次")
        cnt = 0
        while True:
            cnt += 1
            if abs(z) <= Step["z"]:
                self.get_logger().info("Info: 最终深度调整开始")
                self._send_setpoint(0x20, 0x04, 0.0, 0.0, z, 0.0)
                re = self.move_wait()
                if re:
                    self.get_logger().info("Info: 最终深度调整完成")
                else:
                    self.get_logger().info("Warn: 最终深度调整超时！")
                break
            else:
                step = Step["z"] if z > 0 else -Step["z"]
                self._send_setpoint(0x20, 0x04, 0.0, 0.0, step, 0.0)
                z -= step
                time.sleep(0.2)
        self.get_logger().info("======移动结束======")

    def fast_movez(self, z):
        if self.robot.base.vector.z + z < 0:
            self.get_logger().info("Warn: 深度设置超出范围")
        else:
            self.get_logger().info("======开始移动======")
            self._send_setpoint(0x20, 0x04, 0.0, 0.0, z, 0.0)
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

    def move_world_step(self, dx, dy, dz=0.0):
        self.get_logger().info(f"====== 开始在世界坐标系步进移动 (dx={dx}, dy={dy}, dz={dz}) ======")
        start_x = self.MotionController.pos.x
        start_y = self.MotionController.pos.y
        start_z = self.MotionController.pos.z

        step_max = max(Step.get("x", 0.1), Step.get("y", 0.1))
        dist = math.hypot(dx, dy)
        steps = int(math.ceil(dist / step_max)) if dist > 0 else 1
        if abs(dz) > 0:
            steps_z = int(math.ceil(abs(dz) / Step.get("z", 0.1)))
            steps = max(steps, steps_z)

        self.get_logger().info(f"Info: 开始步进移动, 预计需要调整 {steps} 次")

        for i in range(1, steps + 1):
            if not rclpy.ok(): break
            ratio = float(i) / steps
            self._send_setpoint(0x00, 0x0F,
                start_x + dx * ratio,
                start_y + dy * ratio,
                start_z + dz * ratio,
                self.MotionController.pos.rz)
            s = self.move_wait()
            if not s:
                self.get_logger().info(f"Warn: 第 {i} 步移动等待超时，继续下一步...")
        self.get_logger().info("====== 世界坐标系步进移动结束 ======")

    def movex(self, x, dx = Step["x"]):
        if x > 10:
            self.get_logger().info("Warn: 前后位置设置超出范围！")
        else:
            self.get_logger().info("======开始移动======")
            step_cnt = int(abs(x)/dx) + 1
            self.get_logger().info(f"Info: 开始调整前后位置, 需要调整 {step_cnt:d} 次")
            while True:
                if abs(x) <= Step["x"]:
                    self.get_logger().info("Info: 最终前后位置调整开始")
                    self._send_setpoint(0x20, 0x01, x, 0.0, 0.0, 0.0)
                    s = self.move_wait()
                    if s:
                        self.get_logger().info("Info: 最终前后位置调整完成")
                    else:
                        self.get_logger().info("Warn: 最终前后位置调整超时！")
                    break
                else:
                    step = Step["x"] if x > 0 else -Step["x"]
                    self._send_setpoint(0x20, 0x01, step, 0.0, 0.0, 0.0)
                    x -= step
                    time.sleep(0.1)
            self.get_logger().info("======移动结束======")

    def fast_movex(self, x):
        if x > 10:
            self.get_logger().info("Warn: 前后位置设置超出范围！")
        else:
            self.get_logger().info("======开始移动======")
            self._send_setpoint(0x20, 0x01, x, 0.0, 0.0, 0.0)
            self.get_logger().info("======移动结束======")
    


    def movey(self, y):
        if y > 10:
            self.get_logger().info("Warn: 横向位置设置超出范围！")
        else:
            self.get_logger().info("======开始移动======")
            step_cnt = int(abs(y)/Step["y"]) + 1
            self.get_logger().info(f"Info: 开始调整横向位置, 需要调整 {step_cnt:d} 次")
            while True:
                if abs(y) <= Step["y"]:
                    self.get_logger().info("Info: 最终横向位置调整开始")
                    self._send_setpoint(0x20, 0x02, 0.0, y, 0.0, 0.0)
                    s = self.move_wait()
                    if s:
                        self.get_logger().info("Info: 最终横向位置调整完成")
                    else:
                        self.get_logger().info("Warn: 最终横向位置调整超时！")
                    break
                else:
                    step = Step["y"] if y > 0 else -Step["y"]
                    self._send_setpoint(0x20, 0x02, 0.0, step, 0.0, 0.0)
                    y -= step
                    time.sleep(0.1)
            self.get_logger().info("======移动结束======")

    def fast_movey(self, y):
        if y > 10:
            self.get_logger().info("Warn: 横向位置设置超出范围！")
        else:
            self.get_logger().info("======开始移动======")
            self._send_setpoint(0x20, 0x02, 0.0, y, 0.0, 0.0)
            self.get_logger().info("======移动结束======")

    # 移动 rz 的相对位移
    def moverz(self, rz):
        if rz > 180 or rz < -180:
            self.get_logger().info("Warn: 角度设置超出范围！")
            rz = AngleCorrect(rz)
            self.get_logger().info(f"Info: 角度等效为 {rz:.2f}°")

        self.get_logger().info("======开始旋转======")
        step_cnt = int(abs(rz)/Step["rz"]) + 1
        self.get_logger().info(f"Info: 开始调整角度, 需要调整 {step_cnt:d} 次")
        while True:
            if abs(rz) <= Step["rz"]:
                self.get_logger().info("Info: 最终角度调整开始")
                self._send_setpoint(0x20, 0x08, 0.0, 0.0, 0.0, rz)
                s = self.move_wait()
                if s:
                    self.get_logger().info("Info: 最终角度调整完成")
                else:
                    self.get_logger().info("Warn: 最终角度调整超时！")
                break
            else:
                step = Step["rz"] if rz > 0 else -Step["rz"]
                self._send_setpoint(0x20, 0x08, 0.0, 0.0, 0.0, step)
                rz -= step
                time.sleep(0.1)
        self.get_logger().info("======旋转结束======")

    def fast_moverz(self, rz):
        if rz > 180 or rz < -180:
            self.get_logger().info("Warn: 角度设置超出范围！")
            rz = AngleCorrect(rz)
            self.get_logger().info(f"Info: 角度等效为 {rz:.2f}°")

        self.get_logger().info("======开始旋转======")
        self._send_setpoint(0x20, 0x08, 0.0, 0.0, 0.0, rz)
        self.get_logger().info("======移动结束======")

    def movexy(self, x, y):
        if x == 0 and y == 0:
            self.get_logger().info("Warn: 未设置合法位移！")
        else:
            rz = atan2(y, x)*RAD2DEG
            d = sqrt(x*x + y*y)
            self.get_logger().info(f"Info: ======转向目标点,旋转{rz:.2f}°======")
            self.moverz(rz)
            self.get_logger().info("Info: ======转向结束======")
            time.sleep(0.1)
            self.get_logger().info(f"Info: ======移动指定距离,移动{d:.2f}m======")
            self.movex(d)
            self.get_logger().info("Info: ======移动结束======")
            time.sleep(0.1)

    def movexyz(self, x, y, z):
        if z == 0:
            self.get_logger().info("Warn: 未设置合法深度位移！")
        else:
            self.movez(z)
        time.sleep(0.1)
        self.movexy(x, y)

    def setz(self, z):
        if z >= -1.0:
            self.get_logger().info("======开始调整深度======")
            self._send_setpoint(0x00, 0x0F,
                self.MotionController.pos.x,
                self.MotionController.pos.y,
                float(z),
                self.MotionController.pos.rz)
            s = self.move_wait()
            if s:
                self.get_logger().info("Info: 深度调整完成")
            else:
                self.get_logger().info("Warn: 深度调整超时！")
            self.get_logger().info("======移动结束======")
        else:
            self.get_logger().info("Warn: 深度设置错误！")

    def setx(self, x):
        self.get_logger().info("======开始调整X坐标======")
        self._send_setpoint(0x00, 0x0F,
            float(x),
            self.MotionController.pos.y,
            self.MotionController.pos.z,
            self.MotionController.pos.rz)
        s = self.move_wait()
        if s:
            self.get_logger().info("Info: X坐标调整完成")
        else:
            self.get_logger().info("Warn: X坐标调整超时！")
        self.get_logger().info("======移动结束======")

    def sety(self, y):
        self.get_logger().info("======开始调整Y坐标======")
        self._send_setpoint(0x00, 0x0F,
            self.MotionController.pos.x,
            float(y),
            self.MotionController.pos.z,
            self.MotionController.pos.rz)
        s = self.move_wait()
        if s:
            self.get_logger().info("Info: Y坐标调整完成")
        else:
            self.get_logger().info("Warn: Y坐标调整超时！")
        self.get_logger().info("======移动结束======")

    def setrz(self, rz):
        if rz > 180 or rz < -180:
            self.get_logger().info("Warn: 角度设置超出范围！")
            rz = AngleCorrect(rz)
            self.get_logger().info(f"Info: 角度等效为 {rz:.2f}°")

        self.get_logger().info("======开始调整角度======")
        self._send_setpoint(0x00, 0x0F,
            float(self.MotionController.pos.x),
            float(self.MotionController.pos.y),
            float(self.MotionController.pos.z),
            float(rz))
        s = self.move_wait()
        if s:
            self.get_logger().info("Info: 角度调整完成")
        else:
            self.get_logger().info("Warn: 角度调整超时！")
        self.get_logger().info("======移动结束======")

    def setrz_step(self, rz, drz, mode=0):
        Step["rz"] = drz
        if rz > 180 or rz < -180:
            self.get_logger().info("Warn: 角度设置超出范围！")
            rz = AngleCorrect(rz)
            self.get_logger().info(f"Info: 角度等效为 {rz:.2f}°")

        self.get_logger().info(f"======开始步进调整角度: 目标={rz:.2f}, 模式={mode}======")
        current_rz = self.MotionController.pos.rz
        diff_rz = rz - current_rz

        while diff_rz > 360: diff_rz -= 360
        while diff_rz < -360: diff_rz += 360

        if mode == 0:
            if diff_rz > 180:
                diff_rz -= 360
            elif diff_rz < -180:
                diff_rz += 360
        elif mode == 1:
            if diff_rz > 0:
                diff_rz -= 360
        elif mode == 2:
            if diff_rz < 0:
                diff_rz += 360

        self.get_logger().info(f"Info: 最终计算增量 diff_rz = {diff_rz:.2f}°")

        step_rz = Step["rz"]
        step_cnt = int(abs(diff_rz) / step_rz) + 1
        self.get_logger().info(f"Info: 开始调整角度, 需要步进 {step_cnt:d} 次")

        rem_rz = diff_rz
        while True:
            if abs(rem_rz) <= step_rz:
                self.get_logger().info("Info: 最终角度步进调整开始")
                self._send_setpoint(0x20, 0x08, 0.0, 0.0, 0.0, float(rem_rz))
                re = self.move_wait()
                if re:
                    self.get_logger().info("Info: 最终角度步进调整完成")
                else:
                    self.get_logger().info("Warn: 最终角度步进调整超时！")
                break
            else:
                delta = step_rz if rem_rz > 0 else -step_rz
                self._send_setpoint(0x20, 0x08, 0.0, 0.0, 0.0, float(delta))
                rem_rz -= delta
                time.sleep(0.2)

        self.get_logger().info("======步进角度调整结束======")



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
            rz_ = atan2(y, x)*RAD2DEG - 180
            d = sqrt(x*x + y*y)
            self.get_logger().info(f"Info: ======转向目标点,旋转{rz_:.2f}°======")
            self.moverz(rz_)
            self.get_logger().info("Info: ======转向结束======")
            time.sleep(0.1)
            self.get_logger().info(f"Info: ======移动指定距离,移动{d:.2f}m======")
            self.movex(-d)
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
            rz_ = atan2(y, x)*RAD2DEG - 180
            d = sqrt(x*x + y*y)
            self.get_logger().info(f"Info: ======转向目标点,旋转{rz_:.2f}°======")
            self.fast_moverz(rz_)
            self.get_logger().info("Info: ======转向结束======")
            time.sleep(5.0)
            self.get_logger().info(f"Info: ======移动指定距离,移动{rz:.2f}m======")
            self.fast_movex(-d)
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
            rz_ = atan2(y, x)*RAD2DEG - 180
            d = sqrt(x*x + y*y)
            self.get_logger().info(f"Info: ======转向目标点,旋转{rz_:.2f}°======")
            self.moverz(rz_)
            self.get_logger().info("Info: ======转向结束======")
            time.sleep(0.1)
            self.get_logger().info(f"Info: ======移动指定距离,移动{d:.2f}m======")
            self.movex(-d)
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
    def mtty(self, dx, dz):

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
            rz = atan2(y, x)*RAD2DEG
            d = sqrt(x*x + y*y) - dx
            self.get_logger().info(f"Info: ======转向目标点,旋转{rz:.2f}°======")
            self.moverz(rz)
            self.get_logger().info("Info: ======转向结束======")
            time.sleep(0.1)
            self.get_logger().info(f"Info: ======移动指定距离,移动{d:.2f}m======")
            self.movex(d)
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
            self.movey(-x)
            self.get_logger().info("Info: ======移动结束======")
        time.sleep(0.1)

        if y == 0:
            self.get_logger().info("Warn: 前后位移非法！")
        else:
            self.get_logger().info(f"Info: ======前后指定距离,移动{y:.2f}m======")
            self.movex(y)
        time.sleep(0.1)

        self.get_logger().info(f"Info: ======移动结束======")


    def mttxf(self,dy,dx):
        self.robot.target_inworld.vector.x = self.target["x"]
        self.robot.target_inworld.vector.y = self.target["y"]
        self.robot.target_inworld.vector.z = self.target["z"]
        self.robot.world2base()

        x = self.robot.target_inbase.vector.x
        y = self.robot.target_inbase.vector.y
        z = self.robot.target_inbase.vector.z

        self.get_logger().info(
            f"Info:需要调整的位移 x: {x:.2f} y: {y:.2f} z: {z:.2f}")
        x_target = x - dy
        self.get_logger().info(f"Info: ======横移指定距离,移动{x:.2f}m======")
        self.movey(-x_target)
        self.get_logger().info("Info: ======移动结束======")
        time.sleep(0.1)

        if y == 0:
            self.get_logger().info("Warn: 非法位移！")
        else:
            d = y - dx
            time.sleep(0.1)
            self.get_logger().info(f"Info: ======移动指定距离,移动{d:.2f}m======")
            self.movex(d)
            self.get_logger().info("Info: ======移动结束======")
        time.sleep(0.1)


    # 移动至寄存器 self.target 所指定的位置
    def mttz(self, dx, dz):

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
            rz = atan2(y, x)*RAD2DEG
            d = sqrt(x*x + y*y) - dx
            self.get_logger().info(f"Info: ======转向目标点,旋转{rz:.2f}°======")
            self.moverz(rz)
            self.get_logger().info("Info: ======转向结束======")
            time.sleep(0.1)
            self.get_logger().info(f"Info: ======移动指定距离,移动{d:.2f}m======")
            self.movex(d)
            self.get_logger().info("Info: ======移动结束======")

        if z == 0:
            self.get_logger().info("Warn: 深度位移非法！")
        else:
            self.movez(z)
        time.sleep(0.1)
    
    #移动至指定世界坐标位置
    def mttpos(self, x, y, z, rz, dx):
        self.target["x"] = x
        self.target["y"] = y
        self.target["z"] = z

        Step["y"] = 0.04
        self.mttz(dx, 0.0)
        self.setrz(rz)
        Step["y"] = 0.02
    
    #移动至指定世界坐标位置，但最后不旋转
    def mttzpos(self, x, y, z, dx):
        self.target["x"] = x
        self.target["y"] = y
        self.target["z"] = z
        Step["z"] = 0.2
        Step["y"] = 0.04
        self.mttz(dx, 0.0)
        Step["y"] = 0.02
        Step["z"] = 0.1

    #移动至指定世界坐标位置，以比赛开始位置为坐标
    def mttpos_amend(self, x, y, z, rz, dx):
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
        self.mttz(dx, 0.0)
        self.setrz(rz+self.start_pos.base.vector.rz)
        Step["y"] = 0.02
    
    #移动至指定世界坐标位置，以比赛开始位置为坐标
    def mttzpos_amend(self, x, y, z, dx):
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
        self.mttz(dx, 0.0)
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
        if s == 1:
            self._publish_servo(0.43)
        if s == 0:
            self._publish_servo(0.65)
        time.sleep(0.1)


    # 任务启动
    def start(self):
        time.sleep(5)
        #  载入当前目标值
        start_x = self.MotionController.pos.x
        start_y = self.MotionController.pos.y
        start_z = self.MotionController.pos.z
        start_rz = self.MotionController.pos.rz
        self.start_pos.base.vector.x = start_x
        self.start_pos.base.vector.y = start_y
        self.start_pos.base.vector.z = 0.0
        self.start_pos.base.vector.rz = start_rz
        self.start_pos.base.vector.ry = self.start_pos.base.vector.rx = 0.0
        self.start_pos.base.extract()
        self.get_logger().info(f"载入当前世界坐标 x:{start_x:.3f} y:{start_y:.3f} z:{start_z:.3f} rz:{start_rz:.3f}")
        #self.led(1,0)#绿灯
        #time.sleep(3.0)
        #self.led(0,1)#黄灯
        #time.sleep(3.0)
        #self.led(1,1)#红灯

        #time.sleep(3.0)
        self.led(0,0)
        self._send_setpoint(0x00, 0x0F, start_x, start_y, start_z, start_rz)
        # 机械爪加紧T插
        self.pow(0)

        # 开启PID控制器 (ZIT6 doesn't have pid_controllers_set equivalent)
        # pid = PidControllersState()
        # pid.x = pid.y = pid.z = pid.rz = 1
        # pid.rx = pid.ry = 0
        self.get_logger().warn("pid_controllers_set_pub skipped: ZIT6 has no equivalent")
        time.sleep(0.1)

    # 任务结束
    def end(self):
        # 关闭PID控制器 (ZIT6 doesn't have pid_controllers_set equivalent)
        # pid = PidControllersState()
        # pid.x = pid.y = pid.z = pid.rz = 0
        # pid.rx = pid.ry = 0
        self.get_logger().warn("pid_controllers_set_pub skipped: ZIT6 has no equivalent")
        time.sleep(0.1)

    # 采集数据专用：螺旋探测
    def spiral_data_collection(self, turns, initial_radius, step_radius, z_step=0.0):
        """
        以当前位置为中心执行螺旋运动，用于全方位数据集采集。
        :param turns: 旋转圈数
        :param initial_radius: 初始半径
        :param step_radius: 每圈增加的半径
        :param z_step: 螺旋过程中的深度变化量（每圈）
        """
        center_x = self.MotionController.pos.x
        center_y = self.MotionController.pos.y
        base_z = self.MotionController.pos.z
        start_rz = self.MotionController.pos.rz
        
        self.get_logger().info(f"开始螺旋采集: 圈数={turns}, 步进半径={step_radius}")
        
        points_per_turn = 12 # 每圈采样12个点（每30度一个点）
        for t in range(turns):
            for i in range(points_per_turn):
                angle_rad = (2 * PI / points_per_turn) * i
                current_radius = initial_radius + (t + i/points_per_turn) * step_radius
                
                target_x = center_x + current_radius * cos(angle_rad)
                target_y = center_y + current_radius * sin(angle_rad)
                target_z = base_z + (t + i/points_per_turn) * z_step
                
                # 设置目标位姿，面向圆心增加特征
                # 修正：物体在 (target_x, target_y)，中心在 (center_x, center_y)
                # 向量 (从机器人到圆心) 为 (center_x - target_x, center_y - target_y)
                # atan2(dy, dx) = atan2(-sin(angle_rad), -cos(angle_rad)) = angle_rad + PI
                # 在 NWU 坐标系下，-90度是正前方，因此 target_rz = (angle_rad + PI) * RAD2DEG - 90
                target_rz = AngleCorrect((angle_rad * RAD2DEG) + 180.0 - 90.0)
                
                # 使用 cs=0 (世界坐标) 移动
                self._send_setpoint(0x00, 0x0F, target_x, target_y, target_z, target_rz)
                
                self.move_wait()
                time.sleep(0.5) # 停留录制

    def run(self, task: dict):
        self.current_task_name = task.get("name", "unknown")
        self.log_state(self.current_task_name)  # 任务切换时强制记录一次

        if task["name"] == "movexyz":
            self.movexyz(task["params"]["x"], task["params"]
                         ["y"], task["params"]["z"])

        elif task["name"] == "throw_golf":
            self.throw_golf(task["params"]["dx"],
                            task["params"]["depth"])
            
        elif task["name"] == "movexy":
            self.movexy(task["params"]["x"], task["params"]["y"])

            
        elif task["name"] == "move_world_step":
            self.move_world_step(
                float(task["params"].get("x", 0.0)),
                float(task["params"].get("y", 0.0)),
                float(task["params"].get("z", 0.0))
            )

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

        elif task["name"] == "setx":
            self.setx(task["params"]["x"])

        elif task["name"] == "sety":
            self.sety(task["params"]["y"])

        elif task["name"] == "setrz":
            self.setrz(task["params"]["rz"])

        elif task["name"] == "setrz_step":
            mode = int(task["params"].get("mode", 0)) # 显式转为 int，避免底层消息转换失败
            self.setrz_step(float(task["params"]["rz"]), float(task["params"]["drz"]), mode)

        elif task["name"] == "search":
            self.search(task["params"]["name"], task["params"]["cam"])

        elif task["name"] == "mtty":
            self.mtty(task["params"]["x"], task["params"]["z"])
        elif task["name"] == "mttxf":
            self.mttxf(task["params"]["dy"],task["params"]["dx"])
        elif task["name"] == "mttz":
            self.mttz(task["params"]["x"], task["params"]["z"])
        elif task["name"] == "go_to_drump":
            self.go_to_drump()
        elif task["name"] == "crash_target":
            self.crash_target_avoiding_obstacles(task["params"]["target_id"])
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
            self.pass_door(task["params"]["depth"])

        elif task["name"] == "mttpos":
            self.mttpos(task["params"]["x"], task["params"]["y"], task["params"]["z"], task["params"]["rz"], task["params"]["dx"])
            
        elif task["name"] == "mttzpos_amend":
            self.mttzpos_amend(task["params"]["x"], task["params"]["y"], task["params"]["z"], task["params"]["dx"])
        
        elif task["name"] == "mttpos_amend":
            self.mttpos_amend(task["params"]["x"], task["params"]["y"], task["params"]["z"], task["params"]["rz"], task["params"]["dx"])
            
        elif task["name"] == "mttzpos":
            self.mttzpos(task["params"]["x"], task["params"]["y"], task["params"]["z"], task["params"]["dx"])
        
        elif task["name"] == "delay":
            self.delay(task["params"])

        elif task["name"] == "led":
            self.led(task["params"]["led0"],
                     task["params"]["led1"])

        elif task["name"] == "grab_golf":
            self.grab_golf(task["params"]["kind"],
                           task["params"]["dy"],
                           task["params"]["dx"],
                           task["params"]["down_depth"],
                           task["params"]["up_depth"])
        
        elif task["name"] == "put_t":
            self.put_t(task["params"]["num"],
                           task["params"]["dx"],
                           task["params"]["dz"])
            
        elif task["name"] == "move_with_swing":
            self.move_with_swing(task["params"]["x"],
                                 task["params"]["y"],
                                 task["params"]["z"],
                                 task["params"]["swing_angle"],
                                 task["params"]["swing_speed"])
        
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
        
        elif task["name"] == "make_datasets":
            # 无参数，直接调用make_datasets方法
            self.make_datasets()

        elif task["name"] == "pass_gate_avoiding_obstacles":
            self.pass_gate_avoiding_obstacles()

        elif task["name"] == "spiral_data":
            params = task.get("params", {})
            turns = params.get("turns", 2)
            initial_radius = params.get("r0", 1.0)
            step_radius = params.get("dr", 0.5)
            z_step = params.get("dz", 0.0)
            self.spiral_data_collection(turns, initial_radius, step_radius, z_step)

        else:
            self.get_logger().info("Info:非法任务名:  " + task["name"])
    
    def pid_updata(self, error):
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
    
    def cam2world(self, clas, timeout,cam):
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
                    self.robot.target_inbase.vector.x = self.down_cam.target_inworld.vector.x
                    self.robot.target_inbase.vector.y = self.down_cam.target_inworld.vector.y
                    self.robot.target_inbase.vector.z = self.down_cam.target_inworld.vector.z
                    self.robot.target_inbase.vector.rx = self.down_cam.target_inworld.vector.rx
                    self.robot.target_inbase.vector.ry = self.down_cam.target_inworld.vector.ry
                    self.robot.target_inbase.vector.rz = self.down_cam.target_inworld.vector.rz
                    self.robot.base2world()
                   
                    time.sleep(0.015)
                    
            if cam == "front":
                if self.yolov8_data_front.targets[clas].tpos_inworld.x != 0 or self.yolov8_data_front.targets[clas].tpos_inworld.y != 0 or self.yolov8_data_front.targets[clas].tpos_inworld.z != 0:
                    self.front_cam.target_inbase.vector.x = self.yolov8_data_front.targets[clas].tpos_inworld.x
                    self.front_cam.target_inbase.vector.y = self.yolov8_data_front.targets[clas].tpos_inworld.y
                    self.front_cam.target_inbase.vector.z = self.yolov8_data_front.targets[clas].tpos_inworld.z
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
                    time.sleep(0.015)
                    
            pos_list.append([self.robot.target_inworld.vector.x,
                                    self.robot.target_inworld.vector.y, self.robot.target_inworld.vector.z])

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
                    t_z =  self.robot.target_inbase.vector.z = self.down_cam.target_inworld.vector.z
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
                        #self.get_logger().info(f"t_x : {t_x:.2f} t_y: {t_y:.2f} t_z: {t_z:.2f}")
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
                        #self.get_logger().info(f"t_x : {t_x:.2f} t_y: {t_y:.2f} t_z: {t_z:.2f}")
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
        #self.get_logger().info(
            #f"目标在机器人坐标系下的位置: x : {x:.2f} y: {y:.2f} z: {z:.2f}")

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
                    self.movey(-a)
                    self.movex(b-0.175)
                    self.movez(depth)
                    self.movez(-depth)
                    break
                elif color == "yellow":
                    a, b, c = self.cam2robot(5, 5,"down")
                    self.movey(-a)
                    self.movex(b-0.175)
                    self.movez(depth)
                    self.movez(-depth)
                    break
            elif self.yolov8_data_down.state[6] == 1:
                led.led0 = 0
                led.led1 = 1
                r = sqrt((self.yolov8_data_down.targets[6].tpos_inpic.y -  480)**2 + (self.yolov8_data_down.targets[6].tpos_inpic.x -  640)**2)/sqrt(480**2+640**2)
                a, b, c = self.cam2robot_fast(6, 5,"down")
                if r >= pr:
                    self._publish_led(led.led0, led.led1)
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
                    self.fast_movey(-vx)
                    self.fast_movex(vy)
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
        column_index = img.shape[1] // 2
        self.get_logger().info(f"row: {row_index} , column: {column_index}")
        start_time = time.time()
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time >= timeout:
                self.get_logger().info("超时退出")
                break
            if all(x==0 for x in self.yolov8_data_down.state):
                self.get_logger().info(
                f"没有找到任何物体")
            elif self.yolov8_data_down.state[5] == 1:
                r = sqrt((self.yolov8_data_down.targets[5].tpos_inpic.y -  row_index)**2 + (self.yolov8_data_down.targets[5].tpos_inpic.x -  column_index)**2)/sqrt(480**2+640**2)
                led.led0 = 1
                led.led1 = 0
                a, b, c = self.cam2robot_fast(5, 5,"down")
                if r >= pr:
                    self._publish_led(led.led0, led.led1)
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
                    self.fast_movey(-vx)
                    self.fast_movex(vy)
                    time.sleep(step_time)
                else:
                    self.get_logger().info(f"收集框已进入视野中心")
                    break
            
        a, b, c = self.cam2robot(5, 5,"down")
        
        self.movey(-a)
        self.movex(b-0.175)   
        self.pow(0) 
        time.sleep(1)      
        

    def pass_door(self, depth):
        pr = 0.14
        k = 0.05
        step_time = 0.1  # 采样时间间隔，需与卡尔曼滤波器dt一致
        timeout = 50
        num = 2  
        r = 100
        # 初始化卡尔曼滤波器（参数可根据实际噪声调整）
        # process_noise：运动模型不确定性（非匀速时可增大，如0.5）
        # measurement_noise：观测噪声（摄像头抖动大时增大，如5.0）
        kf = KalmanFilter(
            dt=step_time,
            process_noise=0.6,
            measurement_noise=5.0
        )

        self.setz(depth)                                                                
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(self.front_cam_Image_data, "bgr8")
        row_index = img.shape[0] // 2
        column_index = img.shape[1] // 4
        self.get_logger().info(f"row: {row_index} , column: {column_index}")
        start_time = time.time()
        
        if self.yolov8_data_front.state[num] != 0:
            time.sleep(1)
            self.get_logger().info("发现目标")
            a, b, c = self.cam2robot(num, 5, "front")
            if c + self.MotionController.pos.z > 1.0:
                    c = 1.0 - self.MotionController.pos.z
            self.movey(-0.5*a)
            self.movez(c)
            time.sleep(3)
        
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time >= timeout:
                self.get_logger().info("超时退出")
                time.sleep(4)
                if self.yolov8_data_front.state[num] != 0 and r >0.2:
                    time.sleep(1)
                    self.get_logger().info("发现目标")
                    a, b, c = self.cam2robot(num, 5, "front")
                    b = b - 0.3
                    a = a - 0.1
                    rz = atan2(b, a)*RAD2DEG
                    self.get_logger().info(f"Info: ======转向目标点,旋转{rz:.2f}°======")
                    self.moverz(rz)
                    break
            
            if all(x == 0 for x in self.yolov8_data_front.state):
                self.get_logger().info(f"没有找到任何物体")
                time.sleep(2 * step_time)
            elif self.yolov8_data_front.state[num] == 1:
                # 获取原始观测坐标
                raw_x = self.yolov8_data_front.targets[num].tpos_inpic.x
                raw_y = self.yolov8_data_front.targets[num].tpos_inpic.y
                
                # 卡尔曼滤波：先预测，再用新观测更新
                kf.predict()
                kf.update(raw_x, raw_y)
                # 获取滤波后的坐标
                filtered_x, filtered_y = kf.get_filtered_position()
                
                # 用滤波后的坐标计算r
                r = sqrt(
                    (filtered_y - row_index) **2 + 
                    (filtered_x - column_index)** 2
                ) / sqrt(480**2 + 640**2)
                
                rx = sqrt((filtered_x - column_index)** 2) /640
                ry = sqrt((filtered_y - row_index) **2 ) / 480
                a, b, c = self.cam2robot_fast(num, 5, "front")
                
                if rx >= pr:
                    self.get_logger().info(f"发现door，滤波后rx值: {rx:.4f}")
                    vx = k * a
                    # 速度限制
                    if abs(vx) < 0.01:
                        vx = 0.01 if vx >= 0 else -0.01
                    if abs(vx) > 0.1:
                        vx = 0.1 if vx >= 0 else -0.1
                    self.fast_movey(-vx)
                    time.sleep(step_time * 100 * 2 * abs(vx) ) 
                if ry >= pr:
                    self.get_logger().info(f"发现door，滤波后ry值: {ry:.4f}")
                    vz = 0.5 * k * c
                    # 速度限制
                    if abs(vz) < 0.01:
                        vz = 0.01 if vz >= 0 else -0.01
                    if abs(vz) > 0.1:
                        vz = 0.1 if vz >= 0 else -0.1
                    self.fast_movez(vz)
                    time.sleep(step_time * 1000 * abs(vz))
                if ry < pr and rx < pr:
                    self.get_logger().info(f"收集框已进入视野中心")
                    self.stop()
                    break
        
        # 后续流程保持不变
        cnt = 0
        time.sleep(0.5)

        while True:
            cnt += 1
            if (self.yolov8_data_down.state[num] != 0 and
                self.yolov8_data_down.targets[num].tpos_inpic.y > 200) or cnt > 200:
                self.stop()
                self.movex(0.25)
                self.get_logger().info(f"过门任务完成")
                break
            self._send_setpoint(0x20, 0x0F, 0.015, 0.0, 0.0, 0.0)
            time.sleep(0.1)
                   

    def led(self, led0, led1):
        self._publish_led(led0, led1)
    
    def delay(self, t):
        time.sleep(t)
    
    def grab_golf(self, kind, dy, dx, down_depth, up_depth):   
        
        pr = 0.14
        pr = up_depth
        k = 0.05
        step_time = 0.1  # 采样时间间隔，需与卡尔曼滤波器dt一致
        timeout = 50
        r = 100
        num = 0   
        
        if kind == "yellow_golf":
            num = 4
        if kind == "pink_golf":
            num = 5

        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(self.down_cam_Image_data,"bgr8")
        row_index = img.shape[0] // 2
        column_index = img.shape[1] // 2
        self.get_logger().info(f"row: {row_index} , column: {column_index}")
        
        sample_flag = 0
        golf_cnt = 0

        self.pow(1)
        self.led(0, 0)
        s = 1
        while True:
            if self.yolov8_data_front.state[3]== 0  and \
               self.yolov8_data_down.state[3]== 0 :
                # pass
                self._send_setpoint(0x20, 0x0F, 0.0, 0.0, 0.0, 0.1)
                time.sleep(0.01)
                golf_cnt+=1

            else:
                if self.yolov8_data_front.state[3]!= 0 :
                    self.get_logger().info("前视发现置物台")
                    time.sleep(2)
                    a, b, c = self.cam2robot(3, 5,"front")
                    a = a + 0.1
                    b = b - 0.1
                    self.movey(-a)
                    self.movex(b)
                    time.sleep(2)
                    break
                elif self.yolov8_data_down.state[3]!= 0 :
                    self.get_logger().info("下视发现置物台")
                    time.sleep(2)
                    a, b, c = self.cam2robot(3, 5,"down")
                    a = a + 0.1
                    b = b - 0.1
                    self.setz(0.1)
                    self.movey(-a)
                    self.movex(b)
                    time.sleep(2)
                    break
                else:
                    self._send_setpoint(0x20, 0x0F, 0.0, 0.0, 0.0, 0.1)
                    time.sleep(0.01)
                    golf_cnt+=1
            if  golf_cnt>4000 :
                self.get_logger().info("任务失败")
             
                return
                     
        '''s = self.search(kind, "down")
        if s == 0:
            self.get_logger().info("======下无目标======")
            self.get_logger().info("======第一次平移寻找目标目标======")
            self.movey(-0.2)
            self.movex(0.2)
            s = self.search( kind, "down")
            if s == 0:
                self.get_logger().info("======下无目标======")
                self.get_logger().info("======第二次平移寻找目标目标======")
                self.movex(-0.4)
                s = self.search( kind, "down")
                if s == 0:
                    self.get_logger().info("======下无目标======")
                    self.get_logger().info("======第三次平移寻找目标目标======")
                    self.movey(0.4)
                    s = self.search( kind, "down")
                    if s == 0:
                        self.get_logger().info("======下无目标======")
                        self.get_logger().info("======第四次平移寻找目标目标======")
                        self.movex(0.4)
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
            if self.yolov8_data_down.state[num] == 0:
                golf_cnt+=1
                time.sleep(0.01)
                if golf_cnt>2000:
                    self.get_logger().info("======确认无目标======")
                    self.get_logger().info("=====任务失败======")
                    return                  
            elif self.yolov8_data_down.state[num] !=0:
                self.get_logger().info("======发现目标球=====")
                
                rx = sqrt((self.yolov8_data_down.targets[num].tpos_inpic.x - column_index)** 2) /640
                ry = sqrt((self.yolov8_data_down.targets[num].tpos_inpic.y - row_index) **2 ) / 480

                rx = rx - 0.9
                

                a, b, c = self.cam2robot_fast(num, 5, "down")
                if ry < pr and rx < pr:
                    self.get_logger().info(f"收集框已进入视野中心")
                    self.stop()
                    self.movey(-dy)
                    self.movex(dx)  
                    time.sleep(1.0)
                    self.movez(down_depth)
                    break
                if rx >= pr:
                    self.get_logger().info(f"发现ball，滤波后rx值: {rx:.4f}")
                    vx = k * a
                    # 速度限制
                    if abs(vx) < 0.01:
                        vx = 0.01 if vx >= 0 else -0.01
                    if abs(vx) > 0.1:
                        vx = 0.1 if vx >= 0 else -0.1
                    self.fast_movey(-vx)
                    time.sleep(step_time * 200 * 2 * abs(vx) ) 
                if ry >= pr:
                    self.get_logger().info(f"发现ball，滤波后ry值: {ry:.4f}")
                    vy = 0.5 * k * b
                    # 速度限制
                    if abs(vy) < 0.01:
                        vy = 0.01 if vy >= 0 else -0.01
                    if abs(vy) > 0.1:
                        vy = 0.1 if vy >= 0 else -0.1
                    self.fast_movex(vy)
                    time.sleep(step_time * 200 * abs(vy))
                
              
        
        self.get_logger().info("=====抓球结束，开始上浮======")
        self.setz(0.3)
        time.sleep(8.0)
        
        self.get_logger().info("=====上浮结束======")
        return
           
    
    
    def put_t(self, num,dx,dz):
        T_cnt = 0
        T_cnt2 = 0
        T_cnt3 = 0
        while True:
            if self.yolov8_data_front.state[num] == 0:
                self._send_setpoint(0x20, 0x0F, 0.0, 0.0, 0.0, 0.01)
                time.sleep(0.001)
                T_cnt+=1


            elif  self.yolov8_data_front.state[num] != 0:
                time.sleep(2)
                T_cnt = 0
                self.get_logger().info("发现目标")
                a, b, c = self.cam2robot(num, 5,"front")
                a=a-dx
                c=c-dz
                self.movey(-a)
                self.movez(c)
                time.sleep(1)
                while True:
                    T_cnt2+=1
                    if self.yolov8_data_front.state[num] != 0 and T_cnt2 % 600 == 0:
                        time.sleep(2)
                        a, b, c = self.cam2robot(num, 5,"front")
                        a=a-dx
                        c=c-dz
                        if (abs(a) > 0.03 or abs(c) > 0.03) and b > 0.25:
                            self.get_logger().info("======目标偏移======")
                            self._send_setpoint(0x20, 0x0F, float(b), float(a), float(c), 0.0)
                            time.sleep(2)
                        else:
                            self._send_setpoint(0x20, 0x0F, 0.001, 0.0, 0.0, 0.0)
                            self.get_logger().info("======正在前进======")
                            T_cnt+=1
                            time.sleep(0.01)

                    else:
                        self._send_setpoint(0x20, 0x0F, 0.001, 0.0, 0.0, 0.0)
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
        plate_cnt = 0
        while True:
            if self.yolov8_data_front.state[num] ==0 and \
               self.yolov8_data_down.state[num] ==0 :
                # pass
                self._send_setpoint(0x20, 0x0F, 0.0, 0.0, 0.0, 0.2)
                time.sleep(0.01)
                golf_cnt+=1
                
            else:
                if self.yolov8_data_front.state[num] ==1:
                    self.get_logger().info("前视发现目标")
                    time.sleep(2)
                    a, b, c = self.cam2robot(num, 5,"front")
                    a = a - 0.3
                    b = b - 0.5                    
                    self.movey(-a)
                    self.movex(b)
                    time.sleep(1)
                    self.setz(-0.1)
                    self.get_logger().info("任务成功")
                    break
                else:
                    self.get_logger().info("下视发现目标")
                    a, b, c = self.cam2robot(num, 5,"down")
                    a = a - 0.3
                    b = b - 0.5                    
                    self.movey(-a)
                    self.movex(b)
                    time.sleep(1)
                    self.setz(-0.1)
                    self.get_logger().info("任务成功")
                    break
            if  plate_cnt>2000 :
                self.get_logger().info("任务失败")
                return

    def make_datasets(self):
        # 创建保存图像的目录
        save_dir = os.path.join(os.path.expanduser("~"), "uuv_datasets")
        os.makedirs(save_dir, exist_ok=True)
        
        # 初始化 CvBridge
        bridge = CvBridge()
        
        # 初始化图像时间戳记录
        last_image_time = {
            "front_left": 0,
            "front_right": 0,
            "down_left": 0,
            "down_right": 0
        }
        
        # 初始化保存计数
        save_count = 0
        
        # 循环保存图像
        while rclpy.ok():
            # 定义摄像头名称和对应的图像数据
            cameras = [
                ("front_left", self.front_cam_left_Image_data),
                ("front_right", self.front_cam_right_Image_data),
                ("down_left", self.down_cam_left_Image_data),
                ("down_right", self.down_cam_right_Image_data)
            ]
            
            # 轮流检查并保存每个摄像头的图像
            for cam_name, img_data in cameras:
                if img_data is not None:
                    try:
                        # 获取图像消息的时间戳
                        current_time = img_data.header.stamp.sec + img_data.header.stamp.nanosec / 1e9
                        
                        # 只有当图像时间戳更新时才保存
                        if current_time > last_image_time[cam_name]:
                            # 转换图像消息为OpenCV格式
                            cv_img = bridge.imgmsg_to_cv2(img_data, "bgr8")
                            
                            # 生成时间戳
                            timestamp = time.strftime("%Y%m%d_%H%M%S_%f")
                            
                            # 生成文件名
                            filename = f"{cam_name}_{timestamp}.jpg"
                            filepath = os.path.join(save_dir, filename)
                            
                            # 保存图像
                            cv2.imwrite(filepath, cv_img)
                            save_count += 1
                            self.get_logger().info(f"Saved image: {filepath}, Total saved: {save_count}")
                            
                            # 更新时间戳记录
                            last_image_time[cam_name] = current_time
                            time.sleep(0.2)
                    except Exception as e:
                        self.get_logger().error(f"Failed to save image for {cam_name}: {str(e)}")
                else:
                    self.get_logger().warn(f"No image data for {cam_name}")
            
            # 等待一段时间再继续检查
            time.sleep(0.1)

    def throw_golf(self, dx, depth):
        self.get_logger().info("======开始投球======")
        self.led(0, 1)
        self.search('col_basket', 'down')
        self.led(1, 1)
        self.mttxf(0, dx)
        self.movez(depth)
        self.led(1, 0)
        self.pow(0)
        self.delay(1)
        self.led(0, 0)
        self.get_logger().info("======投球结束======")
        
    def strike_ball(self,num):
        
        ball_cnt = 0
        while True:
            if self.yolov8_data_front.state[num] == 0:
                self._send_setpoint(0x20, 0x0F, 0.0, 0.0, 0.0, 0.05)
                time.sleep(0.01)
                ball_cnt+=1
            elif  self.yolov8_data_front.state[num] != 0:
                time.sleep(2)
                ball_cnt = 0
                self.get_logger().info("发现目标")
                a, b, c = self.cam2robot(num, 5,"front")
                if c + self.MotionController.pos.z > 1.0:
                    c = 1.0 - self.MotionController.pos.z
                b = b - 0.2
                rz = atan2(b, a)*RAD2DEG
                self.get_logger().info(f"Info: ======转向目标点,旋转{rz:.2f}°======")
                self.moverz(rz)
                self.movez(c)
                time.sleep(1)
                a, b, c = self.cam2robot(num, 5,"front")
                c = c - 0.15
                b = b - 0.3
                a = a - 0.1
                time.sleep(1)
                self.movez(c)
                self.movexy(a,b)
                self.get_logger().info("======任务完成 正在后退======")
                self.fast_movex(-0.5)
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
            drz = self.pid_updata(angle_error)

            if all(x==0 for x in self.yolov8_data_down.state):
                # pass
                self._send_setpoint(0x20, 0x0F, 0.002, 0.0, 0.0, drz)
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
                        self._send_setpoint(0x20, 0x0F, 0.002, 0.0, 0.0, drz)
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
                        self._send_setpoint(0x20, 0x0F, 0.002, 0.0, 0.0, drz)
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
                        self._send_setpoint(0x20, 0x0F, 0.002, 0.0, 0.0, drz)
                        cnt_1 += 1
                        time.sleep(timesleep)

                else:
                    #pass
                    self._send_setpoint(0x20, 0x0F, 0.002, 0.0, 0.0, drz)
                    cnt_1 += 1
                    time.sleep(timesleep)
            if cnt_1 > 1200:
                self.get_logger().info("Info:任务全部执行完毕")
                return
            if all(x == 1 for x in self.task_lock):
                self._send_setpoint(0x20, 0x0F, 0.004, 0.0, 0.0, drz)
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
                        kf.updata(line_center)
                    
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
                drz = self.pid_updata(angle_error) * 0.5  # 降低控制强度
                dy = 0.004  # 降低前进速度
            else:
                drz = self.pid_updata(angle_error)
                dy = 0.005 - 0.0004 * abs(drz)
            
            # 控制逻辑
            if all(x == 0 for x in self.yolov8_data_down.state):
                self._send_setpoint(0x20, 0x0F, dy, 0.0, 0.0, drz)
                time.sleep(timesleep)

            elif not all(x == 0 for x in self.yolov8_data_down.state):
                if self.yolov8_data_down.state[1] != 0 and self.yolov8_data_down.targets[1].tpos_inpic.y > row_index1:
                        if self.task_lock[0] == 0:
                            self.get_logger().info("检测到黑色方块，开始执行任务")
                            time.sleep(2)
                            magnet.state = 0
                            self._publish_magnet(magnet.state)
                            self.led(1, 1)
                            time.sleep(1)
                            self.get_logger().info("任务执行完毕")
                            self.task_lock[0] = 1
                        else:
                            self._send_setpoint(0x20, 0x0F, dy, 0.0, 0.0, drz)
                            time.sleep(timesleep)
                elif self.yolov8_data_down.state[2] != 0 and self.yolov8_data_down.targets[2].tpos_inpic.y > row_index1:
                    if self.task_lock[1] == 0:
                        self.get_logger().info("检测到绿色圆形，开始执行任务")
                        time.sleep(2)
                        a, b, c = self.cam2robot(2, 4, "down")
                        b = b - 0.4
                        self.movey(-a)
                        self.movex(b)
                        self.movez(depth)
                        self.led(1,0)
                        self.movez(-depth)
                        self.led(0,0)
                        self.get_logger().info("任务执行完毕")
                        self.task_lock[1] = 1
                    else:
                        self._send_setpoint(0x20, 0x0F, dy, 0.0, 0.0, drz)
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
                        self._send_setpoint(0x20, 0x0F, dy, 0.0, 0.0, drz)
                        time.sleep(timesleep)
                else:
                    self._send_setpoint(0x20, 0x0F, dy, 0.0, 0.0, drz)
                    time.sleep(timesleep)
                    
            if all(x == 1 for x in self.task_lock):
                self._send_setpoint(0x20, 0x0F, dy, 0.0, 0.0, drz)
                time.sleep(timesleep)
                cnt += 1
                if cnt > 190:
                    return

    def pass_gate_avoiding_obstacles(self):
        gate_green_id = 2
        gate_red_id = 3
        obstacle_ids = [1, 5, 6, 7]
        safe_radius = 1.5

        self.get_logger().info("====== 开始基于视觉子目标的避障穿门任务 ======")


        # 1. 第一步：获取大门大致位置
        gx1_w = gy1_w = gx2_w = gy2_w = None
        while True:
            if self.yolov8_data_front_pos.state[gate_green_id] > 0 and self.yolov8_data_front_pos.state[gate_red_id] > 0:
                gx1_w = self.yolov8_data_front_pos.targets[gate_green_id].tpos_inworld.x
                gy1_w = self.yolov8_data_front_pos.targets[gate_green_id].tpos_inworld.y
                gx2_w = self.yolov8_data_front_pos.targets[gate_red_id].tpos_inworld.x
                gy2_w = self.yolov8_data_front_pos.targets[gate_red_id].tpos_inworld.y
                break
            self.get_logger().info("等待门柱定位...")
            time.sleep(0.5)

        gate_center_x = (gx1_w + gx2_w) / 2.0
        gate_center_y = (gy1_w + gy2_w) / 2.0

        self.get_logger().info(f"大门初始中心坐标: ({gate_center_x:.2f}, {gate_center_y:.2f})")

        # 用于计算点到线段的距离
        def point_line_distance(px, py, x1, y1, x2, y2):
            l2 = (x2 - x1)**2 + (y2 - y1)**2
            if l2 == 0:
                return math.hypot(px - x1, py - y1)
            t = max(0, min(1, ((px - x1)*(x2 - x1) + (py - y1)*(y2 - y1)) / l2))
            proj_x = x1 + t * (x2 - x1)
            proj_y = y1 + t * (y2 - y1)
            return math.hypot(px - proj_x, py - proj_y)
        
        # 定义阶段标量：0代表前往门前1.5m，1代表穿门
        phase = 0

        # 2. 第二步：循环前往目标
        while True:
            cur_x = self.MotionController.pos.x
            cur_y = self.MotionController.pos.y

            # 实时重估大门中心位置以应对前端推测不准导致的位置偏移
            if self.yolov8_data_front_pos.state[gate_green_id] > 0 and self.yolov8_data_front_pos.state[gate_red_id] > 0:
                gx1_w = self.yolov8_data_front_pos.targets[gate_green_id].tpos_inworld.x
                gy1_w = self.yolov8_data_front_pos.targets[gate_green_id].tpos_inworld.y
                gx2_w = self.yolov8_data_front_pos.targets[gate_red_id].tpos_inworld.x
                gy2_w = self.yolov8_data_front_pos.targets[gate_red_id].tpos_inworld.y
                gate_center_x = (gx1_w + gx2_w) / 2.0
                gate_center_y = (gy1_w + gy2_w) / 2.0

            # 寻找门平面的法向量
            # 向量 V(门柱连线)
            vx = gx2_w - gx1_w
            vy = gy2_w - gy1_w
            # 法向量 N1, N2
            nx, ny = vy, -vx
            n_len = math.hypot(nx, ny)
            if n_len > 0:
                nx /= n_len
                ny /= n_len
            
            # 选择朝向 AUV 当前方向的法向量作为“正前方”
            if (cur_x - gate_center_x) * nx + (cur_y - gate_center_y) * ny > 0:
                nx, ny = -nx, -ny

            # 定义两个目标点
            goal_front_x = gate_center_x - nx * 1 # 门正前方 1米处
            goal_front_y = gate_center_y - ny * 1
            
            if phase == 0:
                goal_x = goal_front_x
                goal_y = goal_front_y
                dist_to_final = math.hypot(goal_x - cur_x, goal_y - cur_y)
                if dist_to_final < 0.3:
                    # 计算门平面的法线朝向角 (正前方方向) + 60度
                    # gate_normal_angle = 90.0 - math.atan2(ny, nx) * RAD2DEG
                    # target_rz = gate_normal_angle + 60.0
                    # while target_rz > 180.0: target_rz -= 360.0
                    # while target_rz <= -180.0: target_rz += 360.0
                    # self.mttpos(goal_x, goal_y, 1, 0, 0.2)
                    
                    # self.get_logger().info(f"已到达大门正前方，调整朝向至 {target_rz:.1f}°，进入视觉锁定穿门阶段！")
                    # self.setrz(target_rz)
                    time.sleep(2.0) # 等待转向稳定
                    phase = 1
            
            if phase == 1:
                # self.get_logger().info("Phase 1: 360度环视扫描双门柱模式...")
                self.setz(1.0)
                time.sleep(0.5)

                green_dir = None
                red_dir = None

                # for step in range(12):
                #     # 等待视觉刷新
                #     time.sleep(0.5)
                    
                #     if green_dir is None and self.yolov8_data_front.state[gate_green_id] > 0:
                #         # 1. 使用相机内像素位置
                #         px = self.yolov8_data_front.targets[gate_green_id].tpos_inpic.x
                #         # 2. 算成视场中的相对角度 (假设图像中心 320，HFOV 90度。左正右负)
                #         rel_angle = (px - 640.0) * (34.19 / 1280.0)
                #         # 3. 从机器人坐标系角度换算成世界坐标绝对角度
                #         green_dir = self.MotionController.pos.rz + rel_angle
                #         while green_dir > 180: green_dir -= 360
                #         while green_dir < -180: green_dir += 360
                #         self.get_logger().info(f"扫到绿柱(左侧): 像素X {px}, 相对角 {rel_angle:.1f}°, 绝对角 {green_dir:.1f}°")

                #     if red_dir is None and self.yolov8_data_front.state[gate_red_id] > 0:
                #         px = self.yolov8_data_front.targets[gate_red_id].tpos_inpic.x
                #         rel_angle = (px - 640.0) * (34.19 / 1280.0)
                #         red_dir = self.MotionController.pos.rz + rel_angle
                #         while red_dir > 180: red_dir -= 360
                #         while red_dir < -180: red_dir += 360
                #         self.get_logger().info(f"扫到红柱(右侧): 像素X {px}, 相对角 {rel_angle:.1f}°, 绝对角 {red_dir:.1f}°")

                #     if green_dir is not None and red_dir is not None:
                #         self.get_logger().info("双柱均已成功记录！停止环视。")
                #         break
                        
                #     self.get_logger().info("未集齐双柱，原地转动 30度...")
                #     self.moverz(30.0)
                #     time.sleep(1.0)

                # if green_dir is not None and red_dir is not None:
                #     # 使用向量平均避免正负180度跃变问题
                #     vx = math.cos(green_dir * math.pi / 180.0) + math.cos(red_dir * math.pi / 180.0)
                #     vy = math.sin(green_dir * math.pi / 180.0) + math.sin(red_dir * math.pi / 180.0)
                #     tgt_abs_angle = 90 - math.atan2(vy, vx) * RAD2DEG
                    
                #     self.get_logger().info(f"双柱方向均值绝对角：{tgt_abs_angle:.1f}°，执行绝对转向 setrz，随后冲刺 3.0m！")
                    
                #     # 4. 直接执行绝对转向 
                #     self.setrz(tgt_abs_angle)
                #     # 给定足够长的时间转到目标角度
                #     time.sleep(3.0) 
                #     self.movey(2.0)
                    
                #     self.get_logger().info("====== 避障穿门任务完成 ======")
                #     break
                # else:
                # 按照刚才算的方向(负法向量，从起方看向门)和门中点，直接闭环过去
                # 修正算理：nx, ny 是门指向AUV的法向，所以潜器过门应当朝向 (-nx, -ny)
                blind_angle = math.atan2(ny, nx) * RAD2DEG + 180
                self.get_logger().info(f"blind_angle={blind_angle:.1f}°")
                # self.get_logger().info(f"一圈转完也没找齐双门: 直接闭环到大门中点 ({gate_center_x:.2f}, {gate_center_y:.2f})！沿绝对角: {blind_angle:.1f}°")
                self.get_logger().info(f"直接闭环到大门中点 ({gate_center_x:.2f}, {gate_center_y:.2f})！沿绝对角: {blind_angle:.1f}°")
                self.setx(gate_center_x)
                self.setrz(0)
                self.sety(gate_center_y)
                #self.mttpos(gate_center_x , gate_center_y + 5, 1.0, blind_angle, 0.2)
                # self.movey(2.0)
                self.get_logger().info("====== 避障穿门任务完成 ======")
                break

            # ============= 只有 phase == 0 (远离阶段) 才会走下面的 A* 避障移动 =============
            
            # 获取所有当前能看见的障碍物位置
            obs = []
            for oid in obstacle_ids:
                if self.yolov8_data_front_pos.state[oid] > 0:
                    ox = self.yolov8_data_front_pos.targets[oid].tpos_inworld.x
                    oy = self.yolov8_data_front_pos.targets[oid].tpos_inworld.y
                    obs.append((ox, oy))

            # 根据 AStar 算法搜索全局路径避障
            planner = AStarPlanner(resolution=0.5, safe_radius=safe_radius)
            target_x, target_y = planner.plan(cur_x, cur_y, goal_x, goal_y, obs)

            if target_x != goal_x or target_y != goal_y:
                self.get_logger().info(f"A* 规划子目标: 向 ({target_x:.2f}, {target_y:.2f}) 移动避开U形障碍")

            # 根据 AStar 算法搜索全局路径避障
            planner = AStarPlanner(resolution=0.5, safe_radius=safe_radius)
            target_x, target_y = planner.plan(cur_x, cur_y, goal_x, goal_y, obs)
            
            if target_x != goal_x or target_y != goal_y:
                self.get_logger().info(f"A* 规划子目标: 向 ({target_x:.2f}, {target_y:.2f}) 移动避开U形障碍")

            # 3. 将世界坐标子目标转化为单步的相对移动和转向，一次最多走3米
            dx = target_x - cur_x
            dy = target_y - cur_y
            dist_target = math.hypot(dx, dy)
            step_dist = min(3.0, dist_target)

            self.robot.target_inworld.vector.x = cur_x + (dx / dist_target) * step_dist
            self.robot.target_inworld.vector.y = cur_y + (dy / dist_target) * step_dist
            self.robot.world2base()
            
            rel_x = self.robot.target_inbase.vector.x
            rel_y = self.robot.target_inbase.vector.y

            rel_rz = math.atan2(rel_y, rel_x)*RAD2DEG
            r_dist = math.hypot(rel_x, rel_y)

            self.moverz(rel_rz)
            self.movex(r_dist)

            # 让新的观测跟上
            time.sleep(0.2)



    def go_to_drump(self):
        self.get_logger().info("====== 开始接近蓝鼓任务 ======")
        blue_drump_id = 0
        
        # 1. 升到 z=0.8
        self.get_logger().info("调整深度为 z=0.8")
        self.setz(0.8)
        time.sleep(1.0)
        
        # 2. 直线走到蓝鼓附近
        self.get_logger().info("尝试定位蓝鼓...")
        while True:
            # 持续利用 front_pos 的位姿消息更新蓝鼓的坐标
            if self.yolov8_data_front_pos.state[blue_drump_id] > 0:
                self.robot.target_inworld.vector.x = self.yolov8_data_front_pos.targets[blue_drump_id].tpos_inworld.x
                self.robot.target_inworld.vector.y = self.yolov8_data_front_pos.targets[blue_drump_id].tpos_inworld.y
                self.robot.world2base()
                
                rx = self.robot.target_inbase.vector.x
                ry = self.robot.target_inbase.vector.y
                dist = math.hypot(rx, ry)
                
                # 若到达距离小于 1.5 米，则认为成功到达
                if dist < 1.5:
                    self.get_logger().info("已成功直线到达蓝鼓附近，任务完成！")
                    break

                angle = math.atan2(ry, rx)*RAD2DEG

                # 每次取距离减去缓冲，最大步进控制在 2.0米内
                step = min(dist - 1.0, 2.0)
                if step > 0:
                    self.get_logger().info(f"走向蓝鼓: 剩 {dist:.2f}m, 此次步进 {step:.2f}m")
                    self.moverz(angle)
                    self.movex(step)
                    time.sleep(0.5)
            else:
                self.get_logger().info("未找到蓝鼓定位消息，等待视野刷新或坐标更新...")
                time.sleep(0.5)

        self.get_logger().info("====== goto_drump 任务结束 ======")

    def crash_target_avoiding_obstacles(self, target_id):
        obstacle_ids = [0, 1, 2, 3, 4, 5] # 全部障碍物集合
        if target_id in obstacle_ids:
            obstacle_ids.remove(target_id)
            
        safe_radius = 2.0
        self.get_logger().info(f"====== 开始基于视觉子目标的撞击任务 (目标ID: {target_id}) ======")

        # 首先等待目标定位
        tx_w = ty_w = None
        while True:
            if self.yolov8_data_front_pos.state[target_id] > 0:
                tx_w = self.yolov8_data_front_pos.targets[target_id].tpos_inworld.x
                ty_w = self.yolov8_data_front_pos.targets[target_id].tpos_inworld.y
                break
            self.get_logger().info("等待目标定位...")
            time.sleep(0.5)

        self.get_logger().info(f"目标初始坐标: ({tx_w:.2f}, {ty_w:.2f})")

        phase = 0
        while True:
            cur_x = self.MotionController.pos.x
            cur_y = self.MotionController.pos.y
            cur_z = self.MotionController.pos.z

            # 实时重估目标位置
            if self.yolov8_data_front_pos.state[target_id] > 0:
                tx_w = self.yolov8_data_front_pos.targets[target_id].tpos_inworld.x
                ty_w = self.yolov8_data_front_pos.targets[target_id].tpos_inworld.y

            goal_x = tx_w
            goal_y = ty_w

            if phase == 0:
                dist_to_final = math.hypot(goal_x - cur_x, goal_y - cur_y)
                # 距离目标 2 米时进入冲刺阶段
                if dist_to_final < 2.0:
                    self.get_logger().info("已到达目标附近，进入视觉锁定冲刺撞击阶段！")
                    phase = 1
            
            if phase == 1:
                # # 视觉闭环控制：通过PID旋转使目标保持在视野x轴中央
                # self.get_logger().info("Phase 1: 进入视觉闭环对准模式（目标居中x轴）...")
                
                # # 第一步：环绕索敌，找到目标
                # target_found = False
                # for step in range(12):
                #     time.sleep(0.5)
                #     if self.yolov8_data_front.state[target_id] > 0:
                #         target_found = True
                #         break
                #     self.moverz(30.0)
                #     time.sleep(1.0)
                
                # if not target_found:
                #     self.get_logger().info("丢视野，向后盲摸索一点再看...")
                #     self.movey(-0.5)
                #     continue
                
                # # 第二步：PID闭环对准——旋转使目标像素对准图像x轴中心(640)
                # self.previous_error = 0.0
                # self.i_error = 0.0
                # self.dt = 1
                # self.pid_parameters.p = 0.05
                # self.pid_parameters.i = 0.001
                # self.pid_parameters.d = 0.3
                # self.pid_parameters.output_limit = 15.0
                
                # center_threshold = 30       # 像素容差，|error|<30 认为对准
                # centered_count = 0
                # required_centered = 5       # 连续对准次数阈值
                # max_iterations = 100        # 最大迭代次数（防死循环）
                
                # for i in range(max_iterations):
                #     # 检查目标是否仍在视野中
                #     if self.yolov8_data_front.state[target_id] <= 0:
                #         self.get_logger().warn(f"闭环对准第{i+1}次: 丢失目标")
                #         continue
                    
                #     # 获取目标在图像中的像素x坐标
                #     px = self.yolov8_data_front.targets[target_id].tpos_inpic.x
                #     error = px - 640.0  # 正值=目标在右侧，需右转
                    
                #     self.get_logger().info(
                #         f"闭环对准 [{i+1}/{max_iterations}]: px={px:.1f}, error={error:.1f}px")
                    
                #     if abs(error) < center_threshold:
                #         centered_count += 1
                #         if centered_count >= required_centered:
                #             self.get_logger().info(
                #                 f"目标已对准视野中央！(连续{required_centered}次误差<{center_threshold}px)")
                #             break
                #     else:
                #         centered_count = 0
                    
                #     # PID计算旋转修正量（单位：度）
                #     drz = self.pid_updata(error)
                #     self.fast_moverz(drz)
                #     time.sleep(0.5)
                
                # self.get_logger().info("====== 视觉闭环对准任务完成 ======")
                # self.fast_movey(3.0)  # 冲刺前进3米

                goal_x = self.yolov8_data_front_pos.targets[target_id].tpos_inworld.x
                goal_y = self.yolov8_data_front_pos.targets[target_id].tpos_inworld.y
                
                #保险起见，转两圈，确认装上目标物体

                self.setz(cur_z)

                self.setx(goal_x)
                time.sleep(0.2)
                self.sety(goal_y)
                time.sleep(1)
                self.setx(goal_x+0.4)
                time.sleep(0.4)
                self.sety(goal_y+0.4)
                time.sleep(0.4)
                self.setx(goal_x)
                time.sleep(0.4)
                self.setx(goal_x-0.4)
                time.sleep(0.4) 
                self.sety(goal_y)
                time.sleep(0.4)
                self.sety(goal_y-0.4)
                time.sleep(0.4)
                self.setx(goal_x)
                time.sleep(0.4)
                self.setx(goal_x+0.4)
                time.sleep(0.4)


                self.setx(goal_x+0.6)
                time.sleep(0.6)
                self.sety(goal_y+0.6)
                time.sleep(0.6)
                self.setx(goal_x)
                time.sleep(0.6)
                self.setx(goal_x-0.6)
                time.sleep(0.6)
                self.sety(goal_y)
                time.sleep(0.6) 
                self.sety(goal_y-0.6)
                time.sleep(0.6)
                self.setx(goal_x)
                time.sleep(0.6)
                self.setx(goal_x+0.6)
                self.setz(cur_z)
                break

            # ============= 只有 phase == 0 (远离阶段) 才会走下面的 A* 避障移动 =============
            
            obs = []
            for oid in obstacle_ids:
                if self.yolov8_data_front_pos.state[oid] > 0:
                    ox = self.yolov8_data_front_pos.targets[oid].tpos_inworld.x
                    oy = self.yolov8_data_front_pos.targets[oid].tpos_inworld.y
                    obs.append((ox, oy))

            planner = AStarPlanner(resolution=0.5, safe_radius=safe_radius)
            target_x, target_y = planner.plan(cur_x, cur_y, goal_x, goal_y, obs)

            if target_x != goal_x or target_y != goal_y:
                self.get_logger().info(f"A* 规划子目标: 向 ({target_x:.2f}, {target_y:.2f}) 移动避开障碍物")

            # 将世界坐标子目标转化为单步的相对移动和转向，一次最多走1米
            dx = target_x - cur_x
            dy = target_y - cur_y
            dist_target = math.hypot(dx, dy)
            step_dist = min(1.0, dist_target)

            self.robot.target_inworld.vector.x = cur_x + (dx / dist_target) * step_dist
            self.robot.target_inworld.vector.y = cur_y + (dy / dist_target) * step_dist
            self.robot.world2base()
            
            rel_x = self.robot.target_inbase.vector.x
            rel_y = self.robot.target_inbase.vector.y

            rel_rz = math.atan2(rel_y, rel_x)*RAD2DEG
            r_dist = math.hypot(rel_x, rel_y)

            self.moverz(rel_rz)
            self.movex(r_dist)

            time.sleep(0.5)


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
            dx = float(args[0])
            depth = float(args[1])
            dic = {
                "name": "throw_golf",
                "params": {
                    "dx": dx,
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
            dx = float(args[0])
            z = float(args[1])
            dic = {
                "name": "mtty",
                "params": {
                    "x": dx,
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
            dy = float(args[0])
            dx = float(args[1])
            dic = {
                "name": "mttxf",
                "params": {
                    "dy": dy,
                    "dx": dx
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'mttz':
        args = parts[1:]
        if len(args) == 2:
            dx = float(args[0])
            z = float(args[1])
            dic = {
                "name": "mttz",
                "params": {
                    "x": dx,
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
            dx = float(args[4])
            dic = {
                "name": "mttpos",
                "params": {
                    "x": x,
                    "y": y,
                    "z": z,
                    "rz": rz,
                    "dx": dx
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
            dx = float(args[3])
            dic = {
                "name": "mttzpos",
                "params": {
                    "x": x,
                    "y": y,
                    "z": z,
                    "dx": dx
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
            dx = float(args[4])
            dic = {
                "name": "mttpos_amend",
                "params": {
                    "x": x,
                    "y": y,
                    "z": z,
                    "rz": rz,
                    "dx": dx
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
            dx = float(args[3])
            dic = {
                "name": "mttzpos_amend",
                "params": {
                    "x": x,
                    "y": y,
                    "z": z,
                    "dx": dx
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
            depth     =   float(args[0])
            dic = {
                "name": "pass_door",
                "params": {
                    "depth": depth
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
            dy = float(args[1])
            dx = float(args[2])
            down_depth = float(args[3])
            up_depth = float(args[4])
            dic = {
                "name": "grab_golf",
                "params": {
                    "kind": kind,
                    "dy": dy,
                    "dx": dx,
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
            dx = float(args[1])
            dz = float(args[2])
            dic = {
                "name": "put_t",
                "params": {
                    "num": num,
                    "dx": dx,
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
    list = load_actions(opt.data_path[0]+"uv_tasks_SAUVC2026.json")

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

    workspace_root = Path(__file__).resolve().parents[2]

    # 默认 data path：优先使用已安装包的 share 目录，其次回退到源码仓库中的 datas
    default_data = str(workspace_root / 'src' / 'datas') + os.sep
    if get_package_share_directory is not None:
        try:
            pkg_share = get_package_share_directory('uv_ai')
            candidate = Path(pkg_share) / 'datas' 
            if candidate.exists():
                default_data = str(candidate) + os.sep
        except Exception:
            pass

    # 加载参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--front-topic', nargs='+', type=str, default=[
                        'front_cam/rectified'], help='前视摄像头')
    parser.add_argument('--down-topic', nargs='+', type=str, default=[
                        'down_cam/rectified'], help='下视摄像头')
    parser.add_argument('--data-path', nargs='+', type=str, default=[
                        default_data], help='PID参数路径')
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

    try:
        rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+Z）
    finally:
        try:
            node.log_state()  # 退出前写最终一条日志
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()  # 关闭rclpy
    