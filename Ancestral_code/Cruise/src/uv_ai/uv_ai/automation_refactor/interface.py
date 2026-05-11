import rclpy
from rclpy.node import Node

from uv_control_py.CoordinateSystem import CoordinateSystems

from uv_msgs.msg import RobotMotionController
from uv_msgs.msg import PidControllersState
from uv_msgs.msg import PidParams
from uv_msgs.msg import RobotAxis
from uv_msgs.msg import ServoSet
from uv_msgs.msg import TargetPosDown
from uv_msgs.msg import Yolov8
from uv_msgs.msg import LedControllers
from uv_msgs.msg import MagnetController
from uv_msgs.srv import DetectRequest

from sensor_msgs.msg import Image

from .config import FRONT_CAM_OFFSET, DOWN_CAM_OFFSET


class RobotInterface(Node):
    def __init__(self, name, opt):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        self.opt = opt

        self.robot = CoordinateSystems()
        self.MotionController = RobotMotionController()

        # PID参数初始化
        self.pid_parameters = PidParams()
        self.previous_error = 0.0
        self.i_error = 0.0
        self.dt = 1

        self.task_lock = True
        self.yolov8_data_down = Yolov8()
        self.yolov8_data_front = Yolov8()
        self.magnet = MagnetController()

        self.target = {"name": "none", "x": 0.0, "y": 0.0, "z": 0.0}
        self.backpoint = {"x": 0.0, "y": 0.0, "z": 0.0, "rz": 0.0}

        self.front_cam = CoordinateSystems()
        self.down_cam = CoordinateSystems()
        self._apply_camera_offsets()

        self.start_pos = CoordinateSystems()

        self.front_cam_Image_data = None
        self.down_cam_Image_data = None
        self.front_cam_left_Image_data = None
        self.segment_img = None

        # 话题发布
        self.target_pos_down_pub = self.create_publisher(
            TargetPosDown, "target_pos_down", 10
        )
        self.servo_control_pub = self.create_publisher(ServoSet, "servo_control", 10)
        self.led_controllers_pub = self.create_publisher(
            LedControllers, "led_controllers", 10
        )
        self.magnet_controller_pub = self.create_publisher(
            MagnetController, "magnet_controller", 10
        )
        self.pid_controllers_set_pub = self.create_publisher(
            PidControllersState, "pid_controllers_set", 10
        )
        self.line_patrol_img_pub = self.create_publisher(Image, "line_patrol_img", 10)
        self.openthrust_data_pub = self.create_publisher(
            RobotAxis, "openloop_thrust", 10
        )

        # 话题接收
        self.create_subscription(
            RobotMotionController,
            "motion_controller",
            self.motion_controller_callback,
            10,
        )
        self.create_subscription(
            Image, opt.front_topic[0], self.front_cam_callback, 10
        )
        self.create_subscription(Image, opt.down_topic[0], self.down_cam_callback, 10)
        self.create_subscription(
            Image, "front_cam/rectified/left", self.front_cam_left_callback, 10
        )
        self.create_subscription(
            PidParams, "track_pid_parameter", self.track_pid_parameter_callback, 10
        )
        self.create_subscription(Yolov8, "uv_detect_down", self.yolov8_down_callback, 10)
        self.create_subscription(
            Yolov8, "uv_detect_front", self.yolov8_front_callback, 10
        )
        self.create_subscription(
            Image, "binary_segment_img", self.segment_img_callback, 10
        )

        # 服务请求
        self.detect_request_client = self.create_client(
            DetectRequest, "uv_detect_srv"
        )

        self.magnet.state = 1
        self.magnet_controller_pub.publish(self.magnet)

        self.get_logger().info("节点初始化完成")

    def _apply_camera_offsets(self):
        # 统一从配置写入相机偏置，避免散落在逻辑里
        self.front_cam.base.vector.x = FRONT_CAM_OFFSET["x"]
        self.front_cam.base.vector.y = FRONT_CAM_OFFSET["y"]
        self.front_cam.base.vector.z = FRONT_CAM_OFFSET["z"]
        self.front_cam.base.vector.rx = FRONT_CAM_OFFSET["rx"]
        self.front_cam.base.vector.ry = FRONT_CAM_OFFSET["ry"]
        self.front_cam.base.vector.rz = FRONT_CAM_OFFSET["rz"]
        self.front_cam.base.extract()

        self.down_cam.base.vector.x = DOWN_CAM_OFFSET["x"]
        self.down_cam.base.vector.y = DOWN_CAM_OFFSET["y"]
        self.down_cam.base.vector.z = DOWN_CAM_OFFSET["z"]
        self.down_cam.base.vector.rx = DOWN_CAM_OFFSET["rx"]
        self.down_cam.base.vector.ry = DOWN_CAM_OFFSET["ry"]
        self.down_cam.base.vector.rz = DOWN_CAM_OFFSET["rz"]
        self.down_cam.base.extract()

        self.get_logger().info(
            f"前置摄像机偏置: x: {self.front_cam.base.vector.x:.2f} y: {self.front_cam.base.vector.y : .2f} z: {self.front_cam.base.vector.z : .2f}"
        )
        self.get_logger().info(
            f"          : rx: {self.front_cam.base.vector.rx:.2f} ry: {self.front_cam.base.vector.ry : .2f} rz: {self.front_cam.base.vector.rz : .2f}"
        )
        self.get_logger().info(
            f"下置摄像机偏置: x: {self.down_cam.base.vector.x:.2f} y: {self.down_cam.base.vector.y : .2f} z: {self.down_cam.base.vector.z : .2f}"
        )
        self.get_logger().info(
            f"          : rx: {self.down_cam.base.vector.rx:.2f} ry: {self.down_cam.base.vector.ry : .2f} rz: {self.down_cam.base.vector.rz : .2f}"
        )

    def track_pid_parameter_callback(self, data):
        pass
        # self.pid_parameters = data

    def front_cam_callback(self, data):
        self.front_cam_Image_data = data

    def down_cam_callback(self, data):
        self.down_cam_Image_data = data

    def front_cam_left_callback(self, data):
        self.front_cam_left_Image_data = data

    def segment_img_callback(self, data):
        self.segment_img = data

    def yolov8_down_callback(self, data):
        self.yolov8_data_down = data

    def yolov8_front_callback(self, data):
        self.yolov8_data_front = data

    def motion_controller_callback(self, data):
        self.MotionController = data

        self.robot.base.vector.x = self.MotionController.pos.x
        self.robot.base.vector.y = self.MotionController.pos.y
        self.robot.base.vector.z = self.MotionController.pos.z
        self.robot.base.vector.rx = self.MotionController.pos.rx
        self.robot.base.vector.ry = self.MotionController.pos.ry
        self.robot.base.vector.rz = self.MotionController.pos.rz
        self.robot.base.extract()
