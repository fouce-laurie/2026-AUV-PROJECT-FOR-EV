import math
import time
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, Float32, UInt8, Float32MultiArray
import cv2
import numpy as np
from cv_bridge import CvBridge

from stonefish_ros2.msg import DVL
from zit6_interfaces.msg import ZitSetpoint, ZitStatus


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class _Pid:
    kp: float
    ki: float
    kd: float
    i_limit: float = 0.5
    z_thrust_ff: float = 0.0  # Feedforward for Z axis to offset buoyancy

    def __post_init__(self) -> None:
        self.integral = 0.0
        self.prev_err = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_err = 0.0

    def step(self, err: float, dt: float) -> float:
        if dt <= 0.0:
            return self.kp * err + self.z_thrust_ff
        self.integral = _clamp(self.integral + err * dt, -self.i_limit, self.i_limit)
        derivative = (err - self.prev_err) / dt
        self.prev_err = err
        return self.kp * err + self.ki * self.integral + self.kd * derivative + self.z_thrust_ff


class UvSimBridge(Node):
    def __init__(self) -> None:
        super().__init__("uv_sim_bridge")

        # Internal state (replaces RobotMotionController / RobotDeviceManager)
        self.pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}   # degrees
        self.vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}   # deg/s
        self.thrust = [0.0] * 6  # 6-thruster normalized [-1, 1]
        self.target_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}
        self.target_cs = 0

        self.force_4dof = [0.0, 0.0, 0.0, 0.0]  # [Fx, Fy, Fz, Mz] body-frame
        self.vel_ctrl_enabled = False
        self.target_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}
        self.vel_world = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}
        self._current_mode = 0  # 0=NONE, 1=POS, 2=VEL, 3=FORCE

        # Thruster publisher (Stonefish)
        self.thruster_pub = self.create_publisher(
            Float64MultiArray, "/auv/thrusters_cmd", 10
        )

        # ZIT6 state publishers (simulating firmware)
        self.zit6_status_pub = self.create_publisher(ZitStatus, "/zit6/state/status", 10)
        self.zit6_pos_pub = self.create_publisher(Float32MultiArray, "/zit6/state/pos", 10)
        self.zit6_vel_pub = self.create_publisher(Float32MultiArray, "/zit6/state/vel", 10)
        self.zit6_thr_pub = self.create_publisher(Float32MultiArray, "/zit6/state/thr", 10)

        # Image republishing (unchanged)
        self.front_rect_left_pub = self.create_publisher(Image, "front_cam/rectified/left", 10)
        self.front_rect_right_pub = self.create_publisher(Image, "front_cam/rectified/right", 10)
        self.down_rect_left_pub = self.create_publisher(Image, "down_cam/rectified/left", 10)
        self.down_rect_right_pub = self.create_publisher(Image, "down_cam/rectified/right", 10)
        self.front_rect_pub = self.create_publisher(Image, "front_cam/rectified", 10)
        self.down_rect_pub = self.create_publisher(Image, "down_cam/rectified", 10)

        self.bridge = CvBridge()
        self.front_left_img = None
        self.front_right_img = None
        self.down_left_img = None
        self.down_right_img = None

        # PID controllers (same tuning as before)
        self.pid_x = _Pid(1600.0, 130.0, 200.0, i_limit=1000.0)
        self.pid_y = _Pid(1200.0, 120.0, 150.0, i_limit=1000.0)
        self.pid_z = _Pid(1200.0, 30.0, 800.0, i_limit=6200.0, z_thrust_ff=230.0)
        self.pid_yaw = _Pid(2, 1.0, 0.1, i_limit=30.0)
        self.pid_yaw_rate = _Pid(30.0, 20.0, 1.0, i_limit=600.0)

        # Velocity-loop PID controllers (body-frame)
        self.pid_vx = _Pid(300.0, 50.0, 10.0, i_limit=500.0)
        self.pid_vy = _Pid(300.0, 50.0, 10.0, i_limit=500.0)
        self.pid_vz = _Pid(300.0, 30.0, 50.0, i_limit=500.0, z_thrust_ff=230.0)
        self.pid_vyaw = _Pid(50.0, 20.0, 2.0, i_limit=300.0)
        self.position_ctrl_enabled = False
        self.current_pose_ready = False
        self.last_pid_time = self.get_clock().now()

        # ZIT6 command subscriptions
        self.create_subscription(ZitSetpoint, "/zit6/cmd/setpoint", self._setpoint_cb, 10)
        self.create_subscription(Float32, "/zit6/cmd/servo", self._servo_cb, 10)
        self.create_subscription(UInt8, "/zit6/cmd/light", self._light_cb, 10)

        # Simulator sensor subscriptions (unchanged)
        self.create_subscription(Odometry, "/auv/odometry", self._odom_cb, 10)
        self.create_subscription(Imu, "/auv/imu", self._imu_cb, 10)
        self.create_subscription(DVL, "/auv/dvl", self._dvl_cb, 10)
        self.create_subscription(FluidPressure, "/auv/pressure", self._pressure_cb, 10)
        self.create_subscription(Image, "/sim/front_cam/left/image_color", self._front_left_img_cb, 10)
        self.create_subscription(Image, "/sim/front_cam/right/image_color", self._front_right_img_cb, 10)
        self.create_subscription(Image, "/sim/down_cam/left/image_color", self._down_left_img_cb, 10)
        self.create_subscription(Image, "/sim/down_cam/right/image_color", self._down_right_img_cb, 10)

        self.create_timer(0.05, self._publish_state)
        self.create_timer(0.05, self._control_step)
        self.get_logger().info("uv_sim_bridge started (ZIT6 protocol)")

    # ── ZIT6 command callbacks ──────────────────────────────────────

    def _setpoint_cb(self, msg: ZitSetpoint) -> None:
        mode = msg.control_key & 0x03       # 0=pos, 1=vel, 2=force
        frame = msg.control_key & 0x10      # 0=world, 0x10=body
        incremental = msg.control_key & 0x20  # 0=absolute, 0x20=incremental

        if mode == 2:
            # Force mode: direct thrust, disable position/velocity control
            self.position_ctrl_enabled = False
            self.vel_ctrl_enabled = False
            self._current_mode = 3  # FORCE
            x = msg.x if (msg.type_mask & 0x01) else 0.0
            y = msg.y if (msg.type_mask & 0x02) else 0.0
            z = msg.z if (msg.type_mask & 0x04) else 0.0
            rz = msg.yaw if (msg.type_mask & 0x08) else 0.0
            self._publish_thrust_from_4dof(x, y, z, rz)

        elif mode == 1:
            # Velocity mode: body-frame velocity targets
            self.position_ctrl_enabled = False
            self.vel_ctrl_enabled = True
            self._current_mode = 2  # VEL
            if msg.type_mask & 0x01:
                self.target_vel['x'] = msg.x
            if msg.type_mask & 0x02:
                self.target_vel['y'] = msg.y
            if msg.type_mask & 0x04:
                self.target_vel['z'] = msg.z
            if msg.type_mask & 0x08:
                self.target_vel['rz'] = math.degrees(msg.yaw)
            self.last_pid_time = self.get_clock().now()
            self.pid_vx.reset()
            self.pid_vy.reset()
            self.pid_vz.reset()
            self.pid_vyaw.reset()
            self.get_logger().info(
                "setpoint vel applied: target=(%.2f, %.2f, %.2f, yaw_rate=%.2f)"
                % (self.target_vel['x'], self.target_vel['y'], self.target_vel['z'], self.target_vel['rz'])
            )

        elif mode == 0:
            self.vel_ctrl_enabled = False
            self._current_mode = 1  # POS
            # Position mode
            yaw_deg = math.degrees(msg.yaw)

            if incremental:
                # Incremental: body-frame offsets → world-frame, then apply
                dx = msg.x if (msg.type_mask & 0x01) else 0.0
                dy = msg.y if (msg.type_mask & 0x02) else 0.0
                yaw_rad = math.radians(self.pos['rz'])
                cy = math.cos(yaw_rad)
                sy = math.sin(yaw_rad)
                self.target_pos['x'] += cy * dx - sy * dy
                self.target_pos['y'] += sy * dx + cy * dy
                if msg.type_mask & 0x04:
                    self.target_pos['z'] += msg.z
                if msg.type_mask & 0x08:
                    self.target_pos['rz'] += yaw_deg
            else:
                # Absolute: set target directly
                if msg.type_mask & 0x01:
                    self.target_pos['x'] = msg.x
                if msg.type_mask & 0x02:
                    self.target_pos['y'] = msg.y
                if msg.type_mask & 0x04:
                    self.target_pos['z'] = msg.z
                if msg.type_mask & 0x08:
                    self.target_pos['rz'] = yaw_deg

            self.position_ctrl_enabled = True
            self.last_pid_time = self.get_clock().now()
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()
            self.pid_yaw.reset()
            self.pid_yaw_rate.reset()

            self.get_logger().info(
                "setpoint pos applied: target=(%.2f, %.2f, %.2f, yaw=%.2f) | pos=(%.2f, %.2f, %.2f, yaw=%.2f)"
                % (self.target_pos['x'], self.target_pos['y'], self.target_pos['z'], self.target_pos['rz'],
                   self.pos['x'], self.pos['y'], self.pos['z'], self.pos['rz'])
            )

    def _servo_cb(self, msg: Float32) -> None:
        # Simulation: no physical servo, just log
        pass

    def _light_cb(self, msg: UInt8) -> None:
        # Simulation: no physical LED, just log
        pass

    # ── Image callbacks (unchanged) ─────────────────────────────────

    def _front_left_img_cb(self, msg: Image) -> None:
        self.front_rect_left_pub.publish(msg)
        self.front_left_img = msg
        self._publish_stitched_front()

    def _front_right_img_cb(self, msg: Image) -> None:
        self.front_rect_right_pub.publish(msg)
        self.front_right_img = msg
        self._publish_stitched_front()

    def _down_left_img_cb(self, msg: Image) -> None:
        self.down_rect_left_pub.publish(msg)
        self.down_left_img = msg
        self._publish_stitched_down()

    def _down_right_img_cb(self, msg: Image) -> None:
        self.down_rect_right_pub.publish(msg)
        self.down_right_img = msg
        self._publish_stitched_down()

    def _publish_stitched_front(self) -> None:
        if self.front_left_img and self.front_right_img:
            try:
                left_cv = self.bridge.imgmsg_to_cv2(self.front_left_img, "bgr8")
                right_cv = self.bridge.imgmsg_to_cv2(self.front_right_img, "bgr8")
                stitched = np.hstack((left_cv, right_cv))
                stitched_msg = self.bridge.cv2_to_imgmsg(stitched, "bgr8")
                stitched_msg.header = self.front_left_img.header
                self.front_rect_pub.publish(stitched_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to stitch front images: {str(e)}")

    def _publish_stitched_down(self) -> None:
        if self.down_left_img and self.down_right_img:
            try:
                left_cv = self.bridge.imgmsg_to_cv2(self.down_left_img, "bgr8")
                right_cv = self.bridge.imgmsg_to_cv2(self.down_right_img, "bgr8")
                stitched = np.hstack((left_cv, right_cv))
                stitched_msg = self.bridge.cv2_to_imgmsg(stitched, "bgr8")
                stitched_msg.header = self.down_left_img.header
                self.down_rect_pub.publish(stitched_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to stitch down images: {str(e)}")

    # ── Thruster mixer ──────────────────────────────────────────────

    def _publish_thrust_from_4dof(self, x: float, y: float, z: float, rz: float) -> None:
        self.force_4dof = [x, y, z, rz]
        MAX_THRUST = 1000.0

        # NED body: x = surge (forward), y = sway (right)
        # 直接使用 NED body 力命令，匹配 xunyun 推进器几何
        h0 = ( x + y + rz * 0.5)   # T0: aft-stbd diagonal
        h1 = ( x - y - rz * 0.5)   # T1: aft-port diagonal
        h4 = -(x - y + rz * 0.5)   # T4: fwd-stbd diagonal
        h5 = -(x + y - rz * 0.5)   # T5: fwd-port diagonal

        h2 = z  # HeaveBow
        h3 = z  # HeaveStern

        raw = [h0, h1, h2, h3, h4, h5]
        for i in range(len(raw)):
            raw[i] = _clamp(raw[i] / MAX_THRUST, -1.0, 1.0)

        cmd = Float64MultiArray()
        cmd.data = raw
        self.thruster_pub.publish(cmd)

        for i in range(6):
            self.thrust[i] = float(raw[i])

    # ── Control loop ────────────────────────────────────────────────

    def _control_step(self) -> None:
        if not self.current_pose_ready:
            return

        now = self.get_clock().now()
        dt = (now - self.last_pid_time).nanoseconds / 1e9
        self.last_pid_time = now
        dt = _clamp(dt, 0.001, 0.2)

        if self.position_ctrl_enabled:
            self._position_pid_step(dt)
        elif self.vel_ctrl_enabled:
            self._velocity_pid_step(dt)

    def _position_pid_step(self, dt: float) -> None:
        x = self.pos['x']
        y = self.pos['y']
        z = self.pos['z']
        yaw = self.pos['rz']

        ex_world = self.target_pos['x'] - x
        ey_world = self.target_pos['y'] - y

        yaw_rad = math.radians(yaw)
        cy = math.cos(yaw_rad)
        sy = math.sin(yaw_rad)
        ex_body = cy * ex_world + sy * ey_world
        ey_body = -sy * ex_world + cy * ey_world

        ez = self.target_pos['z'] - z
        eyaw = self._wrap_angle_deg(self.target_pos['rz'] - yaw)

        target_yaw_rate = self.pid_yaw.step(eyaw, dt)
        target_yaw_rate = _clamp(target_yaw_rate, -45.0, 45.0)
        current_yaw_rate = self.vel['rz']
        eyaw_rate = target_yaw_rate - current_yaw_rate

        cmd_x = float(self.pid_x.step(ex_body, dt))
        cmd_y = float(self.pid_y.step(ey_body, dt))
        cmd_z = float(self.pid_z.step(ez, dt))
        cmd_rz = -float(self.pid_yaw_rate.step(eyaw_rate, dt))

        self._publish_thrust_from_4dof(cmd_x, cmd_y, cmd_z, cmd_rz)

    def _velocity_pid_step(self, dt: float) -> None:
        evx = self.target_vel['x'] - self.vel['x']
        evy = self.target_vel['y'] - self.vel['y']
        evz = self.target_vel['z'] - self.vel['z']
        evyaw = self.target_vel['rz'] - self.vel['rz']

        cmd_x = float(self.pid_vx.step(evx, dt))
        cmd_y = float(self.pid_vy.step(evy, dt))
        cmd_z = float(self.pid_vz.step(evz, dt))
        cmd_rz = float(self.pid_vyaw.step(evyaw, dt))

        self._publish_thrust_from_4dof(cmd_x, cmd_y, cmd_z, cmd_rz)

    # ── Simulator sensor callbacks ──────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        # 直接透传 Stonefish NED 坐标系
        self.pos['x'] = msg.pose.pose.position.x
        self.pos['y'] = msg.pose.pose.position.y
        self.pos['z'] = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(q.x, q.y, q.z, q.w)
        self.pos['rz'] = math.degrees(yaw)

        # 世界系速度
        vx_world = msg.twist.twist.linear.x
        vy_world = msg.twist.twist.linear.y
        self.vel_world['x'] = vx_world
        self.vel_world['y'] = vy_world
        self.vel_world['z'] = msg.twist.twist.linear.z
        self.vel_world['rz'] = math.degrees(msg.twist.twist.angular.z)

        # NED→FRD 旋转：世界系速度 → 机体系速度
        yaw_rad = yaw  # already in radians
        cy = math.cos(yaw_rad)
        sy = math.sin(yaw_rad)
        self.vel['x'] = vx_world * cy + vy_world * sy    # Forward
        self.vel['y'] = -vx_world * sy + vy_world * cy    # Right
        self.vel['z'] = msg.twist.twist.linear.z           # Down (shared)
        self.vel['rz'] = self.vel_world['rz']               # Yaw rate (shared)

        self.current_pose_ready = True

    def _imu_cb(self, msg: Imu) -> None:
        # Angular velocity in deg/s (overwrite odom angular for better fidelity)
        self.vel['rz'] = math.degrees(msg.angular_velocity.z)

    def _dvl_cb(self, msg: DVL) -> None:
        self.vel['x'] = msg.velocity.x
        self.vel['y'] = msg.velocity.y
        self.vel['z'] = msg.velocity.z

    def _pressure_cb(self, msg: FluidPressure) -> None:
        pass  # Could store depth if needed

    # ── ZIT6 state publishing ───────────────────────────────────────

    def _publish_state(self) -> None:
        # ZitStatus
        status = ZitStatus()
        status.is_armed = True
        status.arm_mode = 3
        status.control_level = self._current_mode
        status.ins_state = 3  # SINS/GPS/DVL
        status.navigation_ready = True
        status.forces = [float(self.force_4dof[i]) for i in range(4)]
        status.cycle_time_ms = 50.0
        status.battery_voltage = 16.8
        status.error_flags = 0
        self.zit6_status_pub.publish(status)

        # pos [x, y, z, yaw(rad)]
        pos_msg = Float32MultiArray()
        pos_msg.data = [self.pos['x'], self.pos['y'], self.pos['z'], math.radians(self.pos['rz'])]
        self.zit6_pos_pub.publish(pos_msg)

        # vel [vx, vy, vz, vyaw(rad/s)]
        vel_msg = Float32MultiArray()
        vel_msg.data = [self.vel['x'], self.vel['y'], self.vel['z'], math.radians(self.vel['rz'])]
        self.zit6_vel_pub.publish(vel_msg)

        # thr [Fx, Fy, Fz, Mz]
        thr_msg = Float32MultiArray()
        thr_msg.data = [float(self.force_4dof[i]) for i in range(4)]
        self.zit6_thr_pub.publish(thr_msg)

    # ── Utilities ───────────────────────────────────────────────────

    @staticmethod
    def _quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def _wrap_angle_deg(angle: float) -> float:
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UvSimBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
