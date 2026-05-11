import math
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np
from cv_bridge import CvBridge

from stonefish_ros2.msg import DVL
from uv_msgs.msg import RobotAxis
from uv_msgs.msg import PidControllers
from uv_msgs.msg import PidParams
from uv_msgs.msg import RobotDeviceManager
from uv_msgs.msg import RobotMotionController
from uv_msgs.msg import TargetPosDown


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

        self.motion = RobotMotionController()
        self.device = RobotDeviceManager()
        self.pid_sim_state = PidControllers()

        self.thruster_pub = self.create_publisher(
            Float64MultiArray, "/auv/thrusters_cmd", 10
        )
        self.motion_pub = self.create_publisher(
            RobotMotionController, "motion_controller", 10
        )
        self.device_pub = self.create_publisher(
            RobotDeviceManager, "device_manager", 10
        )

        # Forward Stonefish stereo streams directly to legacy rectified topics.
        # This bridge does not run calibration/rectification; it only republishes.
        self.front_rect_left_pub = self.create_publisher(Image, "front_cam/rectified/left", 10)
        self.front_rect_right_pub = self.create_publisher(Image, "front_cam/rectified/right", 10)
        self.down_rect_left_pub = self.create_publisher(Image, "down_cam/rectified/left", 10)
        self.down_rect_right_pub = self.create_publisher(Image, "down_cam/rectified/right", 10)
        
        # Add publishers for stitched rectified topics
        self.front_rect_pub = self.create_publisher(Image, "front_cam/rectified", 10)
        self.down_rect_pub = self.create_publisher(Image, "down_cam/rectified", 10)
        
        # Initialize CvBridge for image processing
        self.bridge = CvBridge()
        
        # Store latest images
        self.front_left_img = None
        self.front_right_img = None
        self.down_left_img = None
        self.down_right_img = None

        # Sim defaults tuned for current Stonefish setup.
        self.pid_x = _Pid(0.8, 0.03, 0.12, i_limit=1.0)
        self.pid_y = _Pid(0.8, 0.03, 0.12, i_limit=1.0)
        self.pid_z = _Pid(1.2, 0.03, 0.8, i_limit=6.2, z_thrust_ff=0.23)
        self.pid_yaw = _Pid(0.008, 0.0, 0.0015, i_limit=0.6)
        self.position_ctrl_enabled = False
        self.current_pose_ready = False
        self.current_pose_ready = False
        self.target_cs = 0
        self.target_pos = RobotAxis()
        # store pending TargetPosDown if conversion must wait for odom
        self._pending_target_msg = None
        self.last_pid_time = self.get_clock().now()

        self.create_subscription(RobotAxis, "openloop_thrust", self._thrust_cb, 10)
        # Dedicated PID tuning topic for simulation only. Do not reuse real AUV pid_params_set.
        self.create_subscription(PidParams, "pid_sim", self._pid_sim_cb, 10)
        self.create_subscription(TargetPosDown, "target_pos_down", self._target_pos_cb, 10)
        self.create_subscription(Odometry, "/auv/odometry", self._odom_cb, 10)
        self.create_subscription(Imu, "/auv/imu", self._imu_cb, 10)
        self.create_subscription(DVL, "/auv/dvl", self._dvl_cb, 10)
        self.create_subscription(FluidPressure, "/auv/pressure", self._pressure_cb, 10)
        self.create_subscription(Image, "/sim/front_cam/left/image_color", self._front_left_img_cb, 10)
        self.create_subscription(Image, "/sim/front_cam/right/image_color", self._front_right_img_cb, 10)
        self.create_subscription(Image, "/sim/down_cam/left/image_color", self._down_left_img_cb, 10)
        self.create_subscription(Image, "/sim/down_cam/right/image_color", self._down_right_img_cb, 10)

        self.create_timer(0.05, self._publish_state)
        self.create_timer(0.05, self._position_control_step)
        self.pid_sim_pub = self.create_publisher(PidControllers, "pid_sim_state", 10)
        self._refresh_pid_sim_state()
        self.get_logger().info("uv_sim_bridge started with 6-thruster Beyond layout support")

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
                # Convert ROS images to OpenCV format
                left_cv = self.bridge.imgmsg_to_cv2(self.front_left_img, "bgr8")
                right_cv = self.bridge.imgmsg_to_cv2(self.front_right_img, "bgr8")
                
                # Stitch images horizontally
                stitched = np.hstack((left_cv, right_cv))
                
                # Convert back to ROS Image message
                stitched_msg = self.bridge.cv2_to_imgmsg(stitched, "bgr8")
                stitched_msg.header = self.front_left_img.header
                
                # Publish stitched image
                self.front_rect_pub.publish(stitched_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to stitch front images: {str(e)}")

    def _publish_stitched_down(self) -> None:
        if self.down_left_img and self.down_right_img:
            try:
                # Convert ROS images to OpenCV format
                left_cv = self.bridge.imgmsg_to_cv2(self.down_left_img, "bgr8")
                right_cv = self.bridge.imgmsg_to_cv2(self.down_right_img, "bgr8")
                
                # Stitch images horizontally
                stitched = np.hstack((left_cv, right_cv))
                
                # Convert back to ROS Image message
                stitched_msg = self.bridge.cv2_to_imgmsg(stitched, "bgr8")
                stitched_msg.header = self.down_left_img.header
                
                # Publish stitched image
                self.down_rect_pub.publish(stitched_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to stitch down images: {str(e)}")

    def _thrust_cb(self, data: RobotAxis) -> None:
        # Manual thrust command has priority over position control.
        self.position_ctrl_enabled = False
        self._publish_thrust_from_axis(data)

    def _target_pos_cb(self, data: TargetPosDown) -> None:
        # Support multiple coordinate systems:
        # cs==0: world coordinates (existing behavior)
        # cs==1: robot/body coordinates (convert to world using current pose)
        # cs==2: stepping coordinates (offsets applied to current target/world pose)
        try:
            cs = int(data.cs)
        except Exception:
            cs = 0


        # Convert the incoming target into world-frame coordinates
        self._apply_target_msg(data)

    def _pid_sim_cb(self, data: PidParams) -> None:
        name = str(data.name).strip().lower()
        target = {
            "x": self.pid_x,
            "y": self.pid_y,
            "z": self.pid_z,
            "rz": self.pid_yaw,
        }.get(name)

        if name == "z_ff":
            self.pid_z.z_thrust_ff = float(data.p)
            self.pid_z.reset()
            self._refresh_pid_sim_state()
            self.get_logger().info("pid_sim set z_ff=%.4f" % self.pid_z.z_thrust_ff)
            return

        if target is None:
            self.get_logger().warn(
                "pid_sim ignored unknown axis '%s', use x/y/z/rz or z_ff" % name
            )
            return

        target.kp = float(data.p)
        target.ki = float(data.i)
        target.kd = float(data.d)
        target.i_limit = abs(float(data.i_limit))
        target.reset()
        self._refresh_pid_sim_state()
        self.get_logger().info(
            "pid_sim set %s: kp=%.4f ki=%.4f kd=%.4f i_limit=%.4f"
            % (name, target.kp, target.ki, target.kd, target.i_limit)
        )

    def _fill_pid_param(self, pid_msg: PidParams, name: str, pid: _Pid) -> None:
        pid_msg.name = name
        pid_msg.p = float(pid.kp)
        pid_msg.i = float(pid.ki)
        pid_msg.d = float(pid.kd)
        pid_msg.i_limit = float(pid.i_limit)
        pid_msg.output_limit = 0.0

    def _refresh_pid_sim_state(self) -> None:
        self._fill_pid_param(self.pid_sim_state.x, "x", self.pid_x)
        self._fill_pid_param(self.pid_sim_state.y, "y", self.pid_y)
        self._fill_pid_param(self.pid_sim_state.z, "z", self.pid_z)
        self._fill_pid_param(self.pid_sim_state.rz, "rz", self.pid_yaw)
        self._fill_pid_param(self.pid_sim_state.rx, "rx", _Pid(0.0, 0.0, 0.0))
        self._fill_pid_param(self.pid_sim_state.ry, "ry", _Pid(0.0, 0.0, 0.0))
        self._fill_pid_param(self.pid_sim_state.vx, "vx", _Pid(0.0, 0.0, 0.0))
        self._fill_pid_param(self.pid_sim_state.vy, "vy", _Pid(0.0, 0.0, 0.0))
        self.pid_sim_state.z.output_limit = float(self.pid_z.z_thrust_ff)

    def _publish_thrust_from_axis(self, data: RobotAxis) -> None:
        # Mixer derived from girona500auv_full.scn actuator order:
        # [0] ThrusterSurgePort, [1] ThrusterSurgeStarboard,
        # [2] ThrusterHeaveBow, [3] ThrusterHeaveStern,
        # [4] ThrusterSwayBow, [5] ThrusterSwayStern.
        #
        # Horizontal thrusters are mounted at +/-45 and +/-135 degrees, so
        # x/y/rz are allocated together on [0,1,4,5]. z is allocated on [2,3].
        x = float(data.y)
        y = -float(data.x)
        z = float(data.z)
        rz = float(data.rz)

        # Geometry-consistent open-loop allocation for 4 diagonal horizontal thrusters.
        h0 = (x + y + rz*0.25)  # SurgePort
        h1 = (x - y - rz*0.25)  # SurgeStarboard
        h4 = -(x - y + rz*0.25)   # SwayBow
        h5 = -(x + y - rz*0.25)   # SwayStern

        # Heave thrusters share z command equally.
        h2 = z
        h3 = z

        raw = [h0, h1, h2, h3, h4, h5]

        # Scaling logic for Stonefish [-1, 1] range
        # Only normalize horizontal thrusters indices 0,1,4,5. Leave heave (2,3) unchanged.
        horiz_indices = (0, 1, 4, 5)
        horiz_max_abs = max(abs(raw[i]) for i in horiz_indices)
        if horiz_max_abs > 1.0:
            scale = 1.0 / horiz_max_abs
            for i in horiz_indices:
                raw[i] = raw[i] * scale

        # Heave thrusters (2, 3) must be clamped to [-1.0, 1.0] to prevent Stonefish from rejecting the entire array.
        raw[2] = _clamp(raw[2], -1.0, 1.0)
        raw[3] = _clamp(raw[3], -1.0, 1.0)

        cmd = Float64MultiArray()
        cmd.data = raw
        self.thruster_pub.publish(cmd)

        for i in range(len(raw)):
            if i < 8: # RobotMotionController msg thrust limit
                self.motion.thrust.thrust[i] = float(raw[i])

    def _position_control_step(self) -> None:
        if not self.position_ctrl_enabled or not self.current_pose_ready:
            return

        now = self.get_clock().now()
        dt = (now - self.last_pid_time).nanoseconds / 1e9
        self.last_pid_time = now
        dt = _clamp(dt, 0.001, 0.2)

        # Current pose in world frame.
        x = self.motion.pos.x
        y = self.motion.pos.y
        z = self.motion.pos.z
        yaw = self.motion.pos.rz # Assuming this is in degrees now in odom call

        # target_pos_down uses world frame in current workflow. Keep cs for compatibility.
        ex_world = self.target_pos.x - x
        ey_world = self.target_pos.y - y

        # Convert yaw to radians for rotation matrix
        yaw_rad = math.radians(yaw)
        cy = math.cos(yaw_rad)
        sy = math.sin(yaw_rad)
        ex_body = cy * ex_world + sy * ey_world
        ey_body = -sy * ex_world + cy * ey_world

        ez = self.target_pos.z - z
        eyaw = self._wrap_angle_deg(self.target_pos.rz - yaw)

        cmd = RobotAxis()
        cmd.x = float(self.pid_x.step(ex_body, dt))
        cmd.y = float(self.pid_y.step(ey_body, dt))
        cmd.z = float(self.pid_z.step(ez, dt))  # 移除了 * abs(cmd.y)，恢复正常下潜
        cmd.rz = -float(self.pid_yaw.step(eyaw, dt))

        # Mirror low-level controller outputs for observability.
        self.motion.tpos_inbase.x = ex_body
        self.motion.tpos_inbase.y = ey_body
        self.motion.tpos_inbase.z = ez
        self.motion.tpos_inbase.rz = eyaw

        self._publish_thrust_from_axis(cmd)

    def _apply_target_msg(self, data: TargetPosDown) -> None:
        # Convert TargetPosDown message into world coordinates and enable position control.
        cs = int(data.cs) if hasattr(data, 'cs') else 0
        tx = float(data.pos.x)
        ty = float(data.pos.y)
        tz = float(data.pos.z)
        trz = float(data.pos.rz)

        if cs == 0:
            world_x = tx
            world_y = ty
            world_z = tz
            world_rz = trz

        elif cs == 1:
            # Robot/body coordinates -> world: rotate by current yaw and translate by current pose
            yaw = self.motion.pos.rz
            yaw_rad = math.radians(yaw)
            cy = math.cos(yaw_rad)
            sy = math.sin(yaw_rad)
            world_x = self.motion.pos.x + cy * tx - sy * ty
            world_y = self.motion.pos.y + sy * tx + cy * ty
            world_z = self.motion.pos.z + tz
            world_rz = self._wrap_angle_deg(self.motion.pos.rz + trz)

        elif cs == 2:
            # Stepping coordinates: interpreted as offsets relative to current target (or robot pose)
            base_x = self.target_pos.x if (self.target_cs == 0) else self.motion.pos.x
            base_y = self.target_pos.y if (self.target_cs == 0) else self.motion.pos.y
            base_z = self.target_pos.z if (self.target_cs == 0) else self.motion.pos.z
            base_rz = self.target_pos.rz if (self.target_cs == 0) else self.motion.pos.rz

            yaw = self.motion.pos.rz
            yaw_rad = math.radians(yaw)
            cy = math.cos(yaw_rad)
            sy = math.sin(yaw_rad)
            # step offset provided in robot frame -> rotate into world and add
            off_x = cy * tx - sy * ty
            off_y = sy * tx + cy * ty
            world_x = base_x + off_x
            world_y = base_y + off_y
            world_z = base_z + tz
            world_rz = self._wrap_angle_deg(base_rz + trz)

        else:
            # Unknown cs: treat as world
            world_x = tx
            world_y = ty
            world_z = tz
            world_rz = trz

        self.target_cs = 0
        self.target_pos.x = float(world_x)
        self.target_pos.y = float(world_y)
        self.target_pos.z = float(world_z)
        self.target_pos.rz = float(world_rz)

        self.motion.tpos_inworld.x = self.target_pos.x
        self.motion.tpos_inworld.y = self.target_pos.y
        self.motion.tpos_inworld.z = self.target_pos.z
        self.motion.tpos_inworld.rz = self.target_pos.rz

        self.position_ctrl_enabled = True
        self.last_pid_time = self.get_clock().now()
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()
        self._pending_target_msg = None
        self.get_logger().info(
            "target_pos_down applied (world): target=(%.2f, %.2f, %.2f, yaw=%.2f)|position:(%.2f, %.2f, %.2f, yaw=%.2f)"
            % (self.target_pos.x, self.target_pos.y, self.target_pos.z, self.target_pos.rz,
               self.motion.pos.x, self.motion.pos.y, self.motion.pos.z, self.motion.pos.rz)
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self.motion.pos.x = -msg.pose.pose.position.y
        self.motion.pos.y = msg.pose.pose.position.x
        self.motion.pos.z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(q.x, q.y, q.z, q.w)
        # Convert to degrees for consistency across topics
        self.motion.pos.rx = math.degrees(roll)
        self.motion.pos.ry = math.degrees(pitch)
        self.motion.pos.rz = math.degrees(yaw)

        self.motion.imu.pos.x = self.motion.pos.y
        self.motion.imu.pos.y = -self.motion.pos.x
        self.motion.imu.pos.z = self.motion.pos.z
        self.motion.imu.pos.rx = self.motion.pos.rx
        self.motion.imu.pos.ry = self.motion.pos.ry
        self.motion.imu.pos.rz = self.motion.pos.rz

        self.motion.imu.spd.x = msg.twist.twist.linear.y
        self.motion.imu.spd.y = -msg.twist.twist.linear.x
        self.motion.imu.spd.z = msg.twist.twist.linear.z
        self.motion.imu.spd.rx = math.degrees(msg.twist.twist.angular.x)
        self.motion.imu.spd.ry = math.degrees(msg.twist.twist.angular.y)
        self.motion.imu.spd.rz = math.degrees(msg.twist.twist.angular.z)
        self.current_pose_ready = True
        # If there's a pending target waiting for odom, apply it now
        if self._pending_target_msg is not None:
            try:
                self._apply_target_msg(self._pending_target_msg)
            except Exception:
                self.get_logger().warn("Failed to apply pending target after odom arrival")

    def _imu_cb(self, msg: Imu) -> None:
        self.motion.imu.spd.rx = math.degrees(msg.angular_velocity.x)
        self.motion.imu.spd.ry = math.degrees(msg.angular_velocity.y)
        self.motion.imu.spd.rz = math.degrees(msg.angular_velocity.z)

    def _dvl_cb(self, msg: DVL) -> None:
        self.motion.imu.dvl = 2
        self.motion.imu.spd.x = msg.velocity.y
        self.motion.imu.spd.y = -msg.velocity.x
        self.motion.imu.spd.z = msg.velocity.z

    def _pressure_cb(self, msg: FluidPressure) -> None:
        # Approximate depth from pressure for compatibility with legacy fields.
        self.device.vol = float(msg.fluid_pressure)

    def _publish_state(self) -> None:
        self.motion.imu.mode = 4
        self.motion_pub.publish(self.motion)
        self.device_pub.publish(self.device)
        self.pid_sim_pub.publish(self.pid_sim_state)

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