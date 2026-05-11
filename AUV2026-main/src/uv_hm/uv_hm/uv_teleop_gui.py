import sys
import threading
import pygame
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from zit6_interfaces.msg import ZitSetpoint
from cv_bridge import CvBridge

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QProgressBar
)
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import QTimer, Qt, QObject, Signal

class TeleopNode(Node):
    def __init__(self):
        super().__init__('uv_teleop_gui')
        # Publisher for ZIT6 setpoint (force mode)
        self.cmd_pub = self.create_publisher(ZitSetpoint, '/zit6/cmd/setpoint', 10)
        
        # Subscribe to front camera
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            'front_cam/rectified/left',
            self.image_callback,
            30
        )
        self.current_image = None
        self.image_mutex = threading.Lock()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.image_mutex:
                self.current_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            
    def publish_thrust(self, x, y, z, rx, ry, rz):
        msg = ZitSetpoint()
        msg.control_key = 0x02 | 0x10  # force mode + body frame
        msg.type_mask = 0x0F
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.yaw = float(rz)  # drop rx, ry (4DOF only)
        self.cmd_pub.publish(msg)

class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("AUV ROV Teleop Control")
        
        self.joystick = None
        self.init_pygame()

        # UI Setup
        main_widget = QWidget()
        layout = QVBoxLayout()
        
        self.img_label = QLabel("No Camera Feed")
        self.img_label.setAlignment(Qt.AlignCenter)
        self.img_label.setMinimumSize(640, 480)
        self.img_label.setStyleSheet("background-color: black; color: white;")
        layout.addWidget(self.img_label)
        
        # Status layout
        status_layout = QHBoxLayout()
        
        self.status_label = QLabel("Status: Connecting Joystick...")
        status_layout.addWidget(self.status_label)
        
        layout.addLayout(status_layout)
        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)
        
        # Timers
        self.timer_ros = QTimer(self)
        self.timer_ros.timeout.connect(self.update_ros)
        self.timer_ros.start(30) # ~33Hz for ROS node spin and image update
        
        self.timer_joy = QTimer(self)
        self.timer_joy.timeout.connect(self.update_joystick)
        self.timer_joy.start(50) # 20Hz for joystick read

    def init_pygame(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

    def update_ros(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)
        
        with self.ros_node.image_mutex:
            img = self.ros_node.current_image
            
        if img is not None:
            # Convert OpenCV frame (BGR) to QImage (RGB)
            rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.img_label.setPixmap(pixmap.scaled(self.img_label.size(), Qt.KeepAspectRatio))

    def update_joystick(self):
        pygame.event.pump()
        
        if self.joystick is None:
            # Try to connect
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
            else:
                self.status_label.setText("Status: No Joystick Found")
                return
                
        # left stick: x (forward/back) and rz (yaw)
        # Assuming Axis 1 is left stick Y (surge, forward is negative)
        # Assuming Axis 0 is left stick X (yaw, right is positive)
        # right stick: y (sway left/right) and z (heave up/down)
        # Assuming Axis 4 is right stick Y (heave)
        # Assuming Axis 3 is right stick X (sway)
        
        # This mapping can vary slightly between OS and controllers
        # Adjust index if necessary
        try:
            left_y = self.joystick.get_axis(1)
            left_x = self.joystick.get_axis(0)
            
            # Common gamepad mappings:
            # Axis 0: Left Stick X
            # Axis 1: Left Stick Y
            # Axis 2: Left Trigger or Right Stick X
            # Axis 3: Right Stick X or Right Stick Y
            # Axis 4: Right Stick Y
            # We will use axis 3 and 4 for right stick.
            
            axes_count = self.joystick.get_numaxes()
            right_x = self.joystick.get_axis(3) if axes_count > 3 else 0.0
            right_y = self.joystick.get_axis(4) if axes_count > 4 else 0.0
            
            # Deadzone
            deadzone = 0.05
            
            # Surge: left stick Y (negative is forward usually, so we invert)
            surge = -left_y if abs(left_y) > deadzone else 0.0
            # Yaw: left stick X (positive is right)
            yaw = left_x if abs(left_x) > deadzone else 0.0
            
            # Sway: right stick X (positive is right)
            sway = right_x if abs(right_x) > deadzone else 0.0
            # Heave: right stick Y (negative is up, so invert if needed, depends on robot)
            heave = -right_y if abs(right_y) > deadzone else 0.0
            
            # Output ranges [-1, 1], scale if needed.
            # Convert to appropriate RobotAxis bounds
            
            # Publish thrust
            self.ros_node.publish_thrust(
                y=surge/2, # surge
                x=-sway/2,  # sway
                z=-heave/2, # heave
                rx=0.0,
                ry=0.0,
                rz=-yaw/2   # yaw
            )
            
            self.status_label.setText(f"Status: OK | FWD: {surge:.2f} YAW: {yaw:.2f} LAT: {sway:.2f} UP: {heave:.2f}")
            
        except pygame.error:
            self.joystick = None
            self.status_label.setText("Status: Joystick Disconnected")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    
    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()
    
    try:
        sys.exit(app.exec())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
