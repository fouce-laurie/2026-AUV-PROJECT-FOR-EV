# ZIT6 心跳节点
# 原水下机器人硬件管理单元，已改造为仅负责发送 micro-ROS 解锁心跳

import rclpy
from rclpy.node import Node
import time
import threading

from std_msgs.msg import UInt32
from zit6_interfaces.msg import ZitStatus


class CoreNode(Node):
    """ZIT6 心跳节点 — 仅负责发送解锁心跳并监控解锁状态"""

    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("ZIT6 心跳节点已启动: %s" % name)

        # 心跳发布
        self.hbt_pub = self.create_publisher(UInt32, '/zit6/cmd/agxhbt', 10)
        self.hbt_mode = 1  

        # 状态订阅（仅用于日志监控）
        self.create_subscription(ZitStatus, '/zit6/state/status', self._on_status, 10)

        self._armed = False
        self._start_heartbeat()

    def _start_heartbeat(self):
        """启动心跳线程，20Hz"""
        def _loop():
            msg = UInt32()
            msg.data = self.hbt_mode
            while rclpy.ok():
                self.hbt_pub.publish(msg)
                time.sleep(0.05)  # 20Hz
        threading.Thread(target=_loop, daemon=True).start()

    def _on_status(self, s: ZitStatus):
        if s.is_armed and not self._armed:
            self.get_logger().info('已解锁')
            self._armed = True
        elif not s.is_armed and self._armed:
            self.get_logger().warn('已锁定')
            self._armed = False


def main(args=None):
    rclpy.init(args=args)
    node = CoreNode("uv_core")
    rclpy.spin(node)
    rclpy.shutdown()
