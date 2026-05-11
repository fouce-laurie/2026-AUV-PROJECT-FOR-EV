#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import tf2_ros
import math
try:
    import pyqtgraph.opengl as gl
    from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QPushButton, QWidget, QFileDialog
    from PySide6.QtCore import QTimer
    GUI_AVAILABLE = True
except Exception:
    GUI_AVAILABLE = False


def pointcloud2_to_xyz_array(cloud_msg):
    # 简化实现：假设 PointCloud2 使用 FLOAT32 且前 3 个字段为 x,y,z
    if cloud_msg.width == 0:
        return np.empty((0, 3), dtype=np.float32)
    if cloud_msg.point_step == 0:
        return np.empty((0, 3), dtype=np.float32)

    # Interpret raw bytes as float32 array
    try:
        floats = np.frombuffer(cloud_msg.data, dtype=np.float32)
    except Exception:
        return np.empty((0, 3), dtype=np.float32)

    floats_per_point = cloud_msg.point_step // 4
    if floats_per_point == 0:
        return np.empty((0, 3), dtype=np.float32)

    try:
        points = floats.reshape(-1, floats_per_point)[:, :3]
    except Exception:
        return np.empty((0, 3), dtype=np.float32)

    return points.astype(np.float32)


def quat_to_rot_matrix(qx, qy, qz, qw):
    # 返回 3x3 旋转矩阵，输入为四元数 (x,y,z,w)
    # 参考四元数转矩阵公式
    x = qx; y = qy; z = qz; w = qw
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    R = np.array([
        [1 - 2 * (yy + zz),     2 * (xy - wz),       2 * (xz + wy)],
        [2 * (xy + wz),         1 - 2 * (xx + zz),   2 * (yz - wx)],
        [2 * (xz - wy),         2 * (yz + wx),       1 - 2 * (xx + yy)]
    ], dtype=np.float32)
    return R


if GUI_AVAILABLE:
    class PointCloudRecorderWindow(QMainWindow):
        def __init__(self, recorder_node):
            super().__init__()
            self.setWindowTitle('PointCloud Recorder')
            self.recorder = recorder_node

            central = QWidget()
            self.setCentralWidget(central)
            layout = QVBoxLayout(central)

            self.view = gl.GLViewWidget()
            layout.addWidget(self.view)

            # 控件
            btn_save = QPushButton('Save PLY')
            btn_clear = QPushButton('Clear')
            btn_save.clicked.connect(self.save_ply)
            btn_clear.clicked.connect(self.clear_points)
            layout.addWidget(btn_save)
            layout.addWidget(btn_clear)

            # 网格
            grid = gl.GLGridItem()
            self.view.addItem(grid)

            self.scatter = gl.GLScatterPlotItem()
            self.view.addItem(self.scatter)

            self.timer = QTimer()
            self.timer.timeout.connect(self.update_view)
            self.timer.start(100)  # 10 Hz

        def update_view(self):
            pts = self.recorder.get_accumulated_points()
            if pts.size == 0:
                return
            # 限制点数以保障交互
            max_pts = 200000
            if pts.shape[0] > max_pts:
                idx = np.linspace(0, pts.shape[0]-1, max_pts).astype(int)
                display = pts[idx]
            else:
                display = pts

            # pyqtgraph expects Nx3 float32
            self.scatter.setData(pos=display, size=1, color=(1.0, 1.0, 1.0, 1.0))

        def save_ply(self):
            pts = self.recorder.get_accumulated_points()
            if pts.size == 0:
                return
            filename, _ = QFileDialog.getSaveFileName(self, 'Save PLY', os.getcwd(), 'PLY Files (*.ply)')
            if not filename:
                return
            try:
                # 写入 ASCII PLY
                with open(filename, 'w') as f:
                    f.write('ply\n')
                    f.write('format ascii 1.0\n')
                    f.write(f'element vertex {pts.shape[0]}\n')
                    f.write('property float x\n')
                    f.write('property float y\n')
                    f.write('property float z\n')
                    f.write('end_header\n')
                    for p in pts:
                        f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')
            except Exception as e:
                print('保存 PLY 失败:', e)

        def clear_points(self):
            self.recorder.clear_points()
else:
    # GUI 不可用时，定义占位以避免 NameError
    PointCloudRecorderWindow = None
    def __init__(self, recorder_node):
        super().__init__()
        self.setWindowTitle('PointCloud Recorder')
        self.recorder = recorder_node

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        self.view = gl.GLViewWidget()
        layout.addWidget(self.view)

        # 控件
        btn_save = QPushButton('Save PLY')
        btn_clear = QPushButton('Clear')
        btn_save.clicked.connect(self.save_ply)
        btn_clear.clicked.connect(self.clear_points)
        layout.addWidget(btn_save)
        layout.addWidget(btn_clear)

        # 网格
        grid = gl.GLGridItem()
        self.view.addItem(grid)

        self.scatter = gl.GLScatterPlotItem()
        self.view.addItem(self.scatter)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_view)
        self.timer.start(100)  # 10 Hz

    def update_view(self):
        pts = self.recorder.get_accumulated_points()
        if pts.size == 0:
            return
        # 限制点数以保障交互
        max_pts = 200000
        if pts.shape[0] > max_pts:
            idx = np.linspace(0, pts.shape[0]-1, max_pts).astype(int)
            display = pts[idx]
        else:
            display = pts

        # pyqtgraph expects Nx3 float32
        self.scatter.setData(pos=display, size=1, color=(1.0, 1.0, 1.0, 1.0))

    def save_ply(self):
        pts = self.recorder.get_accumulated_points()
        if pts.size == 0:
            return
        filename, _ = QFileDialog.getSaveFileName(self, 'Save PLY', os.getcwd(), 'PLY Files (*.ply)')
        if not filename:
            return
        try:
            # 写入 ASCII PLY
            with open(filename, 'w') as f:
                f.write('ply\n')
                f.write('format ascii 1.0\n')
                f.write(f'element vertex {pts.shape[0]}\n')
                f.write('property float x\n')
                f.write('property float y\n')
                f.write('property float z\n')
                f.write('end_header\n')
                for p in pts:
                    f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')
        except Exception as e:
            print('保存 PLY 失败:', e)

    def clear_points(self):
        self.recorder.clear_points()


class PointCloudRecorderNode(Node):
    def __init__(self, topic_name='/depthmap_raw_points', downsample=5, max_points=500000, target_frame='world'):
        super().__init__('pc_recorder_node')
        self.sub = self.create_subscription(PointCloud2, topic_name, self.pc_callback, 10)
        self.accum = []
        self.downsample = max(1, int(downsample))
        self.max_points = int(max_points)
        # 支持通过 ROS 参数覆盖目标坐标系（例如: map/odom/world）
        try:
            self.declare_parameter('target_frame', target_frame)
            self.target_frame = self.get_parameter('target_frame').value
        except Exception:
            self.target_frame = target_frame

        # 用于避免重复打印同一缺失 frame 的警告
        self._warned_missing_transforms = set()

        # tf2 buffer & listener，用于把点云从其原始 frame 转换到目标世界坐标系
        try:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        except Exception as e:
            self.get_logger().warn(f'tf2 初始化失败，点云将不会被变换到世界坐标系: {e}')
            self.tf_buffer = None

    def pc_callback(self, msg: PointCloud2):
        try:
            pts = pointcloud2_to_xyz_array(msg)
            if pts.size == 0:
                return
            if self.downsample > 1:
                pts = pts[::self.downsample]
            # 如果可用，则把点从 msg.header.frame_id 变换到目标 frame（比如 'world'）
            src_frame = msg.header.frame_id if hasattr(msg, 'header') else ''
            transformed = pts
            if getattr(self, 'tf_buffer', None) is not None and src_frame and src_frame != self.target_frame:
                try:
                    # 尝试获取最近的可用变换（使用 latest）
                    trans = None
                    try:
                        trans = self.tf_buffer.lookup_transform(self.target_frame, src_frame, rclpy.time.Time())
                    except Exception:
                        # 尝试使用最新可用时间
                        trans = self.tf_buffer.lookup_transform(self.target_frame, src_frame, rclpy.time.Time())

                    if trans is not None:
                        tx = trans.transform.translation.x
                        ty = trans.transform.translation.y
                        tz = trans.transform.translation.z
                        qx = trans.transform.rotation.x
                        qy = trans.transform.rotation.y
                        qz = trans.transform.rotation.z
                        qw = trans.transform.rotation.w

                        R = quat_to_rot_matrix(qx, qy, qz, qw)
                        # 应用旋转与平移： p_world = R * p_src + t
                        transformed = (R.dot(pts.T)).T + np.array([tx, ty, tz], dtype=np.float32)
                except Exception as e:
                    key = (src_frame, self.target_frame)
                    # 只在第一次失败时打印详细警告，避免日志被大量刷屏
                    if key not in self._warned_missing_transforms:
                        self._warned_missing_transforms.add(key)
                        msg = f'点云变换失败（{src_frame} -> {self.target_frame}）: {e}.'
                        # 尝试获取可用 frame 列表以便用户诊断（若 tf_buffer 支持）
                        try:
                            frames_info = self.tf_buffer.all_frames_as_yaml()
                            msg += ' 可用 frames（YAML）：\n' + frames_info
                        except Exception:
                            msg += ' 请检查目标 frame 是否存在，或通过参数 target_frame 指定一个有效 frame。'
                        self.get_logger().warn(msg)
                    # 回退为不变换（使用源点云点）

            self.accum.append(transformed)
            # 限制总点数
            total = sum([a.shape[0] for a in self.accum])
            if total > self.max_points:
                # 合并并裁剪为最新 max_points
                allp = np.vstack(self.accum)
                allp = allp[-self.max_points:]
                self.accum = [allp]
        except Exception as e:
            self.get_logger().error(f'点云解析失败: {e}')

    def get_accumulated_points(self):
        if not self.accum:
            return np.empty((0, 3), dtype=np.float32)
        try:
            return np.vstack(self.accum)
        except Exception:
            return np.array(self.accum[0], copy=True)

    def clear_points(self):
        self.accum = []


def main():
    rclpy.init()
    if GUI_AVAILABLE:
        # 创建临时占位节点以便 GUI 创建
        dummy = Node('temp_pc')

        app = QApplication(sys.argv)
        recorder_node = PointCloudRecorderNode()
        window = PointCloudRecorderWindow(recorder_node)
        window.show()

        # 使用 QTimer 驱动 rclpy.spin_once
        ros_timer = QTimer()
        ros_timer.timeout.connect(lambda: rclpy.spin_once(recorder_node, timeout_sec=0.01))
        ros_timer.start(10)

        exit_code = app.exec()

        recorder_node.destroy_node()
        dummy.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)
    else:
        print('GUI 库不可用，进入无界面点云记录模式。安装 pyqtgraph 和 PySide6 可启用可视化。')
        recorder_node = PointCloudRecorderNode()
        try:
            rclpy.spin(recorder_node)
        except KeyboardInterrupt:
            pass

        # 退出时保存点云
        pts = recorder_node.get_accumulated_points()
        if pts.size:
            out = os.path.join(os.getcwd(), f'pc_recorder_{int(time.time())}.ply')
            try:
                with open(out, 'w') as f:
                    f.write('ply\nformat ascii 1.0\n')
                    f.write(f'element vertex {pts.shape[0]}\n')
                    f.write('property float x\nproperty float y\nproperty float z\n')
                    f.write('end_header\n')
                    for p in pts:
                        f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')
                print('Saved', out)
            except Exception as e:
                print('保存点云失败:', e)

        recorder_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
