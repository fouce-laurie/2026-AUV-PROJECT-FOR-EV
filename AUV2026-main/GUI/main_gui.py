#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AUV 上位机综合控制与监控系统
- 3D 轨迹可视化 + 摄像头画面
- ROS2 节点管理 + 仿真启动
- ZIT6 通信全协议控制面板（无需手动计算二进制/十进制）
"""

import sys
import math
import rclpy
import subprocess
import os
import json
import time
import threading
import numpy as np
import pyqtgraph.opengl as gl
import cv2
from cv_bridge import CvBridge

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32, UInt8, Float32, Float32MultiArray

from PySide6.QtWidgets import (
    QMainWindow, QApplication, QMessageBox, QDialog,
    QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QWidget, QListWidgetItem,
    QTabWidget, QGroupBox, QDoubleSpinBox, QComboBox, QCheckBox, QSpinBox,
    QSlider, QLineEdit, QGridLayout, QSplitter, QFrame,
)
from PySide6.QtCore import QTimer, Qt, QThread, Signal
from PySide6.QtGui import QImage, QPixmap, QFont, QColor

from zit6_interfaces.msg import ZitSetpoint, ZitStatus, ZitPid, ZitPidStatus

# Local imports
from GUIForAUV import Ui_AUV_UI
from sim_settings import Ui_SimSettingsDialog
from uv_msgs.msg import RobotMotionController

# ── Constants ─────────────────────────────────────────────────────────────

CONTROL_MODES = {
    "位置环 (Position)": 0,
    "速度环 (Velocity)": 1,
    "推力环 (Force)": 2,
}

CONTROL_MODE_NAMES = {0: "位置环", 1: "速度环", 2: "推力环"}

COORD_FRAMES = {
    "世界系 (NED World)": 0,
    "机体系 (Body)": 1,
}

INS_STATES = {
    0: "待机 (Standby)",
    1: "粗对准 (Coarse)",
    2: "精对准 (Fine)",
    3: "SINS/GPS/DVL",
    4: "SINS/DVL",
    5: "MRU",
}

ERROR_FLAGS = {
    0x01: "力停 (Force Stop)",
    0x02: "传感器故障 (Sensor Fail)",
    0x04: "电压低 (Voltage Low)",
    0x08: "通信超时 (Comm Timeout)",
}

AXIS_NAMES = ["X", "Y", "Z", "Yaw"]

# ── Styles ────────────────────────────────────────────────────────────────

STYLE_GREEN = "background-color: #2ecc71; color: white; font-weight: bold; padding: 4px 12px; border-radius: 4px;"
STYLE_RED = "background-color: #e74c3c; color: white; font-weight: bold; padding: 4px 12px; border-radius: 4px;"
STYLE_YELLOW = "background-color: #f39c12; color: white; font-weight: bold; padding: 4px 12px; border-radius: 4px;"
STYLE_SEND = "background-color: #3498db; color: white; font-weight: bold; padding: 6px 16px; border-radius: 4px;"
STYLE_SEND_HOVER = "background-color: #2980b9;"
STYLE_QUICK = "background-color: #9b59b6; color: white; padding: 4px 8px; border-radius: 3px;"
GROUP_STYLE = "QGroupBox { font-weight: bold; border: 1px solid #bdc3c7; border-radius: 6px; margin-top: 8px; padding-top: 16px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }"


# ── Heartbeat Thread ──────────────────────────────────────────────────────

class HeartbeatThread(QThread):
    """独立线程以 20Hz 发送 ZIT6 解锁心跳"""
    status_update = Signal(str, int)  # (status_text, count)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self._running = False
        self._mode = 3
        self._count = 0

    def set_mode(self, mode):
        self._mode = mode

    def run(self):
        self._running = True
        self._count = 0
        while self._running:
            try:
                msg = UInt32()
                msg.data = self._mode
                self.node.hbt_pub.publish(msg)
                self._count += 1
                self.status_update.emit("发送中", self._count)
            except Exception:
                pass
            time.sleep(0.05)  # 20Hz

    def stop(self):
        self._running = False
        self.status_update.emit("已停止", self._count)

    @property
    def is_running(self):
        return self._running


# ── Sim Settings Dialog (unchanged) ──────────────────────────────────────

class SimSettingsDialog(QDialog, Ui_SimSettingsDialog):
    def __init__(self, parent=None, current_config=None):
        super().__init__(parent)
        self.setupUi(self)
        self.config = current_config or {}

        sim_cfg = self.config.get("simulation", {})
        scenario = sim_cfg.get("current_scenario", "test")
        index = self.combo_scenario.findText(scenario)
        if index >= 0:
            self.combo_scenario.setCurrentIndex(index)

        enable_ai = sim_cfg.get("enable_ai", False)
        self.check_enable_ai.setChecked(enable_ai)
        enable_depth = sim_cfg.get("enable_depth", False)
        try:
            self.check_enable_depth.setChecked(enable_depth)
        except Exception:
            pass

    def get_settings(self):
        return {
            "current_scenario": self.combo_scenario.currentText(),
            "enable_ai": self.check_enable_ai.isChecked(),
            "enable_depth": self.check_enable_depth.isChecked() if hasattr(self, 'check_enable_depth') else False,
        }


# ── Node Control Widget (unchanged) ──────────────────────────────────────

class NodeControlWidget(QWidget):
    def __init__(self, node_name, cmd, repo_root, parent=None):
        super().__init__(parent)
        self.cmd = cmd
        self.repo_root = repo_root
        self.process = None

        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        self.name_label = QLabel(node_name)
        layout.addWidget(self.name_label)

        self.toggle_btn = QPushButton("开启")
        self.toggle_btn.clicked.connect(self.toggle_node)
        layout.addWidget(self.toggle_btn)

    def toggle_node(self):
        if self.process and self.process.poll() is None:
            try:
                os.killpg(os.getpgid(self.process.pid), 9)
            except Exception:
                self.process.kill()
            self.process = None
            self.toggle_btn.setText("开启")
            self.toggle_btn.setStyleSheet("")
        else:
            full_cmd = f"source {self.repo_root}/scripts/env.sh && {self.cmd}"
            try:
                self.process = subprocess.Popen(["bash", "-c", full_cmd], preexec_fn=os.setsid)
                self.toggle_btn.setText("关闭")
                self.toggle_btn.setStyleSheet("background-color: red; color: white;")
            except Exception as e:
                print(f"Failed to start node {self.cmd}: {e}")


# ── Helper: make labeled spinbox ──────────────────────────────────────────

def make_spinbox(label_text, min_val=-100.0, max_val=100.0, step=0.1, decimals=3, suffix=""):
    row = QHBoxLayout()
    lbl = QLabel(label_text)
    lbl.setFixedWidth(50)
    lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
    sb = QDoubleSpinBox()
    sb.setRange(min_val, max_val)
    sb.setSingleStep(step)
    sb.setDecimals(decimals)
    if suffix:
        sb.setSuffix(suffix)
    sb.setMinimumWidth(120)
    row.addWidget(lbl)
    row.addWidget(sb)
    return row, sb


def make_info_label(text="---"):
    lbl = QLabel(text)
    lbl.setStyleSheet("font-family: monospace; font-size: 13px;")
    return lbl


# ══════════════════════════════════════════════════════════════════════════
#  Main Window
# ══════════════════════════════════════════════════════════════════════════

class AUVControlWindow(QMainWindow, Ui_AUV_UI):
    def __init__(self, ros_node):
        super().__init__()
        self.setupUi(self)
        self.ros_node = ros_node
        self.sim_process = None
        self.started_processes = []
        self.config_path = os.path.join(os.path.dirname(__file__), "config.json")
        self.load_config()
        self.repo_root = self.config.get("runtime", {}).get("repo_root", "/home/origin/AUV2026")

        self.setWindowTitle("AUV 上位机综合控制与监控系统")

        # ── Build the enhanced UI ──────────────────────────────────────
        self._build_enhanced_ui()

        # ── Heartbeat thread ───────────────────────────────────────────
        self.hbt_thread = None

        # ── Data caches ────────────────────────────────────────────────
        self.current_pos = [0.0, 0.0, 0.0, 0.0]  # x, y, z, yaw_deg
        self.current_vel = [0.0, 0.0, 0.0, 0.0]
        self.current_forces = [0.0, 0.0, 0.0, 0.0]
        self.latest_front_img = None
        self.latest_down_img = None

        # ZIT6 state cache
        self.zit6_status = None
        self.zit6_pid_status = None

        # ── Existing connections ───────────────────────────────────────
        self.update_status_labels()
        self.button_sim_lauch.clicked.connect(self.on_sim_launch_clicked)
        self.real_lauch_button.setText("结束仿真")
        self.real_lauch_button.clicked.connect(self.on_stop_sim_clicked)
        self.action.triggered.connect(self.on_open_sim_settings)

        # ── Update timer ───────────────────────────────────────────────
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_ui_state)
        self.update_timer.start(50)

    # ── Build enhanced UI ──────────────────────────────────────────────

    def _build_enhanced_ui(self):
        """Replace the right side (3D widget area) with a QTabWidget."""

        # Clear existing content layout items after leftPane
        # The UI has: mainLayout > [statement, statusRow, contentLayout]
        # contentLayout has: [leftPane, widget(3D)]
        # We replace widget with a QTabWidget

        # Remove the old 3D widget placeholder
        old_widget = self.widget
        old_widget.setParent(None)

        # Create tab widget
        self.tabs = QTabWidget()
        self.tabs.setMinimumSize(540, 420)

        # Tab 0: 3D View + Cameras
        self._build_3d_tab()

        # Tab 1: System Status
        self._build_status_tab()

        # Tab 2: Heartbeat
        self._build_heartbeat_tab()

        # Tab 3: Motion Control
        self._build_motion_tab()

        # Tab 4: PID Tuning
        self._build_pid_tab()

        # Tab 5: Peripherals
        self._build_peripherals_tab()

        self.contentLayout.addWidget(self.tabs)
        self.contentLayout.setStretch(1, 1)

    # ── Tab 0: 3D View + Cameras ──────────────────────────────────────

    def _build_3d_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # 3D View
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.setMinimumHeight(300)

        grid = gl.GLGridItem()
        grid.scale(2, 2, 1)
        self.gl_widget.addItem(grid)

        self.position_marker = gl.GLScatterPlotItem(
            pos=np.array([[0.0, 0.0, 0.0]]),
            color=np.array([[1.0, 0.0, 0.0, 1.0]]),
            size=12, pxMode=True)
        self.gl_widget.addItem(self.position_marker)

        self.path_data = []
        self.path_line = gl.GLLinePlotItem(
            pos=np.array([[0.0, 0.0, 0.0]]),
            color=np.array([[0.0, 0.4, 1.0, 1.0]]),
            width=2, antialias=True, mode="line_strip")
        self.gl_widget.addItem(self.path_line)

        layout.addWidget(self.gl_widget)

        btn_row = QHBoxLayout()
        self.btn_clear_path = QPushButton("清除轨迹")
        self.btn_clear_path.setFixedWidth(80)
        self.btn_clear_path.clicked.connect(self.clear_path)
        btn_row.addWidget(self.btn_clear_path)
        btn_row.addStretch()
        layout.addLayout(btn_row)

        # Cameras
        cam_row = QHBoxLayout()

        cam_front_group = QVBoxLayout()
        cam_front_group.addWidget(QLabel("前置摄像头"))
        self.label_front_cam = QLabel("休眠中")
        self.label_front_cam.setFixedSize(320, 240)
        self.label_front_cam.setAlignment(Qt.AlignCenter)
        self.label_front_cam.setStyleSheet("background-color: black; color: white;")
        cam_front_group.addWidget(self.label_front_cam)

        cam_down_group = QVBoxLayout()
        cam_down_group.addWidget(QLabel("下置摄像头"))
        self.label_down_cam = QLabel("休眠中")
        self.label_down_cam.setFixedSize(320, 240)
        self.label_down_cam.setAlignment(Qt.AlignCenter)
        self.label_down_cam.setStyleSheet("background-color: black; color: white;")
        cam_down_group.addWidget(self.label_down_cam)

        cam_row.addLayout(cam_front_group)
        cam_row.addLayout(cam_down_group)
        cam_row.addStretch()
        layout.addLayout(cam_row)

        self.tabs.addTab(tab, "3D 视图")

    # ── Tab 1: System Status ──────────────────────────────────────────

    def _build_status_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # Armed status
        armed_group = QGroupBox("解锁状态")
        armed_group.setStyleSheet(GROUP_STYLE)
        armed_layout = QHBoxLayout(armed_group)
        self.lbl_armed = QLabel("未知")
        self.lbl_armed.setStyleSheet(STYLE_YELLOW)
        self.lbl_armed.setFixedHeight(36)
        self.lbl_armed.setAlignment(Qt.AlignCenter)
        font = QFont()
        font.setPointSize(16)
        font.setBold(True)
        self.lbl_armed.setFont(font)
        armed_layout.addWidget(self.lbl_armed)
        layout.addWidget(armed_group)

        # ZIT6 Status Info
        info_group = QGroupBox("ZIT6 状态信息")
        info_group.setStyleSheet(GROUP_STYLE)
        info_grid = QGridLayout(info_group)

        row = 0
        info_grid.addWidget(QLabel("控制模式:"), row, 0)
        self.lbl_control_level = make_info_label()
        info_grid.addWidget(self.lbl_control_level, row, 1)

        info_grid.addWidget(QLabel("INS 状态:"), row, 2)
        self.lbl_ins_state = make_info_label()
        info_grid.addWidget(self.lbl_ins_state, row, 3)

        row += 1
        info_grid.addWidget(QLabel("导航就绪:"), row, 0)
        self.lbl_nav_ready = make_info_label()
        info_grid.addWidget(self.lbl_nav_ready, row, 1)

        info_grid.addWidget(QLabel("电池电压:"), row, 2)
        self.lbl_battery = make_info_label()
        info_grid.addWidget(self.lbl_battery, row, 3)

        row += 1
        info_grid.addWidget(QLabel("错误标志:"), row, 0)
        self.lbl_errors = make_info_label()
        info_grid.addWidget(self.lbl_errors, row, 1, 1, 3)

        row += 1
        info_grid.addWidget(QLabel("固件心跳:"), row, 0)
        self.lbl_fw_hbt = make_info_label()
        info_grid.addWidget(self.lbl_fw_hbt, row, 1)

        info_grid.addWidget(QLabel("控制周期:"), row, 2)
        self.lbl_cycle_time = make_info_label()
        info_grid.addWidget(self.lbl_cycle_time, row, 3)

        layout.addWidget(info_group)

        # Forces
        forces_group = QGroupBox("4-DOF 力反馈 [Fx, Fy, Fz, Mz]")
        forces_group.setStyleSheet(GROUP_STYLE)
        forces_layout = QHBoxLayout(forces_group)
        self.lbl_forces = [make_info_label("0.000") for _ in range(4)]
        for i, name in enumerate(["Fx:", "Fy:", "Fz:", "Mz:"]):
            forces_layout.addWidget(QLabel(name))
            forces_layout.addWidget(self.lbl_forces[i])
        layout.addWidget(forces_group)

        # NED Position & Velocity
        pos_group = QGroupBox("NED 位置 / 速度")
        pos_group.setStyleSheet(GROUP_STYLE)
        pos_grid = QGridLayout(pos_group)

        pos_grid.addWidget(QLabel("X (北):"), 0, 0)
        self.lbl_pos_x = make_info_label()
        pos_grid.addWidget(self.lbl_pos_x, 0, 1)
        pos_grid.addWidget(QLabel("Vx:"), 0, 2)
        self.lbl_vel_x = make_info_label()
        pos_grid.addWidget(self.lbl_vel_x, 0, 3)

        pos_grid.addWidget(QLabel("Y (东):"), 1, 0)
        self.lbl_pos_y = make_info_label()
        pos_grid.addWidget(self.lbl_pos_y, 1, 1)
        pos_grid.addWidget(QLabel("Vy:"), 1, 2)
        self.lbl_vel_y = make_info_label()
        pos_grid.addWidget(self.lbl_vel_y, 1, 3)

        pos_grid.addWidget(QLabel("Z (下):"), 2, 0)
        self.lbl_pos_z = make_info_label()
        pos_grid.addWidget(self.lbl_pos_z, 2, 1)
        pos_grid.addWidget(QLabel("Vz:"), 2, 2)
        self.lbl_vel_z = make_info_label()
        pos_grid.addWidget(self.lbl_vel_z, 2, 3)

        pos_grid.addWidget(QLabel("Yaw:"), 3, 0)
        self.lbl_pos_yaw = make_info_label()
        pos_grid.addWidget(self.lbl_pos_yaw, 3, 1)
        pos_grid.addWidget(QLabel("Vyaw:"), 3, 2)
        self.lbl_vel_yaw = make_info_label()
        pos_grid.addWidget(self.lbl_vel_yaw, 3, 3)

        layout.addWidget(pos_group)
        layout.addStretch()
        self.tabs.addTab(tab, "系统状态")

    # ── Tab 2: Heartbeat ──────────────────────────────────────────────

    def _build_heartbeat_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # Mode selection
        mode_group = QGroupBox("心跳设置")
        mode_group.setStyleSheet(GROUP_STYLE)
        mode_layout = QGridLayout(mode_group)

        mode_layout.addWidget(QLabel("心跳模式:"), 0, 0)
        self.combo_hbt_mode = QComboBox()
        self.combo_hbt_mode.addItem("默认 (0) - 需要 INS 就绪", 0)
        self.combo_hbt_mode.addItem("遥控模式 (3) - 跳过导航检查", 3)
        self.combo_hbt_mode.setCurrentIndex(1)  # Default to mode 3
        mode_layout.addWidget(self.combo_hbt_mode, 0, 1, 1, 2)

        mode_layout.addWidget(QLabel("说明: 解锁需要连续发送 >=10次 且 >=1秒"), 1, 0, 1, 3)

        layout.addWidget(mode_group)

        # Control buttons
        btn_group = QGroupBox("心跳控制")
        btn_group.setStyleSheet(GROUP_STYLE)
        btn_layout = QVBoxLayout(btn_group)

        btn_row = QHBoxLayout()
        self.btn_hbt_start = QPushButton("启动心跳 (20Hz)")
        self.btn_hbt_start.setStyleSheet(STYLE_SEND)
        self.btn_hbt_start.setFixedHeight(48)
        self.btn_hbt_start.clicked.connect(self._toggle_heartbeat)
        btn_row.addWidget(self.btn_hbt_start)

        self.btn_hbt_stop = QPushButton("停止心跳")
        self.btn_hbt_stop.setStyleSheet(STYLE_RED)
        self.btn_hbt_stop.setFixedHeight(48)
        self.btn_hbt_stop.setEnabled(False)
        self.btn_hbt_stop.clicked.connect(self._stop_heartbeat)
        btn_row.addWidget(self.btn_hbt_stop)

        btn_layout.addLayout(btn_row)

        # Status display
        status_row = QHBoxLayout()
        status_row.addWidget(QLabel("发送状态:"))
        self.lbl_hbt_status = QLabel("未启动")
        self.lbl_hbt_status.setStyleSheet("font-size: 14px; color: #7f8c8d;")
        status_row.addWidget(self.lbl_hbt_status)
        status_row.addStretch()
        btn_layout.addLayout(status_row)

        count_row = QHBoxLayout()
        count_row.addWidget(QLabel("已发送次数:"))
        self.lbl_hbt_count = QLabel("0")
        self.lbl_hbt_count.setStyleSheet("font-size: 14px; font-family: monospace;")
        count_row.addWidget(self.lbl_hbt_count)
        count_row.addStretch()

        count_row.addWidget(QLabel("解锁状态:"))
        self.lbl_hbt_armed = QLabel("未知")
        self.lbl_hbt_armed.setStyleSheet("font-size: 14px; font-weight: bold;")
        count_row.addWidget(self.lbl_hbt_armed)
        btn_layout.addLayout(count_row)

        layout.addWidget(btn_group)
        layout.addStretch()
        self.tabs.addTab(tab, "心跳控制")

    # ── Tab 3: Motion Control ─────────────────────────────────────────

    def _build_motion_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # Control Key builder
        ck_group = QGroupBox("控制模式 (自动计算 control_key)")
        ck_group.setStyleSheet(GROUP_STYLE)
        ck_layout = QGridLayout(ck_group)

        ck_layout.addWidget(QLabel("控制模式:"), 0, 0)
        self.combo_ctrl_mode = QComboBox()
        for name in CONTROL_MODES:
            self.combo_ctrl_mode.addItem(name)
        self.combo_ctrl_mode.currentIndexChanged.connect(self._update_control_key_display)
        ck_layout.addWidget(self.combo_ctrl_mode, 0, 1)

        ck_layout.addWidget(QLabel("坐标系:"), 1, 0)
        self.combo_coord_frame = QComboBox()
        for name in COORD_FRAMES:
            self.combo_coord_frame.addItem(name)
        self.combo_coord_frame.currentIndexChanged.connect(self._update_control_key_display)
        ck_layout.addWidget(self.combo_coord_frame, 1, 1)

        self.chk_incremental = QCheckBox("增量模式 (相对当前位姿)")
        self.chk_incremental.stateChanged.connect(self._update_control_key_display)
        ck_layout.addWidget(self.chk_incremental, 2, 0, 1, 2)

        ck_layout.addWidget(QLabel("control_key:"), 3, 0)
        self.lbl_control_key = QLabel("0x00 (0)")
        self.lbl_control_key.setStyleSheet("font-family: monospace; font-size: 14px; font-weight: bold; color: #2c3e50;")
        ck_layout.addWidget(self.lbl_control_key, 3, 1)

        layout.addWidget(ck_group)

        # Type Mask builder
        tm_group = QGroupBox("轴选择 (自动计算 type_mask)")
        tm_group.setStyleSheet(GROUP_STYLE)
        tm_layout = QHBoxLayout(tm_group)

        self.chk_axis = [QCheckBox(f"  {name}  ") for name in AXIS_NAMES]
        for chk in self.chk_axis:
            chk.setChecked(True)
            chk.stateChanged.connect(self._update_type_mask_display)
            tm_layout.addWidget(chk)

        tm_layout.addWidget(QLabel("  type_mask:"))
        self.lbl_type_mask = QLabel("0x0F (15)")
        self.lbl_type_mask.setStyleSheet("font-family: monospace; font-size: 14px; font-weight: bold; color: #2c3e50;")
        tm_layout.addWidget(self.lbl_type_mask)

        layout.addWidget(tm_group)

        # Target values
        val_group = QGroupBox("目标值")
        val_group.setStyleSheet(GROUP_STYLE)
        val_grid = QGridLayout(val_group)

        self.spin_x, self.spin_y, self.spin_z, self.spin_yaw = None, None, None, None

        val_grid.addWidget(QLabel("X (m):"), 0, 0)
        self.spin_x = QDoubleSpinBox()
        self.spin_x.setRange(-100.0, 100.0)
        self.spin_x.setSingleStep(0.1)
        self.spin_x.setDecimals(3)
        val_grid.addWidget(self.spin_x, 0, 1)

        val_grid.addWidget(QLabel("Y (m):"), 0, 2)
        self.spin_y = QDoubleSpinBox()
        self.spin_y.setRange(-100.0, 100.0)
        self.spin_y.setSingleStep(0.1)
        self.spin_y.setDecimals(3)
        val_grid.addWidget(self.spin_y, 0, 3)

        val_grid.addWidget(QLabel("Z (m):"), 1, 0)
        self.spin_z = QDoubleSpinBox()
        self.spin_z.setRange(-100.0, 100.0)
        self.spin_z.setSingleStep(0.1)
        self.spin_z.setDecimals(3)
        val_grid.addWidget(self.spin_z, 1, 1)

        val_grid.addWidget(QLabel("Yaw (°):"), 1, 2)
        self.spin_yaw = QDoubleSpinBox()
        self.spin_yaw.setRange(-360.0, 360.0)
        self.spin_yaw.setSingleStep(5.0)
        self.spin_yaw.setDecimals(1)
        val_grid.addWidget(self.spin_yaw, 1, 3)

        layout.addWidget(val_group)

        # Quick action buttons
        quick_group = QGroupBox("快捷操作")
        quick_group.setStyleSheet(GROUP_STYLE)
        quick_layout = QGridLayout(quick_group)

        quick_btns = [
            ("下潜 1m", 0, 0, 1),
            ("前进 1m", 0, 1, 1),
            ("右移 1m", 0, 2, 1),
            ("左转 90°", 0, 3, 1),
            ("右转 90°", 0, 4, 1),
            ("上浮 1m", 1, 0, 1),
            ("后退 1m", 1, 1, 1),
            ("左移 1m", 1, 2, 1),
            ("前进 3m + 下潜 1m", 1, 3, 2),
        ]
        for text, r, c, cs in quick_btns:
            btn = QPushButton(text)
            btn.setStyleSheet(STYLE_QUICK)
            btn.clicked.connect(lambda checked, t=text: self._quick_action(t))
            quick_layout.addWidget(btn, r, c, 1, cs)

        layout.addWidget(quick_group)

        # Send button
        send_row = QHBoxLayout()
        self.btn_send_setpoint = QPushButton("发送 Setpoint")
        self.btn_send_setpoint.setStyleSheet(STYLE_SEND)
        self.btn_send_setpoint.setFixedHeight(48)
        self.btn_send_setpoint.clicked.connect(self._send_setpoint)
        send_row.addWidget(self.btn_send_setpoint)

        self.btn_stop = QPushButton("紧急停止 (推力归零)")
        self.btn_stop.setStyleSheet(STYLE_RED)
        self.btn_stop.setFixedHeight(48)
        self.btn_stop.clicked.connect(self._emergency_stop)
        send_row.addWidget(self.btn_stop)

        layout.addLayout(send_row)

        # Current target display
        self.lbl_current_target = QLabel("当前目标: ---")
        self.lbl_current_target.setStyleSheet("font-family: monospace; font-size: 12px; color: #7f8c8d;")
        layout.addWidget(self.lbl_current_target)

        layout.addStretch()

        # Initialize display
        self._update_control_key_display()
        self._update_type_mask_display()

        self.tabs.addTab(tab, "运动控制")

    # ── Tab 4: PID Tuning ─────────────────────────────────────────────

    def _build_pid_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # Axis & loop selection
        sel_group = QGroupBox("参数选择")
        sel_group.setStyleSheet(GROUP_STYLE)
        sel_layout = QHBoxLayout(sel_group)

        sel_layout.addWidget(QLabel("轴:"))
        self.combo_pid_axis = QComboBox()
        for i, name in enumerate(AXIS_NAMES):
            self.combo_pid_axis.addItem(name, i)
        self.combo_pid_axis.currentIndexChanged.connect(self._on_pid_axis_changed)
        sel_layout.addWidget(self.combo_pid_axis)

        sel_layout.addWidget(QLabel("环路:"))
        self.combo_pid_loop = QComboBox()
        self.combo_pid_loop.addItem("位置环 (Position)", True)
        self.combo_pid_loop.addItem("速度环 (Velocity)", False)
        self.combo_pid_loop.currentIndexChanged.connect(self._on_pid_loop_changed)
        sel_layout.addWidget(self.combo_pid_loop)

        self.btn_read_pid = QPushButton("读取当前参数")
        self.btn_read_pid.setStyleSheet(STYLE_SEND)
        self.btn_read_pid.clicked.connect(self._read_current_pid)
        sel_layout.addWidget(self.btn_read_pid)

        layout.addWidget(sel_group)

        # Parameters grid
        param_group = QGroupBox("PID 参数 (负值 = 不更新)")
        param_group.setStyleSheet(GROUP_STYLE)
        self.pid_grid = QGridLayout(param_group)

        self.pid_spins = {}
        pid_params = [
            ("kp", 0.0, 100.0, 0.001),
            ("ki", 0.0, 100.0, 0.001),
            ("kd", 0.0, 100.0, 0.001),
            ("i_limit", 0.0, 10000.0, 0.1),
            ("out_limit", 0.0, 10000.0, 0.1),
            ("max_v", 0.0, 100.0, 0.1),
            ("max_a", 0.0, 100.0, 0.1),
        ]

        for i, (name, min_v, max_v, step) in enumerate(pid_params):
            self.pid_grid.addWidget(QLabel(f"{name}:"), i, 0)
            spin = QDoubleSpinBox()
            spin.setRange(-1.0, max_v)  # -1 means "don't update"
            spin.setSingleStep(step)
            spin.setDecimals(4)
            spin.setMinimumWidth(140)
            self.pid_grid.addWidget(spin, i, 1)

            chk = QCheckBox("不更新")
            chk.setChecked(False)
            chk.stateChanged.connect(lambda state, s=spin, n=name: self._on_pid_skip_changed(state, s))
            self.pid_grid.addWidget(chk, i, 2)

            current_lbl = QLabel("---")
            current_lbl.setStyleSheet("color: #7f8c8d; font-family: monospace;")
            self.pid_grid.addWidget(current_lbl, i, 3)

            self.pid_spins[name] = (spin, chk, current_lbl)

        layout.addWidget(param_group)

        # Send button
        send_row = QHBoxLayout()
        self.btn_send_pid = QPushButton("发送 PID 参数")
        self.btn_send_pid.setStyleSheet(STYLE_SEND)
        self.btn_send_pid.setFixedHeight(48)
        self.btn_send_pid.clicked.connect(self._send_pid_params)
        send_row.addWidget(self.btn_send_pid)

        self.btn_send_all_pid = QPushButton("发送所有轴参数")
        self.btn_send_all_pid.setStyleSheet(STYLE_QUICK)
        self.btn_send_all_pid.setFixedHeight(48)
        self.btn_send_all_pid.clicked.connect(self._send_all_pid_params)
        send_row.addWidget(self.btn_send_all_pid)

        layout.addLayout(send_row)

        self.lbl_pid_status = QLabel("")
        self.lbl_pid_status.setStyleSheet("font-size: 12px; color: #7f8c8d;")
        layout.addWidget(self.lbl_pid_status)

        layout.addStretch()

        # Init visibility
        self._on_pid_loop_changed(0)

        self.tabs.addTab(tab, "PID 调参")

    # ── Tab 5: Peripherals ────────────────────────────────────────────

    def _build_peripherals_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # Servo
        servo_group = QGroupBox("舵机控制")
        servo_group.setStyleSheet(GROUP_STYLE)
        servo_layout = QGridLayout(servo_group)

        servo_layout.addWidget(QLabel("角度 (°):"), 0, 0)
        self.slider_servo = QSlider(Qt.Horizontal)
        self.slider_servo.setRange(-180, 180)
        self.slider_servo.setValue(0)
        self.slider_servo.setTickPosition(QSlider.TicksBelow)
        self.slider_servo.setTickInterval(30)
        self.slider_servo.valueChanged.connect(self._on_servo_slider_changed)
        servo_layout.addWidget(self.slider_servo, 0, 1)

        self.spin_servo = QDoubleSpinBox()
        self.spin_servo.setRange(-180.0, 180.0)
        self.spin_servo.setSingleStep(1.0)
        self.spin_servo.setDecimals(1)
        self.spin_servo.setValue(0.0)
        self.spin_servo.valueChanged.connect(self._on_servo_spin_changed)
        servo_layout.addWidget(self.spin_servo, 0, 2)

        self.lbl_servo_val = QLabel("0.0°")
        self.lbl_servo_val.setStyleSheet("font-family: monospace; font-size: 14px; font-weight: bold;")
        servo_layout.addWidget(self.lbl_servo_val, 0, 3)

        self.btn_send_servo = QPushButton("发送舵机角度")
        self.btn_send_servo.setStyleSheet(STYLE_SEND)
        self.btn_send_servo.clicked.connect(self._send_servo)
        servo_layout.addWidget(self.btn_send_servo, 1, 0, 1, 4)

        layout.addWidget(servo_group)

        # Light
        light_group = QGroupBox("灯光控制")
        light_group.setStyleSheet(GROUP_STYLE)
        light_layout = QGridLayout(light_group)

        self.btn_light_toggle = QPushButton("灯光: 关")
        self.btn_light_toggle.setStyleSheet(STYLE_RED)
        self.btn_light_toggle.setCheckable(True)
        self.btn_light_toggle.setFixedHeight(48)
        self.btn_light_toggle.clicked.connect(self._toggle_light)
        light_layout.addWidget(self.btn_light_toggle, 0, 0, 1, 2)

        light_layout.addWidget(QLabel("亮度:"), 1, 0)
        self.slider_light = QSlider(Qt.Horizontal)
        self.slider_light.setRange(0, 255)
        self.slider_light.setValue(255)
        self.slider_light.setTickPosition(QSlider.TicksBelow)
        self.slider_light.setTickInterval(32)
        light_layout.addWidget(self.slider_light, 1, 1)

        self.lbl_light_val = QLabel("255")
        self.lbl_light_val.setStyleSheet("font-family: monospace; font-size: 14px;")
        light_layout.addWidget(self.lbl_light_val, 1, 2)

        self.slider_light.valueChanged.connect(lambda v: self.lbl_light_val.setText(str(v)))

        self.btn_send_light = QPushButton("发送灯光")
        self.btn_send_light.setStyleSheet(STYLE_SEND)
        self.btn_send_light.clicked.connect(self._send_light)
        light_layout.addWidget(self.btn_send_light, 2, 0, 1, 3)

        layout.addWidget(light_group)

        # INS
        ins_group = QGroupBox("INS/DVL 控制")
        ins_group.setStyleSheet(GROUP_STYLE)
        ins_layout = QHBoxLayout(ins_group)

        ins_layout.addWidget(QLabel("命令:"))
        self.combo_ins = QComboBox()
        self.combo_ins.addItem("启动 INS (1)", 1)
        self.combo_ins.addItem("启动 DVL (2)", 2)
        self.combo_ins.addItem("INS+DVL (3)", 3)
        self.combo_ins.addItem("停止 (0)", 0)
        ins_layout.addWidget(self.combo_ins)

        self.btn_send_ins = QPushButton("发送")
        self.btn_send_ins.setStyleSheet(STYLE_SEND)
        self.btn_send_ins.clicked.connect(self._send_ins)
        ins_layout.addWidget(self.btn_send_ins)

        layout.addWidget(ins_group)

        layout.addStretch()
        self.tabs.addTab(tab, "外设控制")

    # ══════════════════════════════════════════════════════════════════
    #  Signal/Slot Implementations
    # ══════════════════════════════════════════════════════════════════

    # ── Control Key / Type Mask ────────────────────────────────────────

    def _update_control_key_display(self):
        mode = CONTROL_MODES[self.combo_ctrl_mode.currentText()]
        frame = COORD_FRAMES[self.combo_coord_frame.currentText()]
        incr = 0x20 if self.chk_incremental.isChecked() else 0x00
        ck = mode | (frame << 4) | incr
        self.lbl_control_key.setText(f"0x{ck:02X} ({ck})")

    def _update_type_mask_display(self):
        mask = 0
        for i, chk in enumerate(self.chk_axis):
            if chk.isChecked():
                mask |= (1 << i)
        self.lbl_type_mask.setText(f"0x{mask:02X} ({mask})")

    def _get_control_key(self):
        mode = CONTROL_MODES[self.combo_ctrl_mode.currentText()]
        frame = COORD_FRAMES[self.combo_coord_frame.currentText()]
        incr = 0x20 if self.chk_incremental.isChecked() else 0x00
        return mode | (frame << 4) | incr

    def _get_type_mask(self):
        mask = 0
        for i, chk in enumerate(self.chk_axis):
            if chk.isChecked():
                mask |= (1 << i)
        return mask

    # ── Quick Actions ─────────────────────────────────────────────────

    def _quick_action(self, text):
        # Set incremental position mode
        self.combo_ctrl_mode.setCurrentIndex(0)  # Position
        self.combo_coord_frame.setCurrentIndex(1)  # Body
        self.chk_incremental.setChecked(True)
        for chk in self.chk_axis:
            chk.setChecked(False)

        if text == "下潜 1m":
            self.spin_z.setValue(1.0)
            self.chk_axis[2].setChecked(True)  # Z
        elif text == "上浮 1m":
            self.spin_z.setValue(-1.0)
            self.chk_axis[2].setChecked(True)
        elif text == "前进 1m":
            self.spin_x.setValue(1.0)
            self.chk_axis[0].setChecked(True)  # X
        elif text == "后退 1m":
            self.spin_x.setValue(-1.0)
            self.chk_axis[0].setChecked(True)
        elif text == "右移 1m":
            self.spin_y.setValue(1.0)
            self.chk_axis[1].setChecked(True)  # Y
        elif text == "左移 1m":
            self.spin_y.setValue(-1.0)
            self.chk_axis[1].setChecked(True)
        elif text == "左转 90°":
            self.spin_yaw.setValue(-90.0)
            self.chk_axis[3].setChecked(True)  # Yaw
        elif text == "右转 90°":
            self.spin_yaw.setValue(90.0)
            self.chk_axis[3].setChecked(True)
        elif text == "前进 3m + 下潜 1m":
            self.spin_x.setValue(3.0)
            self.spin_z.setValue(1.0)
            self.chk_axis[0].setChecked(True)
            self.chk_axis[2].setChecked(True)

        self._send_setpoint()

    # ── Send Setpoint ─────────────────────────────────────────────────

    def _send_setpoint(self):
        msg = ZitSetpoint()
        msg.control_key = self._get_control_key()
        msg.type_mask = self._get_type_mask()
        msg.x = float(self.spin_x.value())
        msg.y = float(self.spin_y.value())
        msg.z = float(self.spin_z.value())
        msg.yaw = math.radians(float(self.spin_yaw.value()))

        self.ros_node.setpoint_pub.publish(msg)

        ck = msg.control_key
        tm = msg.type_mask
        self.lbl_current_target.setText(
            f"已发送: ck=0x{ck:02X} tm=0x{tm:02X} | "
            f"X={msg.x:.3f} Y={msg.y:.3f} Z={msg.z:.3f} Yaw={self.spin_yaw.value():.1f}°"
        )

    def _emergency_stop(self):
        msg = ZitSetpoint()
        msg.control_key = 0x02 | 0x10  # force + body
        msg.type_mask = 0x0F
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        msg.yaw = 0.0
        self.ros_node.setpoint_pub.publish(msg)
        self.lbl_current_target.setText("紧急停止: 所有推力归零")

    # ── Heartbeat ─────────────────────────────────────────────────────

    def _toggle_heartbeat(self):
        if self.hbt_thread and self.hbt_thread.is_running:
            return  # already running

        mode = self.combo_hbt_mode.currentData()
        self.hbt_thread = HeartbeatThread(self.ros_node)
        self.hbt_thread.set_mode(mode)
        self.hbt_thread.status_update.connect(self._on_hbt_status)
        self.hbt_thread.start()

        self.btn_hbt_start.setEnabled(False)
        self.btn_hbt_stop.setEnabled(True)
        self.btn_hbt_start.setStyleSheet(STYLE_YELLOW)

    def _stop_heartbeat(self):
        if self.hbt_thread:
            self.hbt_thread.stop()
        self.btn_hbt_start.setEnabled(True)
        self.btn_hbt_stop.setEnabled(False)
        self.btn_hbt_start.setStyleSheet(STYLE_SEND)

    def _on_hbt_status(self, text, count):
        self.lbl_hbt_status.setText(text)
        self.lbl_hbt_count.setText(str(count))

    # ── PID Tuning ────────────────────────────────────────────────────

    def _on_pid_axis_changed(self, index):
        self._update_pid_param_names()

    def _on_pid_loop_changed(self, index):
        is_pos = self.combo_pid_loop.currentData()
        # Show/hide max_v and max_a for position loop
        for name in ["max_v", "max_a"]:
            if name in self.pid_spins:
                spin, chk, lbl = self.pid_spins[name]
                visible = is_pos
                spin.setVisible(visible)
                chk.setVisible(visible)
                lbl.setVisible(visible)
                # Find the label in the same row
                for i in range(self.pid_grid.count()):
                    item = self.pid_grid.itemAt(i)
                    if item and item.widget() and isinstance(item.widget(), QLabel):
                        if item.widget().text() == f"{name}:":
                            item.widget().setVisible(visible)

        # ki and kd not used in position loop
        for name in ["ki", "kd"]:
            if name in self.pid_spins:
                spin, chk, lbl = self.pid_spins[name]
                chk.setEnabled(not is_pos)
                if is_pos:
                    chk.setChecked(True)  # Don't update ki/kd for position loop

        self._update_pid_param_names()

    def _update_pid_param_names(self):
        axis = self.combo_pid_axis.currentText()
        loop = "位置环" if self.combo_pid_loop.currentData() else "速度环"
        self.lbl_pid_status.setText(f"当前编辑: {axis}轴 {loop}")

    def _on_pid_skip_changed(self, state, spin):
        spin.setEnabled(not state)
        if state:
            spin.setValue(-1.0)

    def _read_current_pid(self):
        """Read current PID params from /zit6/state/pid_status cache."""
        if self.zit6_pid_status is None:
            self.lbl_pid_status.setText("未收到 PID 状态数据，请确保固件/仿真桥运行中")
            return

        axis = self.combo_pid_axis.currentIndex()
        is_pos = self.combo_pid_loop.currentData()
        ps = self.zit6_pid_status

        if is_pos:
            vals = {
                "kp": ps.pos_kp[axis],
                "i_limit": 0.0,  # not in pos pid status
                "out_limit": ps.pos_out_limit[axis],
                "max_v": ps.pos_max_v[axis],
                "max_a": ps.pos_max_a[axis],
            }
        else:
            vals = {
                "kp": ps.vel_kp[axis],
                "ki": ps.vel_ki[axis],
                "kd": ps.vel_kd[axis],
                "i_limit": ps.vel_i_limit[axis],
                "out_limit": ps.vel_out_limit[axis],
            }

        for name, (spin, chk, lbl) in self.pid_spins.items():
            if name in vals:
                lbl.setText(f"当前: {vals[name]:.4f}")
            else:
                lbl.setText("---")

        self.lbl_pid_status.setText(f"已读取 {AXIS_NAMES[axis]}轴 {'位置环' if is_pos else '速度环'} 参数")

    def _send_pid_params(self):
        axis = self.combo_pid_axis.currentIndex()
        is_pos = self.combo_pid_loop.currentData()

        msg = ZitPid()
        msg.axis = axis
        msg.is_pos_ring = is_pos

        # Default: don't update (-1)
        msg.kp = -1.0
        msg.ki = -1.0
        msg.kd = -1.0
        msg.i_limit = -1.0
        msg.out_limit = -1.0
        msg.max_v = -1.0
        msg.max_a = -1.0

        for name, (spin, chk, lbl) in self.pid_spins.items():
            if not chk.isChecked():
                val = float(spin.value())
                if val >= 0:
                    setattr(msg, name, val)

        self.ros_node.pid_pub.publish(msg)
        self.lbl_pid_status.setText(f"已发送 {AXIS_NAMES[axis]}轴 {'位置环' if is_pos else '速度环'} PID 参数")

    def _send_all_pid_params(self):
        for axis in range(4):
            for is_pos in [True, False]:
                msg = ZitPid()
                msg.axis = axis
                msg.is_pos_ring = is_pos
                msg.kp = -1.0
                msg.ki = -1.0
                msg.kd = -1.0
                msg.i_limit = -1.0
                msg.out_limit = -1.0
                msg.max_v = -1.0
                msg.max_a = -1.0

                # Only send values that match current UI selection
                if axis == self.combo_pid_axis.currentIndex() and is_pos == self.combo_pid_loop.currentData():
                    for name, (spin, chk, lbl) in self.pid_spins.items():
                        if not chk.isChecked():
                            val = float(spin.value())
                            if val >= 0:
                                setattr(msg, name, val)

                self.ros_node.pid_pub.publish(msg)
                time.sleep(0.01)

        self.lbl_pid_status.setText("已发送所有轴 PID 参数 (仅当前选择的参数已更新，其余保持不变)")

    # ── Peripherals ───────────────────────────────────────────────────

    def _on_servo_slider_changed(self, val):
        self.spin_servo.blockSignals(True)
        self.spin_servo.setValue(float(val))
        self.spin_servo.blockSignals(False)
        self.lbl_servo_val.setText(f"{val}°")

    def _on_servo_spin_changed(self, val):
        self.slider_servo.blockSignals(True)
        self.slider_servo.setValue(int(val))
        self.slider_servo.blockSignals(False)
        self.lbl_servo_val.setText(f"{val:.1f}°")

    def _send_servo(self):
        msg = Float32()
        msg.data = float(self.spin_servo.value())
        self.ros_node.servo_pub.publish(msg)

    def _toggle_light(self, checked):
        if checked:
            self.btn_light_toggle.setText("灯光: 开")
            self.btn_light_toggle.setStyleSheet(STYLE_GREEN)
        else:
            self.btn_light_toggle.setText("灯光: 关")
            self.btn_light_toggle.setStyleSheet(STYLE_RED)

    def _send_light(self):
        msg = UInt8()
        if self.btn_light_toggle.isChecked():
            msg.data = self.slider_light.value()
        else:
            msg.data = 0
        self.ros_node.light_pub.publish(msg)

    def _send_ins(self):
        msg = UInt8()
        msg.data = self.combo_ins.currentData()
        self.ros_node.ins_pub.publish(msg)

    # ══════════════════════════════════════════════════════════════════
    #  Existing Functionality (preserved)
    # ══════════════════════════════════════════════════════════════════

    def setup_nodes_list(self):
        self.lisr_node.clear()

        nodes_to_manage = [
            {"name": "HW Bridge (仿真桥接)", "cmd": "ros2 run uv_hm uv_sim_bridge"},
            {"name": "YOLO 检测", "cmd": "ros2 run uv_ai uv_detect_demo --ros-args -p use_sim_time:=true"},
            {"name": "Automaton (状态机)", "cmd": "ros2 run uv_ai uv_automation"},
            {"name": "3D 定位解析", "cmd": "ros2 run uv_ai uv_position"},
            {"name": "PC/Depth Logger", "cmd": "ros2 run uv_ai pc_recorder"},
        ]

        for n_info in nodes_to_manage:
            item = QListWidgetItem(self.lisr_node)
            widget = NodeControlWidget(n_info["name"], n_info["cmd"], self.repo_root)
            item.setSizeHint(widget.sizeHint())
            self.lisr_node.setItemWidget(item, widget)

    def clear_path(self):
        self.path_data = []
        self.path_line.setData(
            pos=np.array([[0.0, 0.0, 0.0]]),
            color=np.array([[0.0, 0.4, 1.0, 1.0]]),
        )

    def load_config(self):
        try:
            if os.path.exists(self.config_path):
                with open(self.config_path, "r", encoding="utf-8") as f:
                    self.config = json.load(f)
            else:
                self.config = {
                    "simulation": {"current_scenario": "test", "enable_ai": False, "enable_depth": False},
                    "runtime": {"repo_root": "/home/origin/AUV2026", "nodes_to_start": []},
                }
        except Exception:
            self.config = {
                "simulation": {"current_scenario": "test", "enable_ai": False, "enable_depth": False},
                "runtime": {"repo_root": "/home/origin/AUV2026"},
            }

    def save_config(self):
        try:
            with open(self.config_path, "w", encoding="utf-8") as f:
                json.dump(self.config, f, indent=4, ensure_ascii=False)
        except Exception as e:
            self.ros_node.get_logger().error(f"保存配置失败: {e}")

    def update_status_labels(self):
        sim_cfg = self.config.get("simulation", {})
        current_scen = sim_cfg.get("current_scenario", "test")
        self.statement.setText(f"目前待命场景: {current_scen} | 就绪")

    def on_open_sim_settings(self):
        dialog = SimSettingsDialog(self, self.config)
        if dialog.exec() == QDialog.Accepted:
            st = dialog.get_settings()
            if "simulation" not in self.config:
                self.config["simulation"] = {}
            for k, v in st.items():
                self.config["simulation"][k] = v
            self.save_config()
            self.update_status_labels()

    def on_sim_launch_clicked(self):
        if self.sim_process and self.sim_process.poll() is None:
            QMessageBox.warning(self, "运行中", "仿真主程序已在运行！")
            return

        scenario = self.config.get("simulation", {}).get("current_scenario", "test")
        enable_ai = self.config.get("simulation", {}).get("enable_ai", False)

        self.label_motion.setText(f"正在启动: {scenario}...")
        self.statement.setText("状态: 仿真(Stonefish) 运行中!")

        enable_depth = self.config.get("simulation", {}).get("enable_depth", False)
        ai_arg = "enable_ai:=true" if enable_ai else "enable_ai:=false"
        depth_flag = "--enable-depth" if enable_depth else ""
        cmd = f"source {self.repo_root}/scripts/env.sh && uuv_source && uuv_sim {scenario} {ai_arg} {depth_flag}"

        try:
            self.sim_process = subprocess.Popen(["bash", "-c", cmd], preexec_fn=os.setsid)
        except Exception as e:
            QMessageBox.critical(self, "错误", f"全量启动失败: {e}")

    def on_stop_sim_clicked(self):
        if self.sim_process and self.sim_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.sim_process.pid), 9)
            except Exception:
                self.sim_process.kill()
            self.sim_process = None
            self.statement.setText("状态: 仿真主进程已关闭")

    def closeEvent(self, event):
        self.on_stop_sim_clicked()
        # Stop heartbeat
        if self.hbt_thread and self.hbt_thread.is_running:
            self.hbt_thread.stop()
        # Clean up child nodes
        for i in range(self.lisr_node.count()):
            widget = self.lisr_node.itemWidget(self.lisr_node.item(i))
            if widget and widget.process and widget.process.poll() is None:
                try:
                    os.killpg(os.getpgid(widget.process.pid), 9)
                except Exception:
                    pass
        event.accept()

    # ── UI Update Timer ───────────────────────────────────────────────

    def update_ui_state(self):
        # 3D marker
        x, y, z, yaw = self.current_pos
        self.position_marker.setData(pos=np.array([[x, y, -z]]))

        if len(self.path_data) == 0 or (np.linalg.norm(np.array([x, y, -z]) - np.array(self.path_data[-1])) > 0.05):
            self.path_data.append([x, y, -z])
            self.path_line.setData(pos=np.array(self.path_data))

        self.label_motion.setText(f"Position (NED): X={x:.2f} Y={y:.2f} Z={z:.2f} Yaw={yaw:.1f}°")

        # Camera feeds
        if self.latest_front_img is not None:
            img = self.latest_front_img
            h, w, ch = img.shape
            qImg = QImage(img.data, w, h, ch * w, QImage.Format_BGR888)
            self.label_front_cam.setPixmap(QPixmap.fromImage(qImg).scaled(self.label_front_cam.size(), Qt.KeepAspectRatio))
            self.latest_front_img = None

        if self.latest_down_img is not None:
            img = self.latest_down_img
            h, w, ch = img.shape
            qImg = QImage(img.data, w, h, ch * w, QImage.Format_BGR888)
            self.label_down_cam.setPixmap(QPixmap.fromImage(qImg).scaled(self.label_down_cam.size(), Qt.KeepAspectRatio))
            self.latest_down_img = None

        # Update ZIT6 status display
        self._update_status_display()

    def _update_status_display(self):
        """Update Tab 1 (System Status) and Tab 2 (Heartbeat) with cached ZIT6 data."""
        s = self.zit6_status
        if s is None:
            return

        # Armed
        if s.is_armed:
            self.lbl_armed.setText("已解锁 (ARMED)")
            self.lbl_armed.setStyleSheet(STYLE_GREEN)
            self.lbl_hbt_armed.setText("已解锁")
            self.lbl_hbt_armed.setStyleSheet("color: #2ecc71; font-weight: bold;")
        else:
            self.lbl_armed.setText("已锁定 (LOCKED)")
            self.lbl_armed.setStyleSheet(STYLE_RED)
            self.lbl_hbt_armed.setText("已锁定")
            self.lbl_hbt_armed.setStyleSheet("color: #e74c3c; font-weight: bold;")

        # Control level
        level = s.control_level
        level_names = {0: "无 (None)", 1: "位置环 (POS)", 2: "速度环 (VEL)", 3: "推力环 (FORCE)"}
        self.lbl_control_level.setText(level_names.get(level, f"未知({level})"))

        # INS state
        self.lbl_ins_state.setText(INS_STATES.get(s.ins_state, f"未知({s.ins_state})"))

        # Navigation ready
        self.lbl_nav_ready.setText("是" if s.navigation_ready else "否")
        self.lbl_nav_ready.setStyleSheet(
            "font-family: monospace; font-size: 13px; color: #2ecc71;" if s.navigation_ready
            else "font-family: monospace; font-size: 13px; color: #e74c3c;"
        )

        # Battery
        self.lbl_battery.setText(f"{s.battery_voltage:.1f} V")

        # Errors
        if s.error_flags == 0:
            self.lbl_errors.setText("无错误")
            self.lbl_errors.setStyleSheet("font-family: monospace; font-size: 13px; color: #2ecc71;")
        else:
            errs = []
            for bit, name in ERROR_FLAGS.items():
                if s.error_flags & bit:
                    errs.append(name)
            self.lbl_errors.setText(" | ".join(errs))
            self.lbl_errors.setStyleSheet("font-family: monospace; font-size: 13px; color: #e74c3c;")

        # Cycle time
        self.lbl_cycle_time.setText(f"{s.cycle_time_ms:.1f} ms")

        # Forces
        for i in range(4):
            self.lbl_forces[i].setText(f"{s.forces[i]:.2f}")

        # Position & Velocity
        self.lbl_pos_x.setText(f"{self.current_pos[0]:.3f}")
        self.lbl_pos_y.setText(f"{self.current_pos[1]:.3f}")
        self.lbl_pos_z.setText(f"{self.current_pos[2]:.3f}")
        self.lbl_pos_yaw.setText(f"{self.current_pos[3]:.1f}°")

        self.lbl_vel_x.setText(f"{self.current_vel[0]:.3f}")
        self.lbl_vel_y.setText(f"{self.current_vel[1]:.3f}")
        self.lbl_vel_z.setText(f"{self.current_vel[2]:.3f}")
        self.lbl_vel_yaw.setText(f"{self.current_vel[3]:.1f}°/s")


# ══════════════════════════════════════════════════════════════════════════
#  ROS2 Node
# ══════════════════════════════════════════════════════════════════════════

class AUVGuiNode(Node):
    def __init__(self, main_window):
        super().__init__('auv_gui_node')
        self.main_window = main_window
        self.bridge = CvBridge()

        # ── Publishers ─────────────────────────────────────────────────
        self.setpoint_pub = self.create_publisher(ZitSetpoint, '/zit6/cmd/setpoint', 10)
        self.hbt_pub = self.create_publisher(UInt32, '/zit6/cmd/agxhbt', 10)
        self.pid_pub = self.create_publisher(ZitPid, '/zit6/cmd/pid', 10)
        self.servo_pub = self.create_publisher(Float32, '/zit6/cmd/servo', 10)
        self.light_pub = self.create_publisher(UInt8, '/zit6/cmd/light', 10)
        self.ins_pub = self.create_publisher(UInt8, '/zit6/cmd/ins', 10)

        # ── Subscribers ────────────────────────────────────────────────
        # ZIT6 state
        self.create_subscription(ZitStatus, '/zit6/state/status', self._status_cb, 10)
        self.create_subscription(Float32MultiArray, '/zit6/state/pos', self._pos_cb, 10)
        self.create_subscription(Float32MultiArray, '/zit6/state/vel', self._vel_cb, 10)
        self.create_subscription(Float32MultiArray, '/zit6/state/thr', self._thr_cb, 10)
        self.create_subscription(UInt32, '/zit6/state/zithbt', self._fw_hbt_cb, 10)
        self.create_subscription(ZitPidStatus, '/zit6/state/pid_status', self._pid_status_cb, 10)

        # Legacy motion (for 3D trajectory)
        self.create_subscription(RobotMotionController, '/uv_hm/sim_bridge/motion', self._motion_cb, 10)

        # Camera images
        self.create_subscription(Image, "detectedimg_front", self._front_img_cb, 10)
        self.create_subscription(Image, "detectedimg_down", self._down_img_cb, 10)
        self.create_subscription(Image, "/uv_vision/front/image_raw", self._front_img_fallback_cb, 10)

    # ── ZIT6 state callbacks ──────────────────────────────────────────

    def _status_cb(self, msg):
        self.main_window.zit6_status = msg

    def _pos_cb(self, msg):
        if len(msg.data) >= 4:
            self.main_window.current_pos = [
                msg.data[0], msg.data[1], msg.data[2],
                math.degrees(msg.data[3]),  # yaw rad -> deg
            ]

    def _vel_cb(self, msg):
        if len(msg.data) >= 4:
            self.main_window.current_vel = [
                msg.data[0], msg.data[1], msg.data[2],
                math.degrees(msg.data[3]),
            ]

    def _thr_cb(self, msg):
        if len(msg.data) >= 4:
            self.main_window.current_forces = list(msg.data[:4])

    def _fw_hbt_cb(self, msg):
        if self.main_window.zit6_status is not None:
            # Store firmware heartbeat timestamp
            pass  # Displayed via zit6_status

    def _pid_status_cb(self, msg):
        self.main_window.zit6_pid_status = msg

    # ── Legacy motion callback ────────────────────────────────────────

    def _motion_cb(self, msg):
        # Only use if ZIT6 pos not available
        if self.main_window.zit6_status is None:
            self.main_window.current_pos = [msg.pos.x, msg.pos.y, msg.pos.z, 0.0]

    # ── Camera callbacks ──────────────────────────────────────────────

    def _front_img_cb(self, msg):
        try:
            self.main_window.latest_front_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def _front_img_fallback_cb(self, msg):
        if self.main_window.latest_front_img is None:
            try:
                self.main_window.latest_front_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception:
                pass

    def _down_img_cb(self, msg):
        try:
            self.main_window.latest_down_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass


# ══════════════════════════════════════════════════════════════════════════
#  Main Entry Point
# ══════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    app = QApplication(sys.argv)

    # Create a temporary node for initial setup
    gui_node = Node('temp_gui_launcher')
    main_window = AUVControlWindow(gui_node)

    # Create the real ROS communication node
    comm_node = AUVGuiNode(main_window)
    main_window.ros_node = comm_node

    # Spin ROS in background thread
    def ros_spin():
        try:
            rclpy.spin(comm_node)
        except Exception:
            pass

    spin_thread = threading.Thread(target=ros_spin, daemon=True)
    spin_thread.start()

    main_window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
