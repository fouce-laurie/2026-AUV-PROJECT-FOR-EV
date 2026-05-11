# AUV2026 ROS2 节点总览与接口文档

本文档基于当前仓库源码整理，目标是快速回答以下问题：

- 这个节点做什么
- 这个节点发布/订阅哪些消息
- 这个节点提供/调用哪些服务

## 1. 系统架构概述

AUV2026 采用 ZIT6 通信协议（基于 micro-ROS），实现上位机与固件之间的标准化通信。全系统统一使用 **NED (北东地)** 坐标系。

### 核心通信架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                        上位机 (AGX/Jetson)                          │
├─────────────────────────────────────────────────────────────────────┤
│  uv_automaton ──→ /zit6/cmd/setpoint ──→ uv_sim_bridge ──→ Stonefish │
│       │                    │                    │                    │
│       │              /zit6/cmd/agxhbt           │                    │
│       │                    │                    │                    │
│       └──← /zit6/state/status ←────────────────┘                    │
│              /zit6/state/pos                                        │
│              /zit6/state/vel                                        │
└─────────────────────────────────────────────────────────────────────┘
```

## 2. 节点总览

当前工程可见的主要 ROS2 节点如下：

1. `stonefish_simulator`（C++）（仿真)
2. `stonefish_simulator_nogpu`（C++）（仿真)
3. `uv_sim_bridge`（Python）- 仿真桥接，ZIT6 协议适配
4. `uv_detect_demo`（Python）- YOLO 目标检测
5. `uv_segment`（Python）- 语义分割
6. `uv_automaton` / `uv_automaton_debug`（Python）- 状态机任务调度核心
7. `uv_position`（Python）- 视觉定位与 3D 坐标估计
8. `uv_core`（`uv_hmu` 可执行，Python）- 实车硬件管理
9. `uv_capture`（Python）- 相机图像采集
10. `uv_depthimg`（Python）- 实车双目深度计算
11. `uv_depthimg_sim`（Python）- 仿真环境深度图与点云发布
12. `pc_recorder`（Python）- 点云记录/可视化工具

## 3. 节点详细说明

### 3.1 `stonefish_simulator`

- 所属包: `stonefish_ros2`
- 作用: 启动带图形界面的 Stonefish 仿真器，加载场景并创建场景内传感器/执行器对应的 ROS2 接口。

发布（当前 `Data/xunyun.scn` 场景下）：

- `/auv/odometry` (`nav_msgs/msg/Odometry`) - 30Hz
- `/auv/pressure` (`sensor_msgs/msg/FluidPressure`) - 5Hz
- `/auv/dvl` (`stonefish_ros2/msg/DVL`) - 20Hz
- `/auv/dvl_altitude` (`sensor_msgs/msg/Range`)
- `/auv/imu` (`sensor_msgs/msg/Imu`) - 20Hz
- `/sim/front_cam/left/image_color` (`sensor_msgs/msg/Image`) - 10Hz
- `/sim/front_cam/right/image_color` (`sensor_msgs/msg/Image`) - 10Hz
- `/sim/down_cam/left/image_color` (`sensor_msgs/msg/Image`) - 30Hz
- `/sim/down_cam/right/image_color` (`sensor_msgs/msg/Image`) - 30Hz
- `/auv/thrusters_state` (`stonefish_ros2/msg/ThrusterState`)

订阅：

- `/auv/thrusters_cmd` (`std_msgs/msg/Float64MultiArray`)

服务：

- `enable_currents` (`std_srvs/srv/Trigger`) - 开启水流
- `disable_currents` (`std_srvs/srv/Trigger`) - 关闭水流
- `respawn_robot` (`stonefish_ros2/srv/Respawn`) - 重生机器人

### 3.2 `uv_sim_bridge`

- 所属包: `uv_hm`
- 作用: 仿真桥接节点。将 Stonefish 传感器数据转换为 ZIT6 协议话题，同时接收 ZIT6 控制指令转换为推进器命令。内置 4-DOF PID 位置控制器。

**发布（ZIT6 协议话题）：**

| 话题 | 消息类型 | 频率 | 说明 |
|------|----------|------|------|
| `/zit6/state/status` | `ZitStatus` | 10Hz | 核心状态（解锁、错误、推力等） |
| `/zit6/state/pos` | `Float32MultiArray` | 30Hz | 位置 [x_ned, y_ned, z, yaw_ned_rad] |
| `/zit6/state/vel` | `Float32MultiArray` | 50Hz | 速度 [vx_ned, vy_ned, vz, vyaw_ned_rad_s] |
| `/zit6/state/thr` | `Float32MultiArray` | 30Hz | 推力 [Fx, Fy, Fz, Mz] |
| `/zit6/state/zithbt` | `UInt32` | 1Hz | 固件心跳（毫秒时间戳） |
| `/zit6/state/pid_status` | `ZitPidStatus` | 1Hz | 全轴 PID 参数回传 |
| `/auv/thrusters_cmd` | `Float64MultiArray` | - | 推进器命令（发给 Stonefish） |

**订阅：**

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/zit6/cmd/setpoint` | `ZitSetpoint` | 运动控制目标 |
| `/zit6/cmd/agxhbt` | `UInt32` | 解锁心跳 |
| `/zit6/cmd/pid` | `ZitPid` | PID 参数在线调优 |
| `/zit6/cmd/ins` | `UInt8` | INS/DVL 控制命令 |
| `/zit6/cmd/servo` | `Float32` | 舵机角度 |
| `/zit6/cmd/light` | `UInt8` | 灯光控制 |
| `/auv/odometry` | `Odometry` | Stonefish 里程计 |
| `/auv/imu` | `Imu` | Stonefish IMU |
| `/auv/dvl` | `DVL` | Stonefish DVL |
| `/auv/pressure` | `FluidPressure` | Stonefish 压力计 |
| `/sim/*/image_color` | `Image` | 相机图像 |

**坐标系处理：**
- 直接透传 Stonefish NED 坐标系，不做任何坐标转换
- 位置：`pos['x'] = odom.position.x` (North)
- 速度：`vel['x'] = odom.twist.twist.linear.x` (North)
- Yaw：NED 偏航角（弧度）

**控制器：**
- 内置 4-DOF PID 位置控制器（`_position_control_step`）
- 增量模式下自动将机体系偏移旋转到世界系
- 推力混合：4-DOF 力命令 → 6 推进器分配

### 3.3 `uv_automaton`

- 所属包: `uv_ai`
- 作用: 任务/状态机调度核心节点，AUV 的"大脑"。从 JSON 文件读取任务定义，融合检测结果，生成 ZIT6 控制指令。

**发布（ZIT6 协议话题）：**

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/zit6/cmd/setpoint` | `ZitSetpoint` | 运动控制目标 |
| `/zit6/cmd/agxhbt` | `UInt32` | 解锁心跳（20Hz） |
| `/zit6/cmd/servo` | `Float32` | 舵机角度 |
| `/zit6/cmd/light` | `UInt8` | 灯光控制 |

**订阅：**

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/zit6/state/status` | `ZitStatus` | 核心状态 |
| `/zit6/state/pos` | `Float32MultiArray` | 位置反馈 |
| `/zit6/state/vel` | `Float32MultiArray` | 速度反馈 |
| `uv_detect_down` | `Yolov8` | 下视检测结果 |
| `uv_detect_front` | `Yolov8` | 前视检测结果 |
| `down_cam/rectified/left` | `Image` | 下视相机图像 |
| `front_cam/rectified/left` | `Image` | 前视相机图像 |

**服务客户端：**
- `uv_detect_srv` (`uv_msgs/srv/DetectRequest`) - 按需检测

**控制方法：**
- `_send_setpoint(control_key, type_mask, x, y, z, rz_deg)` - 核心控制方法
- `movex(x)` - 机体系前进（NED body X）
- `movey(y)` - 机体系右移（NED body Y）
- `movez(z)` - 深度调整
- `moverz(rz)` - 偏航旋转
- `movexy(x, y)` - XY 平面移动
- `search(name, cam)` - 视觉搜索

**坐标系：**
- 全部使用 NED 坐标系
- 增量模式（`control_key & 0x20`）：机体系偏移
- 绝对模式：世界系 NED 坐标

### 3.4 `uv_position`

- 所属包: `uv_ai`
- 作用: 单目射线交汇法视觉定位节点，从 2D 检测估计 3D 世界坐标。

**发布：**

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `yolov8_data_front_pos` | `Yolov8` | 前视目标世界坐标 |
| `yolov8_data_down_pos` | `Yolov8` | 下视目标世界坐标 |

**订阅：**

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/zit6/state/pos` | `Float32MultiArray` | 机器人 NED 位置 |
| `uv_detect_front` | `Yolov8` | 前视检测结果 |
| `uv_detect_down` | `Yolov8` | 下视检测结果 |

**算法：**
1. 像素坐标 → 相机光心射线 → 机体 NED → 世界 NED
2. 历史积累（最多 20 条射线）
3. RANSAC 过滤离群射线
4. 两两交汇中值法估计 3D 坐标
5. 合理性校验（距离 < 50m）

**坐标系：**
- 直接使用 bridge 发布的 NED yaw
- 相机偏移：前视 (0.25, -0.02825, 0.25)m，下视 (0.0, 0.03765, 0.21)m

### 3.5 `uv_detect_demo`

- 所属包: `uv_ai`
- 作用: YOLO 目标检测节点。

**发布：**

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `detectedimg_down` | `Image` | 下视检测可视化图 |
| `detectedimg_front` | `Image` | 前视检测可视化图 |
| `uv_detect_down` | `Yolov8` | 下视检测结果 |
| `uv_detect_front` | `Yolov8` | 前视检测结果 |

**订阅：**

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `down_cam/rectified` | `Image` | 下视相机图像 |
| `front_cam/rectified` | `Image` | 前视相机图像 |

**服务：**
- `uv_detect_srv` (`uv_msgs/srv/DetectRequest`) - 按需检测并返回 3D 坐标

### 3.6 `uv_segment`

- 所属包: `uv_ai`
- 作用: 语义分割节点，用于巡线等特定任务。

**发布：**

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `segment_img` | `Image` | 叠加可视化图 |
| `binary_segment_img` | `Image` | 二值分割图 |

**订阅：**

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `down_cam/rectified/left` | `Image` | 下视相机左图 |

### 3.7 `uv_core`（`uv_hmu`）

- 所属包: `uv_hm`
- 作用: 实车硬件管理单元。通过串口与 STM32 通信，使用 ZIT6 协议。

**说明：** 实车环境使用 `uv_core` 节点，通过 micro-ROS agent 与固件通信。仿真环境使用 `uv_sim_bridge` 替代。

### 3.8 `uv_capture`

- 所属包: `uv_vision`
- 作用: 双目相机采集与矫正发布节点。

**发布（按参数启用）：**

- `front_cam/raw/left`, `front_cam/raw/right` (`Image`)
- `front_cam/rectified/left`, `front_cam/rectified/right` (`Image`)
- `down_cam/raw/left`, `down_cam/raw/right` (`Image`)
- `down_cam/rectified/left`, `down_cam/rectified/right` (`Image`)

### 3.9 `uv_depthimg_sim`

- 所属包: `uv_vision`
- 作用: 仿真环境专用，基于双目图像计算深度图与点云。

**发布：**
- `depthmap` (`Image`) - 可视化深度图
- `depthmap_raw` (`Image`) - 原始深度（32FC1）
- `depthmap_raw_points` (`PointCloud2`) - 点云

**订阅：**
- `/sim/front_cam/left/image_color` (`Image`)
- `/sim/front_cam/right/image_color` (`Image`)

### 3.10 `pc_recorder`

- 所属包: `uv_ai`
- 作用: 点云记录与 3D 可视化工具。

**订阅：**
- `depthmap_raw_points` (`PointCloud2`)

**功能：**
- 累积点云数据
- 支持 PySide6/pyqtgraph GUI 可视化
- 保存为 PLY 文件

## 4. ZIT6 通信协议

### 4.1 控制指令（上位机 → 固件）

| 话题 | 消息类型 | 频率 | 用途 |
|------|----------|------|------|
| `/zit6/cmd/setpoint` | `ZitSetpoint` | 10-50Hz | 运动控制目标 |
| `/zit6/cmd/agxhbt` | `UInt32` | ≥10Hz | 解锁心跳（必须持续发送） |
| `/zit6/cmd/pid` | `ZitPid` | 按需 | PID 参数在线调优 |
| `/zit6/cmd/ins` | `UInt8` | 按需 | INS/DVL 控制命令 |
| `/zit6/cmd/servo` | `Float32` | 按需 | 舵机角度 |
| `/zit6/cmd/light` | `UInt8` | 按需 | 灯光控制 |

### 4.2 状态反馈（固件 → 上位机）

| 话题 | 消息类型 | 频率 | 用途 |
|------|----------|------|------|
| `/zit6/state/status` | `ZitStatus` | 10Hz | 核心状态（解锁、错误、推力等） |
| `/zit6/state/pos` | `Float32MultiArray` | 30Hz | 位置 [x_ned, y_ned, z, yaw_ned_rad] |
| `/zit6/state/vel` | `Float32MultiArray` | 50Hz | 速度 [vx_ned, vy_ned, vz, vyaw_ned_rad_s] |
| `/zit6/state/thr` | `Float32MultiArray` | 30Hz | 推力 [Fx, Fy, Fz, Mz] |
| `/zit6/state/zithbt` | `UInt32` | 1Hz | 固件心跳（毫秒时间戳） |
| `/zit6/state/pid_status` | `ZitPidStatus` | 1Hz | 全轴 PID 参数回传 |

### 4.3 控制模式

`ZitSetpoint` 的 `control_key` 字段编码控制模式：

| Bit | 含义 | 值 |
|-----|------|-----|
| [1:0] | 控制模式 | 0=位置环, 1=速度环, 2=推力环 |
| [4] | 坐标系 | 0=世界系(NED), 1=机体系 |
| [5] | 增量模式 | 0=绝对量, 1=相对量 |

`type_mask` 控制哪些轴生效（仅位置模式有效）：

- Bit 0 (0x01): X轴
- Bit 1 (0x02): Y轴
- Bit 2 (0x04): Z轴
- Bit 3 (0x08): Yaw轴

### 4.4 心跳解锁流程

```
上电 → 固件初始化 → 上位机开始发心跳 → 等待 ≥10次 + ≥1秒 → 解锁成功 → 可发送控制指令
                                            ↓
                                     心跳不能停！
                                     500ms 超时 → 自动锁定
```

## 5. 常见通信链路

### 5.1 仿真闭环链路

1. `stonefish_simulator` 发布仿真传感器数据（NED 坐标系）
2. `uv_sim_bridge` 订阅传感器数据，发布 ZIT6 状态话题
3. `uv_detect_demo` + `uv_segment` 处理图像，发布感知结果
4. `uv_position` 融合检测结果，估计目标世界坐标
5. `uv_automaton` 融合状态与感知，发布 ZIT6 控制指令
6. `uv_sim_bridge` 接收控制指令，计算推进器命令，回灌仿真

### 5.2 实机控制链路

1. `uv_automaton` 发布 ZIT6 控制指令
2. micro-ROS agent 将指令转发给 STM32 固件
3. 固件执行控制，发布 ZIT6 状态反馈
4. `uv_automaton` 接收状态反馈，进行闭环控制

## 6. 服务接口汇总

1. `uv_detect_srv` (`uv_msgs/srv/DetectRequest`) - 目标检测服务
2. `enable_currents` (`std_srvs/srv/Trigger`) - Stonefish 开启水流
3. `disable_currents` (`std_srvs/srv/Trigger`) - Stonefish 关闭水流
4. `respawn_robot` (`stonefish_ros2/srv/Respawn`) - Stonefish 重生机器人

## 7. 运行期排障

### 7.1 `rqt` 看得到 topic 但无法解析类型

这通常不是节点发布错误，而是当前终端没有加载到正确 overlay 或消息包未构建。

推荐恢复步骤：

```bash
cd ~/AUV2026
source /opt/ros/jazzy/setup.bash
colcon build --packages-select uv_msgs zit6_interfaces stonefish_ros2 --symlink-install
source install/setup.bash
```

### 7.2 `Package 'uv_launch_pkg' not found`

```bash
cd ~/AUV2026
colcon build --packages-select uv_launch_pkg
source install/setup.bash
```

### 7.3 仿真启动后无话题

1. 检查 `uv_sim_bridge` 是否启动
2. 检查心跳是否发送：`ros2 topic hz /zit6/cmd/agxhbt`
3. 检查解锁状态：`ros2 topic echo /zit6/state/status --field is_armed`

### 7.4 控制指令无响应

1. 确认已解锁：`is_armed` 应为 `true`
2. 确认 `control_key` 编码正确
3. 检查 `type_mask` 是否包含目标轴
4. 位置模式需要 `navigation_ready == true`

## 8. 维护建议

1. 每新增节点时，按本文结构补充"作用 + pub/sub + service"。
2. 每次改 `*.launch.py` 后，更新"常见通信链路"章节。
3. 若更换 Stonefish 场景文件，需重新核对场景内传感器配置。
