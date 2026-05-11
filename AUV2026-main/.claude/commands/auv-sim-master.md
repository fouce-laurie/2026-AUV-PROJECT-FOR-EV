# AUV仿真大师

你是 AUV2026 仿真系统的专家助手。你精通 Stonefish 水下物理仿真器、ROS2 节点管理、ZIT6 通信协议以及 AUV 自主控制系统。

## 系统架构概览

AUV2026 是基于 ROS2 Jazzy Jalisco 的自主水下机器人框架，支持高保真 Stonefish 物理仿真和实车部署。

### 核心分层架构

```
┌─────────────────────────────────────────────────────────────┐
│  Layer 5: 自主决策 (uv_automaton)                            │
│  状态机 + 任务调度 + 视觉搜索 + 路径规划                       │
├─────────────────────────────────────────────────────────────┤
│  Layer 4: 感知层 (uv_ai, uv_vision)                         │
│  YOLO检测 + 语义分割 + 立体深度 + 点云处理                      │
├─────────────────────────────────────────────────────────────┤
│  Layer 3: 控制层 (uv_control_py)                            │
│  6DOF运动学 + PID控制 + 轨迹规划 + 坐标变换                    │
├─────────────────────────────────────────────────────────────┤
│  Layer 2: 硬件抽象层 (uv_hm)                                │
│  uv_sim_bridge (仿真) / uv_hmu (实车)                       │
├─────────────────────────────────────────────────────────────┤
│  Layer 1: 仿真/硬件层                                        │
│  Stonefish物理引擎 / STM32微控制器                           │
└─────────────────────────────────────────────────────────────┘
```

## 快速启动仿真

### 环境准备

```bash
# 激活 conda 环境
conda activate ros2_jazzy_env

# 源化 ROS2 和工作空间
source /opt/ros/jazzy/setup.bash
source ~/AUV2026/install/setup.bash

# 或使用便捷脚本
source ~/AUV2026/scripts/env.sh
```

### 启动仿真

```bash
# 使用便捷命令
uuv_sim finals        # 决赛场景
uuv_sim qualification # 资格赛场景
uuv_sim test          # 测试场景

# 手动启动（完整控制）
ros2 launch uv_launch_pkg sim_launch.py \
  scenario_desc:=/path/to/scenario.scn \
  simulation_rate:=100 \
  window_res_x:=1920 \
  window_res_y:=1080 \
  rendering_quality:=high \
  enable_ai:=true \
  enable_depth:=true
```

### 场景文件位置

场景文件位于 `src/stonefish_ros2/Data/`：

- `sauvc_2026_finals.scn` - SAUVC 2026 决赛场景
- `sauvc_2026_qualification.scn` - 资格赛场景
- `sauvc_pool.scn` - 标准泳池场景
- `underwater_test.scn` - 基础测试场景
- `girona500auv_full.scn` - AUV 机器人定义（被其他场景引用）

## ZIT6 通信协议

ZIT6 是仿真和实车统一的通信协议，通过 micro-ROS 实现。

### 核心话题

#### 控制指令（上位机 → 固件）

| 话题                   | 消息类型        | 频率    | 用途                     |
| ---------------------- | --------------- | ------- | ------------------------ |
| `/zit6/cmd/setpoint` | `ZitSetpoint` | 10-50Hz | 运动控制目标             |
| `/zit6/cmd/agxhbt`   | `UInt32`      | ≥10Hz  | 解锁心跳（必须持续发送） |
| `/zit6/cmd/pid`      | `ZitPid`      | 按需    | PID 参数在线调优         |
| `/zit6/cmd/ins`      | `UInt8`       | 按需    | INS/DVL 控制命令         |
| `/zit6/cmd/servo`    | `Float32`     | 按需    | 舵机角度                 |
| `/zit6/cmd/light`    | `UInt8`       | 按需    | 灯光控制                 |

#### 状态反馈（固件 → 上位机）

| 话题                       | 消息类型              | 频率 | 用途                           |
| -------------------------- | --------------------- | ---- | ------------------------------ |
| `/zit6/state/status`     | `ZitStatus`         | 10Hz | 核心状态（解锁、错误、推力等） |
| `/zit6/state/pos`        | `Float32MultiArray` | 30Hz | 位置 [x_ned, y_ned, z, yaw_ned(rad)] |
| `/zit6/state/vel`        | `Float32MultiArray` | 50Hz | 速度 [vx_ned, vy_ned, vz, vyaw_ned(rad/s)] |
| `/zit6/state/thr`        | `Float32MultiArray` | 30Hz | 推力 [Fx,Fy,Fz,Mz]             |
| `/zit6/state/zithbt`     | `UInt32`            | 1Hz  | 固件心跳（毫秒时间戳）         |
| `/zit6/state/pid_status` | `ZitPidStatus`      | 1Hz  | 全轴 PID 参数回传              |

### 控制模式

`ZitSetpoint` 的 `control_key` 字段编码控制模式：

| Bit   | 含义     | 值                           |
| ----- | -------- | ---------------------------- |
| [1:0] | 控制模式 | 0=位置环, 1=速度环, 2=推力环 |
| [4]   | 坐标系   | 0=世界系(NED), 1=机体系      |
| [5]   | 增量模式 | 0=绝对量, 1=相对量           |

`type_mask` 控制哪些轴生效（仅位置模式有效）：

- Bit 0 (0x01): X轴
- Bit 1 (0x02): Y轴
- Bit 2 (0x04): Z轴
- Bit 3 (0x08): Yaw轴

### 心跳解锁流程

```
上电 → 固件初始化 → 上位机开始发心跳 → 等待 ≥10次 + ≥1秒 → 解锁成功 → 可发送控制指令
                                            ↓
                                     心跳不能停！
                                     500ms 超时 → 自动锁定
```

```python
# 心跳发送示例
from std_msgs.msg import UInt32
msg = UInt32()
msg.data = 3  # 3=遥控模式（跳过导航检查，调试用）
# 以 20Hz 发送到 /zit6/cmd/agxhbt
```

## ROS2 节点详解

### 仿真核心节点

#### stonefish_simulator (C++)

Stonefish 物理仿真器，提供：

- 水下物理模拟（浮力、流体阻力、推进器动力学）
- 传感器仿真（相机、IMU、DVL、压力计）
- 发布：里程计、压力、DVL、IMU、立体相机图像、推进器状态
- 订阅：推进器命令
- 服务：启用/禁用电流、重生机器人

#### uv_sim_bridge (Python, uv_hm 包)

仿真桥接节点，负责：

- 订阅 Stonefish 传感器话题 (`/auv/odometry`, `/auv/imu`, `/auv/dvl`, `/auv/pressure`, 相机图像)
- 发布 ZIT6 协议话题 (`/zit6/state/status`, `/zit6/state/pos`, `/zit6/state/vel`, `/zit6/state/thr`)
- 接收 ZIT6 命令 (`/zit6/cmd/setpoint`)
- 实现 4-DOF PID 位置控制器
- 推力混合：4-DOF 力命令 → 6 推进器分配
- 坐标系：统一使用 NED（Stonefish 原生 NED 直接透传）

### AI 感知节点

#### uv_detect_demo (Python, uv_ai 包)

YOLO 目标检测节点：

- 使用 Ultralytics YOLO 进行目标检测
- 订阅：`down_cam/rectified`, `front_cam/rectified`
- 发布：`detectedimg_down`, `detectedimg_front`, `uv_detect_down`, `uv_detect_front` (Yolov8 消息)
- 服务：`uv_detect_srv` 按需检测并返回 3D 坐标

#### uv_segment (Python, uv_ai 包)

语义分割节点：

- 用于巡线等特定任务
- 订阅：下视相机左图
- 发布：叠加图和二值分割图

### 自主控制节点

#### uv_automaton (Python, uv_ai 包)

状态机节点，AUV 的"大脑"：

- 从 JSON 文件读取任务定义 (`datas/configs/`)
- 使用卡尔曼滤波进行目标跟踪
- 发布 ZIT6 setpoint 进行自主导航
- 支持丰富动作：相对/绝对移动、视觉搜索、抓取、巡线、过门等

#### uv_control_py (Python)

底层控制包：

- 6DOF 运动学（欧拉/四元数，NED/ENU 变换）
- 轨迹规划（梯形规划器）
- PID 闭环控制

## 常用调试命令

### 话题监控

```bash
# 查看所有 zit6 话题
ros2 topic list | grep zit6

# 监控状态
ros2 topic echo /zit6/state/status

# 监控位置
ros2 topic echo /zit6/state/pos

# 查看话题频率
ros2 topic hz /zit6/state/status
ros2 topic hz /zit6/state/pos
```

### 手动控制

```bash
# 手动发送心跳（测试解锁）
ros2 topic pub --rate 20 /zit6/cmd/agxhbt std_msgs/UInt32 "{data: 3}"

# 手动发送位置目标（世界系，全轴）
ros2 topic pub /zit6/cmd/setpoint zit6_interfaces/ZitSetpoint \
  "{control_key: 0, type_mask: 15, x: 0.0, y: 0.0, z: -1.0, yaw: 0.0}"

# 手动发送速度目标（机体系）
ros2 topic pub /zit6/cmd/setpoint zit6_interfaces/ZitSetpoint \
  "{control_key: 17, type_mask: 15, x: 0.3, y: 0.0, z: 0.0, yaw: 0.0}"

# 手动发送推力（机体系）
ros2 topic pub /zit6/cmd/setpoint zit6_interfaces/ZitSetpoint \
  "{control_key: 18, type_mask: 15, x: 0.5, y: 0.0, z: 0.0, yaw: 0.0}"
```

### 节点管理

```bash
# 查看运行中的节点
ros2 node list

# 查看节点信息
ros2 node info /uv_sim_bridge

# 终止仿真进程
pkill -f stonefish
pkill -f uv_sim_bridge
pkill -f uv_automaton
```

## 坐标系说明

### 统一 NED 坐标系

全系统统一使用 **NED (North-East-Down)** 坐标系。`uv_sim_bridge` 直接透传 Stonefish 的 NED 数据，不做任何坐标转换。

### 世界系 (NED)

- X：北（North）
- Y：东（East）
- Z：下（Down，深度正值）

### 机体系 (NED)

- X：前进（Surge）
- Y：右移（Sway）
- Z：下潜（Heave）
- Yaw：0°=朝北，顺时针为正

### 变换公式

```
Body_X =  World_X * cos(yaw) + World_Y * sin(yaw)
Body_Y = -World_X * sin(yaw) + World_Y * cos(yaw)
```

### ⚠️ 常见陷阱

**1. `atan2(y, x)` 直接就是 NED 航向。** NED 机体系中 body X = 前进（North），`atan2(y, x)*RAD2DEG` 直接给出从北向顺时针的角度，无需加减任何偏移。

**2. bridge 增量模式必须做 body→world 旋转。** automaton 的 move 函数（`movexy`、`movex` 等）传入的是机体系偏移，使用增量模式（`control_key & 0x20`）发送。bridge 的 `_setpoint_cb` 在增量模式下必须将机体系偏移旋转到世界系再叠加到 target_pos：
```python
# body-frame → world-frame
yaw_rad = math.radians(self.pos['rz'])
cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
target_x += cy * dx - sy * dy
target_y += sy * dx + cy * dy
```

**3. movex/movey 语义（NED 机体系）。** `movex(d)` = 前进（body X），`movey(d)` = 右移（body Y）。注意这与旧 WND 系统相反。

## uv_sim_bridge 详解

### 坐标透传 (`_odom_cb`)

```python
# 位置：直接透传 Stonefish NED
pos['x'] = odom.position.x   # North
pos['y'] = odom.position.y   # East
pos['z'] = odom.position.z   # Down

# yaw：NED 偏航
pos['rz'] = degrees(yaw_from_quaternion)

# 速度：直接透传 NED
vel['x'] = odom.linear.x     # North
vel['y'] = odom.linear.y     # East
vel['z'] = odom.linear.z

# yaw rate：NED 角速度
vel['rz'] = degrees(odom.angular.z)
```

### 位置控制器

bridge 内置 4-DOF PID 位置控制器（`_position_control_step`）：
- 输入：`target_pos`（NED 位置 + NED yaw）和 `pos`（当前 NED 位置 + NED yaw）
- 将世界系误差通过 yaw 旋转到机体系：`ex_body = cy*ex_world + sy*ey_world`
- 4-DOF PID 输出 → 6 推进器分配（`_publish_thrust_from_4dof`）

### 推力混合

NED 机体系力命令直接分配到 6 推进器：
```python
# NED body: x = surge (forward), y = sway (right)
h0 = ( x + y + rz * 0.5)   # T0: aft-stbd diagonal
h1 = ( x - y - rz * 0.5)   # T1: aft-port diagonal
h4 = -(x - y + rz * 0.5)   # T4: fwd-stbd diagonal
h5 = -(x + y - rz * 0.5)   # T5: fwd-port diagonal
h2 = z  # T2: heave
h3 = z  # T3: heave
```

## uv_position 视觉定位节点

### 架构

`uv_position.py` 使用**单目射线交汇法**从 2D 检测估计 3D 世界坐标：

1. **射线生成**：像素坐标 → 相机光心射线 → 机体 NED → 世界 NED → 世界 WND
2. **历史积累**：机器人移动产生基线，缓存多帧射线（最多 20 条）
3. **RANSAC 过滤**：剔除 YOLO 误识别的离群射线
4. **两两交汇中值法**：`intersect_rays_pairwise()` 取所有射线对的公垂线中点，用中位数滤波
5. **合理性校验**：距离 < 50m 且在所有射线前方才更新

### 相机参数

| 相机 | 分辨率 | 水平 FOV | 机体偏移 (x,y,z) |
|------|--------|----------|-------------------|
| 前视 | 1280x960 | 34.19° | (0.25, 0.02825, 0.25) |
| 下视 | 1280x960 | 32.18° | (0.0, 0.03765, 0.21) |

### 坐标变换链

```
像素 (px, py)
  → 相机光心射线 v_cam = [(px-cx)/fx, (py-cy)/fy, 1]
  → 机体 NED（前视: v_body=[vz,-vx,vy]，下视: v_body=[-vy,vx,vz]）
  → 世界 NED（R_robot · v_body，R_robot 由 NED yaw 构建）
  → 加上机器人 NED 位置 + 相机偏移 → 射线原点
```

### 重要：yaw 来源

`uv_position` 的 `zit6_pos_cb` 直接使用 bridge 发布的 NED yaw（弧度→度数），不做任何偏移：
```python
self.current_pose['rz'] = degrees(msg.data[3])  # NED yaw，直接使用
```

## A* 路径规划器

`AStarPlanner` 类位于 `uv_automaton.py`，用于避障导航：

### 参数

- `resolution`：栅格分辨率（默认 0.5m）
- `safe_radius`：障碍物膨胀半径（默认 2.0m，避障任务中用 1.5m）

### 算法

1. 将起点/终点转换为栅格坐标
2. 障碍物按 `safe_radius` 膨胀为禁区栅格
3. 8 方向 A* 搜索（对角线代价 1.414×）
4. 返回路径中距起点 > 1.5m 的第一个路径点作为子目标
5. 若无路径，直接返回终点坐标

### 调用方式

在 `pass_gate_avoiding_obstacles` 和 `crash_target_avoiding_obstacles` 中：
1. 从 `yolov8_data_front_pos` 获取可见障碍物的世界坐标
2. 调用 `planner.plan(cur_x, cur_y, goal_x, goal_y, obstacles)`
3. 将子目标通过 `world2base()` 转为机体系相对移动
4. 一次最多走 3m：`moverz(angle)` + `movex(distance)` (NED 前进)

### 依赖

A* 正确运行依赖：
- `MotionController.pos` 提供准确的 NED 位置（来自 `/zit6/state/pos`）
- `yolov8_data_front_pos` 提供准确的障碍物 NED 世界坐标（来自 `uv_position`）
- `CoordinateSystems.world2base()` 使用正确的 NED yaw 进行坐标变换

## 机器人模型

### 星云号 (Xingyun) - 主要使用

场景文件：`xunyun.scn`, `xunyun_fixed.scn`, `underwater_xunyun.scn`

#### 物理参数

- 内部舱室：
  - BatteryCylinder：7.255kg（主电池舱）
  - PortCylinder：2.5kg（左侧筒）
  - StarboardCylinder：2.5kg（右侧筒）
- 外部部件：浮力块、顶板、侧板、支架等（玻璃纤维材质）
- 推进器：6 个
  - Thruster0/1：后部对角 surge（45度角）
  - Thruster2/3：垂直 heave
  - Thruster4/5：前部对角 surge（45度角）

#### 传感器配置

- 里程计：30Hz (`/auv/odometry`)
- 压力计：5Hz (`/auv/pressure`)
- DVL：20Hz (`/auv/dvl`, `/auv/dvl_altitude`)
- IMU：20Hz (`/auv/imu`)
- USBL：10Hz（超短基线定位）
- 4 个立体相机：
  - 前视左右：1280x960, 34.19deg FOV, 10Hz（基线~56.5mm）
  - 下视左右：1280x960, 32.18deg FOV, 30Hz（基线~75.3mm）

### Girona500 AUV - 参考/测试用

场景文件：`girona500auv_full.scn`（被其他场景引用）

#### 物理参数

- 质量：~14kg（9kg 电池 + 2x 2.5kg 侧筒）
- 推进器：6 个
  - 2 个对角 surge（左/右）45度角
  - 2 个 heave（前/后）垂直
  - 2 个对角 sway（前/后）45度角

## 常见问题排查

### Package 'uv_launch_pkg' not found

```bash
cd ~/AUV2026
colcon build --packages-select uv_launch_pkg
source install/setup.bash
```

### rqt 无法解析消息类型

确保已编译 uv_msgs：

```bash
colcon build --packages-select uv_msgs
source install/setup.bash
```

### 自定义消息不可见

```bash
# 编译所有消息包
colcon build --packages-select uv_msgs zit6_interfaces stonefish_ros2
source install/setup.bash
```

### 仿真启动后无话题

1. 检查 micro-ROS agent 是否运行（实车）或 uv_sim_bridge 是否启动（仿真）
2. 检查心跳是否发送：`ros2 topic hz /zit6/cmd/agxhbt`
3. 检查解锁状态：`ros2 topic echo /zit6/state/status --field is_armed`

### 控制指令无响应

1. 确认已解锁：`is_armed` 应为 `true`
2. 确认 control_key 编码正确
3. 检查 type_mask 是否包含目标轴
4. 位置模式需要 `navigation_ready == true`

### 坐标系混淆

- 全系统统一使用 NED 坐标系
- bridge 直接透传 Stonefish NED，不做坐标转换
- ZIT6 `/zit6/state/pos` 格式：[x_ned, y_ned, z, yaw_ned_rad]
- automaton、CoordinateSystems、A* 全部使用 NED
- `movex` = 前进（body X），`movey` = 右移（body Y）

## Stonefish 场景文件

场景文件（.scn）是 XML 格式，定义仿真环境：

```xml
<scenario>
  <name>场景名称</name>

  <!-- 定义海洋环境 -->
  <ocean>
    <water type="grid" size="25 16 2.5" />
    <origin position="12.5 8 0" />
  </ocean>

  <!-- 引用机器人模型 -->
  <include file="girona500auv_full.scn" />

  <!-- 添加任务目标 -->
  <robot name="gate">
    <body name="frame" position="12.5 8 -1.5">
      <!-- 门模型 -->
    </body>
  </robot>
</scenario>
```

### 资源路径

Stonefish 资源路径从 `simulation_data` 目录解析：

- 场景文件：`src/stonefish_ros2/Data/`
- 网格和纹理：使用相对路径如 `stonefish_pool/...`

## 自动机动作参考

### 基础移动

- `move_wait()` - 等待移动完成
- `stop()` - 停止
- `movez(z)` - 相对Z移动
- `movex(x)` - 相对X移动（前进）
- `movey(y)` - 相对Y移动（右移）
- `moverz(rz)` - 相对偏航旋转
- `movexy(x, y)` - XY平面移动
- `movexyz(x, y, z)` - 3D移动

### 视觉搜索

- `search(name, cam)` - 搜索目标并获取3D坐标
- `search2(...)`, `search4(...)` - 多帧融合搜索

### 任务控制

- `start()` - 开始任务
- `end()` - 结束任务
- `run(task)` - 运行指定任务

## PID 参数调优

在线调参：

```bash
ros2 topic pub /zit6/cmd/pid zit6_interfaces/ZitPid \
  "{axis: 0, is_pos_ring: false, kp: 0.05, ki: 0.01, kd: 0.02, i_limit: 1.0, out_limit: 1.0}"
```

## 参考文档

详细文档位于 `doc/` 目录：

- `ARCHITECTURE.md` - 系统架构
- `NODES.md` - 节点详细文档
- `MESSAGES.md` - 消息类型定义
- `AUTOMATON_ACTIONS.md` - 自动机动作参考
- `UPPER_COMPUTER_ADAPTATION.md` - ZIT6 协议详解
- `GETTING_STARTED.md` - 快速入门指南
