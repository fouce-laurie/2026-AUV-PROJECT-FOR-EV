# AUV2026 系统架构与设计文档

## 1. 系统概述
AUV2026 是一个基于 ROS2 (Jazzy) 的自主水下机器人(Autonomous Underwater Vehicle) 控制与仿真框架，由之前的 UUV2025 系统重构而来。系统不仅支持接入真实机器人的硬件控制，还完整集成了基于 Stonefish 的高保真物理仿真器，支持端到端的水下视觉、导航和自主任务系统测试。

## 2. 核心系统模块
整个工作空间可划分为以下几个核心子系统：

### 2.1 感知与视觉子系统 (`uv_vision`, `uv_ai`)
- **uv_vision (底层视觉与处理)**：
  负责相机图像获取（真实相机 `uv_capture`）、基于双目的深度估计（`uv_depthimg` / `stereocam`），以及通过 `uv_depthimg_sim` 利用仿真图像生成对应的深度信息和点云。
- **uv_ai (高阶感知与自主决策)**：
  利用 YOLO 模型进行目标检测（`uv_detect_demo`）并计算目标的三维坐标，或者进行图像语义分割（`uv_segment`）。此外，还包含 3D 点云的记录和可视化（`pc_recorder`）。

### 2.2 自主导航与任务调度系统 (`uv_ai`)
- **uv_automaton**：
  整个 AUV 的"大脑"。采用状态机(State Machine)架构，根据 `datas/configs/` 中预设的 JSON 任务流（如巡线、撞门、抓取），读取传感器状态及 AI 检测结果，生成运动控制指令传给底层。

### 2.3 底层控制与运动解算 (`uv_control_py`)
- 提供水下机器人六自由度（6-DoF）的运动学计算（如 `CoordinateSystem` 的欧拉角/四元数及 NED/ENU 变换）、运动轨迹规划（`Curve`）以及闭环位姿控制（`Pid`）。它将高级的位置或速度指令转换为各个推进器的推力指令。

### 2.4 硬件与仿真桥接 (`uv_hm`, `stonefish_ros2`)
- **uv_hm (Hardware Manager)**：
  硬件抽象层。当在真实环境中运行时，通过 `uv_hmu` 节点经串口解析传感器数据。在仿真环境中，通过 `uv_sim_bridge` 将 Stonefish 发布的话题转化为实际系统通用的消息类型。
- **stonefish_ros2**：
  高效的 C++ 物理仿真引擎封装。处理多流体动力学、传感器的物理反馈（DVL交互、IMU等）并渲染相机画面。

## 3. 数据流与消息交互

### 结构图抽象 (Data Flow):
```text
[ 仿真器 Stonefish ]  <--推力指令--- [ 仿真桥接 Bridge ]
        |                                    |
(图像, IMU, 深度, DVL)                    (统一格式化的话题数据)
        |                                    |
        v                                    v
[ 视觉模块 uv_vision/uv_ai ]            [ 导航与状态机 uv_automaton ]
        |                                    |
(YOLO框, 3D坐标, 点云)                  (位姿设定, 运动指令)
        \                                    /
         \---->[ 解算与控制 uv_control_py ]</
                     |
                     v
             [ 推力分配计算 ]
```

## 4. 坐标系规范 (Coordinate Systems)
全系统统一使用 **NED (北东地)** 坐标系：
- 世界系：X=北(North), Y=东(East), Z=下(Down)
- 机体系：X=前进(Surge), Y=右移(Sway), Z=下潜(Heave)
- Yaw：0°=朝北，顺时针为正
- `uv_sim_bridge` 直接透传 Stonefish 的 NED 数据，不做坐标转换
- `CoordinateSystem` 类的 `world2base()`/`base2world()` 使用标准 ZYX 欧拉角旋转矩阵
