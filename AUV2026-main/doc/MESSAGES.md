# AUV2026 消息文档

本文档覆盖 AUV2026 系统中使用的两类消息包：

- `zit6_interfaces` - ZIT6 协议消息（仿真和实车统一使用）
- `uv_msgs` - 传统消息（部分仍在使用）

---

## 第一部分：zit6_interfaces 消息

ZIT6 是 AUV2026 的核心通信协议，通过 micro-ROS 实现上位机与固件之间的标准化通信。

### 1. 消息列表

1. `ZitSetpoint.msg` - 运动控制目标
2. `ZitStatus.msg` - 核心状态反馈
3. `ZitPid.msg` - PID 参数设置
4. `ZitPidStatus.msg` - PID 状态回传

### 2. 消息定义详情

#### 2.1 `ZitSetpoint.msg`

运动控制目标消息，用于发送位置/速度/推力控制指令。

| 字段 | 类型 | 说明 |
|------|------|------|
| `control_key` | `uint8` | 控制模式编码（见下表） |
| `type_mask` | `uint8` | 轴掩码（仅位置模式有效） |
| `x` | `float32` | X 轴目标值 |
| `y` | `float32` | Y 轴目标值 |
| `z` | `float32` | Z 轴目标值 |
| `yaw` | `float32` | 偏航目标值（弧度） |
| `seq` | `uint32` | 序列号（可选） |

**`control_key` 编码表：**

| Bit | 含义 | 值 |
|-----|------|-----|
| [1:0] | 控制模式 | 0=位置环, 1=速度环, 2=推力环 |
| [4] | 坐标系 | 0=世界系(NED), 1=机体系 |
| [5] | 增量模式 | 0=绝对量, 1=相对量 |

**`type_mask` 编码表（仅位置模式有效）：**

| Bit | 轴 | 值 |
|-----|-----|-----|
| 0 | X | 0x01 |
| 1 | Y | 0x02 |
| 2 | Z | 0x04 |
| 3 | Yaw | 0x08 |

**常用 control_key 值：**
- `0x00` = 位置模式 + 世界系 + 绝对量
- `0x20` = 位置模式 + 世界系 + 增量模式
- `0x10` = 位置模式 + 机体系
- `0x01` = 速度模式 + 世界系
- `0x11` = 速度模式 + 机体系
- `0x02` = 推力模式 + 世界系
- `0x12` = 推力模式 + 机体系

**代码示例：**

```python
from zit6_interfaces.msg import ZitSetpoint

# 位置模式（世界系，全轴）
msg = ZitSetpoint()
msg.control_key = 0x00       # 位置模式 + 世界系
msg.type_mask = 0x0F         # 全轴生效
msg.x = 1.0                  # 目标 X 位置 (m)
msg.y = 0.0                  # 目标 Y 位置 (m)
msg.z = -2.0                 # 目标深度 (m, 负值=下潜)
msg.yaw = 0.0                # 目标航向 (rad)

# 增量模式（机体系前进 1m）
msg = ZitSetpoint()
msg.control_key = 0x20       # 增量模式
msg.type_mask = 0x01         # 仅 X 轴
msg.x = 1.0                  # 前进 1m
```

#### 2.2 `ZitStatus.msg`

核心状态反馈消息，10Hz 发布。

| 字段 | 类型 | 说明 |
|------|------|------|
| `is_armed` | `bool` | 解锁状态 |
| `arm_mode` | `uint8` | 解锁模式 |
| `control_level` | `uint8` | 当前控制级别 (0=无, 1=位置, 2=速度, 3=推力) |
| `ins_state` | `uint8` | INS 状态 (0=待机, 1=粗对准, 2=精对准, 3=SINS/GPS/DVL, 4=SINS/DVL, 5=MRU) |
| `navigation_ready` | `bool` | 导航就绪（位置模式需要） |
| `forces` | `float32[4]` | 推力 [Fx, Fy, Fz, Mz] |
| `cycle_time_ms` | `uint16` | 控制循环耗时 (ms) |
| `battery_voltage` | `float32` | 电池电压（当前固件硬编码为 0） |
| `error_flags` | `uint32` | 错误标志（当前固件硬编码为 0） |

**代码示例：**

```python
from zit6_interfaces.msg import ZitStatus

def status_callback(msg):
    if msg.is_armed:
        print("已解锁")
    if msg.navigation_ready:
        print("导航就绪，INS状态:", msg.ins_state)
```

#### 2.3 `ZitPid.msg`

PID 参数设置消息，用于在线调参。

| 字段 | 类型 | 说明 |
|------|------|------|
| `axis` | `uint8` | 轴编号 (0=X, 1=Y, 2=Z, 3=Yaw) |
| `is_pos_ring` | `bool` | true=位置环, false=速度环 |
| `kp` | `float32` | 比例系数 |
| `ki` | `float32` | 积分系数（位置环无效） |
| `kd` | `float32` | 微分系数（位置环无效） |
| `i_limit` | `float32` | 积分限幅 |
| `out_limit` | `float32` | 输出限幅 |
| `max_v` | `float32` | 最大速度（仅位置环有效） |
| `max_a` | `float32` | 最大加速度（仅位置环有效） |

**注意：** 负值表示不更新（保留当前值）。

**代码示例：**

```python
from zit6_interfaces.msg import ZitPid

# 设置 X 轴速度环 PID
msg = ZitPid()
msg.axis = 0              # X轴
msg.is_pos_ring = False   # 速度环
msg.kp = 0.05
msg.ki = 0.01
msg.kd = 0.02
msg.i_limit = 1.0
msg.out_limit = 1.0
msg.max_v = -1.0          # 不更新
msg.max_a = -1.0          # 不更新
```

#### 2.4 `ZitPidStatus.msg`

PID 状态回传消息，1Hz 发布。

| 字段 | 类型 | 说明 |
|------|------|------|
| `pos_kp` | `float32[4]` | 位置环 kp [X, Y, Z, Yaw] |
| `vel_kp` | `float32[4]` | 速度环 kp [X, Y, Z, Yaw] |
| `vel_ki` | `float32[4]` | 速度环 ki [X, Y, Z, Yaw] |
| `vel_kd` | `float32[4]` | 速度环 kd [X, Y, Z, Yaw] |

### 3. 标准消息类型

ZIT6 协议还使用以下标准 ROS2 消息类型：

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/zit6/cmd/agxhbt` | `std_msgs/UInt32` | 解锁心跳 |
| `/zit6/cmd/ins` | `std_msgs/UInt8` | INS/DVL 控制命令 |
| `/zit6/cmd/servo` | `std_msgs/Float32` | 舵机角度 |
| `/zit6/cmd/light` | `std_msgs/UInt8` | 灯光控制 |
| `/zit6/state/pos` | `std_msgs/Float32MultiArray` | 位置 [x, y, z, yaw_rad] |
| `/zit6/state/vel` | `std_msgs/Float32MultiArray` | 速度 [vx, vy, vz, vyaw_rad_s] |
| `/zit6/state/thr` | `std_msgs/Float32MultiArray` | 推力 [Fx, Fy, Fz, Mz] |
| `/zit6/state/zithbt` | `std_msgs/UInt32` | 固件心跳 |

---

## 第二部分：uv_msgs 消息

传统消息包，部分功能仍在使用（如视觉检测服务）。

本文档根据 `src/uv_msgs` 目录下的定义整理，覆盖：

- `msg/*.msg` 消息
- `srv/*.srv` 服务

## 1. 包内容概览

### 1.1 消息列表（`msg/`）

1. `CabinState.msg`
2. `ImuData.msg`
3. `LedControllers.msg`
4. `MagnetController.msg`
5. `MotorThrust.msg`
6. `PidControllers.msg`
7. `PidControllersState.msg`
8. `PidParams.msg`
9. `PixelAxis.msg`
10. `PropellerThrust.msg`
11. `RobotAxis.msg`
12. `RobotDeviceManager.msg`
13. `RobotMotionController.msg`
14. `ServoSet.msg`
15. `TargetAxis.msg`
16. `TargetParams.msg`
17. `TargetPosDown.msg`
18. `ThrustCurve.msg`
19. `ThrustCurves.msg`
20. `WorkState.msg`
21. `Yolov8.msg`

### 1.2 服务列表（`srv/`）

1. `DetectRequest.srv`

## 2. 通用类型约定

- `RobotAxis`：六自由度轴量（`x/y/z/rx/ry/rz`），常用于位置、速度、姿态等。
- `TargetAxis`：三维目标坐标（`x/y/z`）。
- `PixelAxis`：图像像素坐标（`x/y`）。
- 多处使用 `uint8` 作为状态位/开关量，建议在上层代码中维护统一枚举常量。

## 3. 消息定义详情

### 3.1 `CabinState.msg`

舱体环境与基础执行器状态。

| 字段        | 类型           | 说明         |
| ----------- | -------------- | ------------ |
| `temp`    | `float32`    | 温度         |
| `hum`     | `float32`    | 湿度         |
| `leak`    | `uint8`      | 漏水状态     |
| `voltage` | `float32`    | 电压         |
| `servo`   | `float32[2]` | 两路舵机角度 |

### 3.2 `ImuData.msg`

导航模式、DVL 标志位及位姿速度。

| 字段     | 类型          | 说明                   |
| -------- | ------------- | ---------------------- |
| `mode` | `uint8`     | 导航模式（见下方枚举） |
| `dvl`  | `uint8`     | DVL 状态（见下方枚举） |
| `pos`  | `RobotAxis` | 姿态/位置轴量          |
| `spd`  | `RobotAxis` | 速度轴量               |

`mode` 取值（来自消息内注释）：

- `0x00`：待机
- `0x01`：粗对准
- `0x02`：精对准
- `0x04`：SINS/DVL
- `0x05`：MRU（无 DVL 数据时自动进入姿态模式）
- `0xFF`：系统故障

`dvl` 取值（来自消息内注释）：

- `0x00`：DVL 未上传数据
- `0x01`：DVL 数据更新但无效
- `0x02`：DVL 数据更新且有效

### 3.3 `LedControllers.msg`

灯光控制。

| 字段     | 类型      | 说明        |
| -------- | --------- | ----------- |
| `led0` | `uint8` | LED0 控制量 |
| `led1` | `uint8` | LED1 控制量 |

### 3.4 `MagnetController.msg`

电磁铁控制。

| 字段      | 类型      | 说明       |
| --------- | --------- | ---------- |
| `state` | `uint8` | 电磁铁状态 |

### 3.5 `MotorThrust.msg`

六推进器推力。

| 字段       | 类型           | 说明           |
| ---------- | -------------- | -------------- |
| `thrust` | `float32[6]` | 6 路推进器推力 |

### 3.6 `PidParams.msg`

单个 PID 参数结构。

| 字段             | 类型        | 说明     |
| ---------------- | ----------- | -------- |
| `name`         | `string`  | PID 名称 |
| `p`            | `float32` | 比例系数 |
| `i`            | `float32` | 积分系数 |
| `d`            | `float32` | 微分系数 |
| `i_limit`      | `float32` | 积分限幅 |
| `output_limit` | `float32` | 输出限幅 |

### 3.7 `PidControllers.msg`

多轴 PID 参数集合。

| 字段   | 类型          | 说明         |
| ------ | ------------- | ------------ |
| `x`  | `PidParams` | X 轴 PID     |
| `y`  | `PidParams` | Y 轴 PID     |
| `z`  | `PidParams` | Z 轴 PID     |
| `rx` | `PidParams` | 横滚轴 PID   |
| `ry` | `PidParams` | 俯仰轴 PID   |
| `rz` | `PidParams` | 偏航轴 PID   |
| `vx` | `PidParams` | X 向速度 PID |
| `vy` | `PidParams` | Y 向速度 PID |

### 3.8 `PidControllersState.msg`

各 PID 通道启停状态。

| 字段   | 类型      | 说明              |
| ------ | --------- | ----------------- |
| `x`  | `uint8` | X 轴 PID 状态     |
| `y`  | `uint8` | Y 轴 PID 状态     |
| `z`  | `uint8` | Z 轴 PID 状态     |
| `rx` | `uint8` | 横滚轴 PID 状态   |
| `ry` | `uint8` | 俯仰轴 PID 状态   |
| `rz` | `uint8` | 偏航轴 PID 状态   |
| `vy` | `uint8` | Y 向速度 PID 状态 |
| `vx` | `uint8` | X 向速度 PID 状态 |

### 3.9 `PixelAxis.msg`

二维像素坐标。

| 字段  | 类型       | 说明   |
| ----- | ---------- | ------ |
| `x` | `uint32` | 像素 X |
| `y` | `uint32` | 像素 Y |

### 3.10 `PropellerThrust.msg`

六推进器推力（与 `MotorThrust` 字段一致）。

| 字段       | 类型           | 说明           |
| ---------- | -------------- | -------------- |
| `thrust` | `float32[6]` | 6 路推进器推力 |

### 3.11 `RobotAxis.msg`

机器人六自由度轴量。

| 字段   | 类型        | 说明 |
| ------ | ----------- | ---- |
| `x`  | `float32` | X    |
| `y`  | `float32` | Y    |
| `z`  | `float32` | Z    |
| `rx` | `float32` | 横滚 |
| `ry` | `float32` | 俯仰 |
| `rz` | `float32` | 偏航 |

### 3.12 `RobotDeviceManager.msg`

设备管理状态汇总。

| 字段       | 类型           | 说明          |
| ---------- | -------------- | ------------- |
| `leak`   | `uint8`      | 漏水状态      |
| `tem`    | `float32`    | 温度          |
| `hum`    | `float32`    | 湿度          |
| `vol`    | `float32`    | 电压          |
| `magnet` | `uint8`      | 电磁铁状态    |
| `led`    | `uint8[2]`   | 两路 LED 状态 |
| `angle`  | `float32[2]` | 两路角度值    |

### 3.13 `RobotMotionController.msg`

运动控制状态总线。

| 字段             | 类型                    | 说明               |
| ---------------- | ----------------------- | ------------------ |
| `pos`          | `RobotAxis`           | 当前位姿           |
| `tpos_inbase`  | `RobotAxis`           | 基坐标系目标位姿   |
| `tpos_inworld` | `RobotAxis`           | 世界坐标系目标位姿 |
| `imu`          | `ImuData`             | IMU/导航信息       |
| `thrust`       | `MotorThrust`         | 推进器推力         |
| `pidstate`     | `PidControllersState` | PID 通道状态       |

### 3.14 `ServoSet.msg`

单次舵机设定。

| 字段      | 类型        | 说明     |
| --------- | ----------- | -------- |
| `num`   | `uint8`   | 舵机编号 |
| `angle` | `float32` | 目标角度 |

### 3.15 `TargetAxis.msg`

三维目标坐标。

| 字段  | 类型        | 说明   |
| ----- | ----------- | ------ |
| `x` | `float32` | 目标 X |
| `y` | `float32` | 目标 Y |
| `z` | `float32` | 目标 Z |

### 3.16 `TargetParams.msg`

目标图像坐标 + 世界坐标。

| 字段             | 类型           | 说明     |
| ---------------- | -------------- | -------- |
| `tpos_inpic`   | `PixelAxis`  | 图像坐标 |
| `tpos_inworld` | `TargetAxis` | 世界坐标 |

### 3.17 `TargetPosDown.msg`

下位机目标/位姿信息。

| 字段    | 类型          | 说明      |
| ------- | ------------- | --------- |
| `cs`  | `uint8`     | 状态字    |
| `pos` | `RobotAxis` | 位置/姿态 |

### 3.18 `ThrustCurve.msg`

单推进器推力曲线参数。

| 字段       | 类型        | 说明                       |
| ---------- | ----------- | -------------------------- |
| `num`    | `uint8`   | 推进器编号                 |
| `np_mid` | `float32` | 负向区间中点（PWM/归一化） |
| `np_ini` | `float32` | 负向区间起点               |
| `pp_ini` | `float32` | 正向区间起点               |
| `pp_mid` | `float32` | 正向区间中点               |
| `nt_end` | `float32` | 负推力终点                 |
| `nt_mid` | `float32` | 负推力中点                 |
| `pt_mid` | `float32` | 正推力中点                 |
| `pt_end` | `float32` | 正推力终点                 |

### 3.19 `ThrustCurves.msg`

六推进器曲线集合。

| 字段   | 类型            | 说明          |
| ------ | --------------- | ------------- |
| `m0` | `ThrustCurve` | 推进器 0 曲线 |
| `m1` | `ThrustCurve` | 推进器 1 曲线 |
| `m2` | `ThrustCurve` | 推进器 2 曲线 |
| `m3` | `ThrustCurve` | 推进器 3 曲线 |
| `m4` | `ThrustCurve` | 推进器 4 曲线 |
| `m5` | `ThrustCurve` | 推进器 5 曲线 |

### 3.20 `WorkState.msg`

工作状态字。

| 字段      | 类型      | 说明     |
| --------- | --------- | -------- |
| `state` | `uint8` | 工作状态 |

### 3.21 `Yolov8.msg`

视觉检测状态与目标数组。

| 字段        | 类型                 | 说明          |
| ----------- | -------------------- | ------------- |
| `state`   | `float32[13]`      | 13 路状态值   |
| `targets` | `TargetParams[13]` | 13 路目标参数 |

## 4. 服务定义详情

### 4.1 `DetectRequest.srv`

目标检测请求服务。

请求（Request）：

| 字段        | 类型                  | 说明                                    |
| ----------- | --------------------- | --------------------------------------- |
| `imagein` | `sensor_msgs/Image` | 输入图像                                |
| `target`  | `string`            | 目标类别/名称                           |
| `stero`   | `string`            | 双目/立体参数标识（原字段名 `stero`） |

响应（Response）：

| 字段  | 类型        | 说明     |
| ----- | ----------- | -------- |
| `s` | `uint8`   | 检测状态 |
| `x` | `float32` | 目标 X   |
| `y` | `float32` | 目标 Y   |
| `z` | `float32` | 目标 Z   |

## 5. 消息从属关系

下面从“消息嵌套组合关系”角度描述 `msg` 的从属关系。

```text
uv_msgs
├─ 基础原子类型
│  ├─ RobotAxis(x,y,z,rx,ry,rz)
│  ├─ PixelAxis(x,y)
│  ├─ TargetAxis(x,y,z)
│  ├─ PidParams(name,p,i,d,i_limit,output_limit)
│  ├─ ThrustCurve(num,np_mid,np_ini,pp_ini,pp_mid,nt_end,nt_mid,pt_mid,pt_end)
│  ├─ ServoSet(num,angle)
│  ├─ LedControllers(led0,led1)
│  ├─ MagnetController(state)
│  ├─ WorkState(state)
│  └─ CabinState(temp,hum,leak,voltage,servo[2])
│
├─ 组合类型
│  ├─ ImuData
│  │  ├─ mode
│  │  ├─ dvl
│  │  ├─ pos: RobotAxis
│  │  └─ spd: RobotAxis
│  ├─ MotorThrust(thrust[6])
│  ├─ PropellerThrust(thrust[6])
│  ├─ PidControllers
│  │  ├─ x,y,z,rx,ry,rz: PidParams
│  │  └─ vx,vy: PidParams
│  ├─ PidControllersState(x,y,z,rx,ry,rz,vy,vx)
│  ├─ RobotDeviceManager(leak,tem,hum,vol,magnet,led[2],angle[2])
│  ├─ TargetParams
│  │  ├─ tpos_inpic: PixelAxis
│  │  └─ tpos_inworld: TargetAxis
│  ├─ TargetPosDown
│  │  ├─ cs
│  │  └─ pos: RobotAxis
│  ├─ ThrustCurves
│  │  └─ m0..m5: ThrustCurve
│  ├─ Yolov8
│  │  ├─ state[13]
│  │  └─ targets[13]: TargetParams
│  └─ RobotMotionController
│     ├─ pos: RobotAxis
│     ├─ tpos_inbase: RobotAxis
│     ├─ tpos_inworld: RobotAxis
│     ├─ imu: ImuData
│     ├─ thrust: MotorThrust
│     └─ pidstate: PidControllersState
│
└─ 服务
	 └─ DetectRequest.srv
			├─ Request: imagein(sensor_msgs/Image), target, stero
			└─ Response: s, x, y, z
```

如果从“被复用层级”看，核心链路是：

- `RobotAxis` -> `ImuData` / `TargetPosDown` / `RobotMotionController`
- `PidParams` -> `PidControllers`
- `ThrustCurve` -> `ThrustCurves`
- `PixelAxis` + `TargetAxis` -> `TargetParams` -> `Yolov8`
- `ImuData` + `MotorThrust` + `PidControllersState` + `RobotAxis` -> `RobotMotionController`

## 6. 消息可见性与构建排查

当 `rqt` 或 `ros2 topic info` 提示无法解析 `uv_msgs/*` 或 `stonefish_ros2/*` 时，请先确认运行环境：

```bash
cd .
source conda activate ros2_jazzy_env
source /opt/ros/jazzy/setup.bash
source Cruise/install/setup.bash
```

检查关键接口：

```bash
ros2 interface list | grep -E 'uv_msgs/msg/RobotAxis|uv_msgs/msg/RobotMotionController|uv_msgs/msg/RobotDeviceManager|uv_msgs/msg/PidParams|uv_msgs/msg/PidControllers|uv_msgs/msg/TargetPosDown|stonefish_ros2/msg/DVL|stonefish_ros2/msg/ThrusterState'
```

如果缺失，重建消息包：

```bash
cd Cruise
source /opt/ros/jazzy/setup.bash
colcon build --packages-select uv_msgs stonefish_ros2 --symlink-install
source install/setup.bash
```

再验证一个运行中 topic 的类型解析：

```bash
ros2 topic info /openloop_thrust
```

预期应显示：`Type: uv_msgs/msg/RobotAxis`。

## 7. msg/srv 使用场景（基于代码简单检索）

检索范围：`Cruise/src` 下主要包（`uv_ai`、`uv_hm`、`uv_control_py`、`uv_vision`、`uv_launch_pkg`）。

### 7.1 核心运行场景

- 硬件管理与底层桥接：`uv_hm/uv_hm/uv_hmu.py`

  - 发布：`RobotMotionController`(`motion_controller`)、`RobotDeviceManager`(`device_manager`)、`PidControllers`(`pid_controllers`)、`ThrustCurves`(`curves`)
  - 订阅：`RobotAxis`(`openloop_thrust`)、`ServoSet`、`PidParams`、`PidControllersState`、`TargetPosDown`、`ImuData`(`dvl_set`)、`ThrustCurve`、`LedControllers`、`MagnetController`
  - 作用：上层指令下发到串口设备，同时回传综合状态。
- 自动任务控制：`uv_ai/uv_ai/uv_automaton.py`

  - 发布：`TargetPosDown`、`ServoSet`、`LedControllers`、`MagnetController`、`PidControllersState`、`RobotAxis`(`openloop_thrust`)
  - 订阅：`RobotMotionController`、`Yolov8`（`uv_detect_down`/`uv_detect_front`）、`PidParams`
  - 服务客户端：`DetectRequest`（调用 `uv_detect_srv`）
  - 作用：任务流程决策、视觉结果融合、运动目标下发。
- 视觉检测服务：`uv_ai/uv_ai/uv_detect_demo.py`

  - 发布：`Yolov8`（`uv_detect_down`、`uv_detect_front`）
  - 服务端：`DetectRequest`（`uv_detect_srv`）
  - 作用：检测结果发布 + 按请求返回三维目标坐标。
- 仿真桥：`uv_hm/uv_hm/uv_sim_bridge.py`

  - 订阅：`RobotAxis`(`openloop_thrust`)
  - 发布：`RobotMotionController`、`RobotDeviceManager`
  - 作用：将仿真传感器/里程计转换为项目内部统一消息。
- Web 控制面板：`uv_hm/uv_hm/uv_web_pannel.py`

  - 订阅：`CabinState`、`PropellerThrust`、`RobotAxis`、`WorkState`
  - 发布：`RobotAxis`(`openloop_thrust`)（以及代码中定义的 `servo_control`、`work_state`）
  - 作用：可视化状态 + 远程控制输入桥接。

### 7.2 其他检索出现

- 参数工具类使用：
  - `uv_control_py/uv_control_py/Pid.py` 使用 `PidParams`、`PidControllers`
  - `uv_control_py/uv_control_py/Curve.py` 使用 `ThrustCurve`、`ThrustCurves`
- 当前代码中直接引用较少或未检索到明确业务文件的消息：`MotorThrust`、`PixelAxis`、`TargetAxis`、`TargetParams`
  - 这类消息主要作为组合消息的子结构出现（例如 `RobotMotionController`、`Yolov8` 内部字段）。
    ## 待完成展望
