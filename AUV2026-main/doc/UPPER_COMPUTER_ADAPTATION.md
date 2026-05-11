# ZIT6 上位机适配指南 (v1.0)

本文档为上位机（AGX/Jetson）从旧协议（AUV2026 串口直连）迁移到新协议（micro-ROS）的完整适配指南。

---

## 1. 系统架构对比

### 旧系统（AUV2026）

```
AGX (Python)  ──FA AF..FB BF──>  H750 (运动控制)
                  ttyUSB1            串口直连

AGX (Python)  ──FA AF..FB BF──>  F407 (设备管理)
                  ttyUSB0            串口直连
```

- 上位机直接用二进制帧协议（`0xFA 0xAF ... 0xFB 0xBF`）与两个 STM32 通信
- 消息类型：`uv_msgs`（21个自定义消息）
- 桥接节点：`uv_hmu.py`，手动拼装字节流

### 新系统（ZIT6 micro-ROS）

```
AGX (ROS2)  ──micro-ROS serial──>  STM32H743 (单板)
              ttyUSBx (USART2)         统一固件
```

- 上位机使用标准 ROS2 话题通信，micro-ROS agent 负责串口传输
- 消息类型：`zit6_interfaces`（4个自定义消息）
- 无需手写字节流，直接 pub/sub ROS2 消息

### 关键差异

| 项目 | 旧系统 (AUV2026) | 新系统 (ZIT6) |
|------|------------------|---------------|
| 通信方式 | 手动二进制帧 `FA AF..FB BF` | micro-ROS 标准话题 |
| 串口数量 | 2个（H750 + F407） | 1个（H743 统一） |
| 消息包 | `uv_msgs` (21个) | `zit6_interfaces` (4个) |
| 解锁机制 | 无 | 必须通过心跳解锁 |
| 控制模式 | 直接发推力/位置 | 通过 `control_key` 选择模式 |
| PID 管理 | JSON 文件 + 串口下行 | ROS2 话题在线调参 |
| 推力曲线 | 固件内部存储 | 由下游运动控制板处理 |

---

## 2. 前置准备

### 2.1 安装 zit6_interfaces

将 `zit6_interfaces` 包添加到上位机的 ROS2 工作空间：

```bash
# 方式一：符号链接（开发阶段推荐）
cd ~/AUV2026/src
ln -s /path/to/AUV_zit6_cmake/zit6_interfaces zit6_interfaces

# 方式二：复制
cp -r /path/to/AUV_zit6_cmake/zit6_interfaces ~/AUV2026/src/

# 编译
cd ~/AUV2026
colcon build --packages-select zit6_interfaces
source install/setup.bash
```

### 2.2 启动 micro-ROS Agent

新系统不再需要上位机直接操作串口。micro-ROS agent 负责 AGX 与 STM32 之间的串口桥接：

```bash
# 安装 micro-ROS agent（如未安装）
sudo apt install ros-${ROS_DISTRO}-micro-ros-agent

# 启动 agent（假设串口为 /dev/ttyUSB0，波特率 115200）
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

Agent 启动后，STM32 的所有话题会自动出现在 ROS2 网络中。可用 `ros2 topic list` 验证：

```bash
ros2 topic list
# 预期输出：
# /zit6/cmd/agxhbt
# /zit6/cmd/ins
# /zit6/cmd/light
# /zit6/cmd/pid
# /zit6/cmd/servo
# /zit6/cmd/setpoint
# /zit6/state/pid_status
# /zit6/state/pos
# /zit6/state/status
# /zit6/state/thr
# /zit6/state/vel
# /zit6/state/zithbt
```

---

## 3. 消息类型映射

### 3.1 控制指令映射

#### 3.1.1 目标位置/速度/推力：`TargetPosDown` → `ZitSetpoint`

旧系统通过 `target_pos_down` 话题发送 `TargetPosDown`（含 `cs` 坐标系标志 + 6DOF 目标）。
新系统通过 `/zit6/cmd/setpoint` 话题发送 `ZitSetpoint`（含 `control_key` + `type_mask` + 4DOF 目标）。

**字段映射：**

| 旧字段 (TargetPosDown) | 新字段 (ZitSetpoint) | 说明 |
|------------------------|---------------------|------|
| `cs` (坐标系) | `control_key` Bit4 | 旧: 0=世界系, 1=机体系 → 新: Bit4=0 世界系, Bit4=1 机体系 |
| `pos.x` | `x` | X轴目标值 |
| `pos.y` | `y` | Y轴目标值 |
| `pos.z` | `z` | Z轴目标值（深度） |
| `pos.rx` | — | 旧系统有 roll，新系统不支持 |
| `pos.ry` | — | 旧系统有 pitch，新系统不支持 |
| `pos.rz` | `yaw` | 偏航目标值 |
| — | `control_key` 低2位 | 新增：必须指定控制模式 (0=位置, 1=速度, 2=推力) |
| — | `type_mask` | 新增：轴掩码，控制哪些轴生效 |

**`control_key` 编码表：**

| Bit | 含义 | 值 |
|-----|------|-----|
| [1:0] | 控制模式 | 0=位置环, 1=速度环, 2=推力环 |
| [4] | 坐标系 | 0=世界系(NED), 1=机体系 |
| [5] | 增量模式 | 0=绝对量, 1=相对量(叠加到当前位置) |

**`type_mask` 编码表（仅位置模式有效）：**

| Bit | 轴 | 值 |
|-----|-----|-----|
| 0 | X | 0x01 |
| 1 | Y | 0x02 |
| 2 | Z | 0x04 |
| 3 | Yaw | 0x08 |

示例：`type_mask = 0x01 | 0x02 = 0x03` 表示仅控制 X 和 Y 轴。

**代码示例 — 位置模式（世界系，全轴）：**

```python
from zit6_interfaces.msg import ZitSetpoint

msg = ZitSetpoint()
msg.control_key = 0x00       # 位置模式(0) + 世界系(Bit4=0)
msg.type_mask = 0x0F         # 全轴生效 (X|Y|Z|Yaw)
msg.x = 1.0                  # 目标 X 位置 (m)
msg.y = 0.0                  # 目标 Y 位置 (m)
msg.z = -2.0                 # 目标深度 (m, 负值=下潜)
msg.yaw = 0.0                # 目标航向 (rad)
msg.seq = 123                # 序列号（可选）
```

**代码示例 — 速度模式（机体系）：**

```python
msg = ZitSetpoint()
msg.control_key = 0x01 | 0x10  # 速度模式(1) + 机体系(Bit4)
msg.type_mask = 0x0F
msg.x = 0.3                   # 前进速度 (m/s)
msg.y = 0.0                   # 横移速度 (m/s)
msg.z = 0.0                   # 垂直速度 (m/s)
msg.yaw = 0.1                 # 偏航角速度 (rad/s)
```

**代码示例 — 推力模式：**

```python
msg = ZitSetpoint()
msg.control_key = 0x02 | 0x10  # 推力模式(2) + 机体系(Bit4)
msg.type_mask = 0x0F
msg.x = 0.5                   # X方向归一化推力 [-1, 1]
msg.y = 0.0
msg.z = 0.3                   # Z方向推力
msg.yaw = 0.0                 # 偏航力矩
```

#### 3.1.2 开环推力：`openloop_thrust` (RobotAxis) → `/zit6/cmd/setpoint` (FORCE 模式)

旧系统的 `openloop_thrust` 话题直接发送 6DOF 推力。新系统需要通过 `ZitSetpoint` 的推力模式实现。

| 旧字段 | 新字段 | 说明 |
|--------|--------|------|
| `RobotAxis.x` | `ZitSetpoint.x` | X推力 |
| `RobotAxis.y` | `ZitSetpoint.y` | Y推力 |
| `RobotAxis.z` | `ZitSetpoint.z` | Z推力 |
| `RobotAxis.rz` | `ZitSetpoint.yaw` | 偏航力矩 |
| `RobotAxis.rx` | — | 不支持 roll 力矩 |
| `RobotAxis.ry` | — | 不支持 pitch 力矩 |

```python
msg = ZitSetpoint()
msg.control_key = 0x02 | 0x10  # 推力模式 + 机体系
msg.type_mask = 0x0F
msg.x = data.x
msg.y = data.y
msg.z = data.z
msg.yaw = data.rz
```

#### 3.1.3 PID 参数设置：`PidParams` / `PidControllers` → `ZitPid`

旧系统通过 JSON 文件 + 串口下行。新系统通过 `/zit6/cmd/pid` 话题在线设置。

**旧系统 PID 结构 (PidControllers)：**
- 8个独立 PID 通道：x, y, z, rx, ry, rz, vx, vy
- 每个通道有：p, i, d, i_limit, output_limit

**新系统 PID 结构 (ZitPid)：**
- 4个轴：0=X, 1=Y, 2=Z, 3=Yaw
- 每个轴有 2 个环：位置环 + 速度环
- 位置环参数：kp, max_v, max_a
- 速度环参数：kp, ki, kd, i_limit, out_limit
- **负值表示不更新**（保留当前值）

**映射关系：**

| 旧通道 | 新轴 | 新环 | 字段映射 |
|--------|------|------|----------|
| `x` (位置P) | axis=0 | 位置环 | `kp` ← `p` |
| `y` (位置P) | axis=1 | 位置环 | `kp` ← `p` |
| `z` (位置P) | axis=2 | 位置环 | `kp` ← `p` |
| `rz` (位置P) | axis=3 | 位置环 | `kp` ← `p` |
| `vx` (速度PID) | axis=0 | 速度环 | `kp/ki/kd` ← `p/i/d` |
| `vy` (速度PID) | axis=1 | 速度环 | `kp/ki/kd` ← `p/i/d` |
| — | axis=2 | 速度环 | `kp/ki/kd` ← 独立设置 |
| — | axis=3 | 速度环 | `kp/ki/kd` ← 独立设置 |

**代码示例 — 设置 X 轴速度环 PID：**

```python
from zit6_interfaces.msg import ZitPid

msg = ZitPid()
msg.axis = 0          # X轴
msg.is_pos_ring = False  # 速度环
msg.kp = 0.05
msg.ki = 0.01
msg.kd = 0.02
msg.i_limit = 1.0
msg.out_limit = 1.0
msg.max_v = -1.0      # 负值=不更新（仅位置环有效）
msg.max_a = -1.0      # 负值=不更新
```

**代码示例 — 设置 Z 轴位置环参数：**

```python
msg = ZitPid()
msg.axis = 2          # Z轴
msg.is_pos_ring = True   # 位置环
msg.kp = 0.02
msg.ki = -1.0         # 不更新
msg.kd = -1.0         # 不更新
msg.i_limit = -1.0    # 不更新
msg.out_limit = -1.0  # 不更新
msg.max_v = 0.3       # 最大速度限制
msg.max_a = 0.1       # 最大加速度限制
```

#### 3.1.4 舵机控制：`ServoSet` → `/zit6/cmd/servo` (std_msgs/Float32)

旧系统发 `ServoSet(num, angle)` 到 F407。新系统直接发 `Float32` 到 H743。

| 旧 | 新 | 说明 |
|----|-----|------|
| `ServoSet.num` | — | 新系统可能仅支持单舵机，需确认固件端 |
| `ServoSet.angle` (0~1) | `Float32.data` | 角度值直接传递 |

```python
from std_msgs.msg import Float32

msg = Float32()
msg.data = 0.5  # 舵机角度 (0~1)
```

#### 3.1.5 灯光控制：`LedControllers` → `/zit6/cmd/light` (std_msgs/UInt8)

| 旧 | 新 | 说明 |
|----|-----|------|
| `LedControllers.led0` + `led1` | `UInt8.data` | 新系统编码方式需确认固件端 |

```python
from std_msgs.msg import UInt8

msg = UInt8()
msg.data = 1  # 灯光状态
```

#### 3.1.6 DVL 控制：`ImuData` → `/zit6/cmd/ins` (std_msgs/UInt8)

旧系统通过 `dvl_set` 话题发 `ImuData.dvl`。新系统通过 `/zit6/cmd/ins` 发送命令码。

| 旧 `ImuData.dvl` 值 | 新 `UInt8.data` 值 | 命令 |
|----------------------|---------------------|------|
| 0x00 (DVL未上传) | 2 | DVL 关闭 |
| 0x01 (数据无效) | — | — |
| 0x02 (数据有效) | 1 | DVL 开启 |
| — | 3 | 重启 INS |
| — | 4 | 重置位置 |
| — | 5 | 设置初始位置 |

```python
from std_msgs.msg import UInt8

# 开启 DVL
msg = UInt8()
msg.data = 1

# 重启 INS
msg = UInt8()
msg.data = 3
```

---

### 3.2 状态反馈映射

#### 3.2.1 运动状态：`/zit6/state/status` (ZitStatus) → `RobotMotionController`

| 新字段 (ZitStatus) | 旧字段 (RobotMotionController) | 说明 |
|--------------------|---------------------------------|------|
| `is_armed` | — | 新增，旧系统无此概念 |
| `arm_mode` | — | 新增 |
| `control_level` | — | 新增 (0=无, 1=位置, 2=速度, 3=推力) |
| `ins_state` | `imu.mode` | 值域不同，需映射（见下表） |
| `navigation_ready` | — | 新增 |
| `forces[0..3]` | `thrust.thrust[0..3]` | 推力数据（新系统4DOF，旧系统6DOF） |
| `cycle_time_ms` | — | 新增，控制循环耗时 |
| `battery_voltage` | `DeviceManager.vol` | 电压（当前固件硬编码为0） |
| `error_flags` | — | 新增（当前固件硬编码为0） |

**INS 状态映射：**

| 新 `ins_state` | 含义 | 旧 `imu.mode` |
|-----------------|------|----------------|
| 0 | 待机 | 0x00 |
| 1 | 粗对准 | 0x01 |
| 2 | 精对准 | 0x02 |
| 3 | SINS/GPS/DVL | 0x04 |
| 4 | SINS/DVL | 0x04 (与3共用) |
| 5 | MRU | 0x05 |

**代码示例 — 状态转换：**

```python
def zit_status_to_motion_controller(status):
    mc = RobotMotionController()
    mc.pos.x = 0.0   # 需从 /zit6/state/pos 获取
    mc.pos.y = 0.0
    mc.pos.z = 0.0
    mc.pos.rz = 0.0

    # INS 状态映射
    ins_map = {0: 0x00, 1: 0x01, 2: 0x02, 3: 0x04, 4: 0x04, 5: 0x05}
    mc.imu.mode = ins_map.get(status.ins_state, 0x00)

    # 推力映射 (4DOF → 6DOF)
    mc.thrust.thrust[0] = status.forces[0]  # Fx
    mc.thrust.thrust[1] = status.forces[1]  # Fy
    mc.thrust.thrust[2] = status.forces[2]  # Fz
    mc.thrust.thrust[3] = 0.0               # 无 roll
    mc.thrust.thrust[4] = 0.0               # 无 pitch
    mc.thrust.thrust[5] = status.forces[3]  # Mz → yaw

    return mc
```

#### 3.2.2 位置反馈：`/zit6/state/pos` (Float32MultiArray) → `RobotMotionController.pos`

数据格式：`[x, y, z, yaw]`，单位 `[m, m, m, rad]`，NED 世界坐标系。

```python
# 订阅 /zit6/state/pos
def pos_callback(msg):
    mc.pos.x = msg.data[0]    # 北向位置 (m)
    mc.pos.y = msg.data[1]    # 东向位置 (m)
    mc.pos.z = msg.data[2]    # 深度 (m, 正值=下潜)
    mc.pos.rz = msg.data[3]   # 航向 (rad)
```

#### 3.2.3 速度反馈：`/zit6/state/vel` (Float32MultiArray) → `RobotMotionController.imu.spd`

数据格式：`[vx, vy, vz, vyaw]`，单位 `[m/s, m/s, m/s, rad/s]`，**机体系**。

```python
def vel_callback(msg):
    mc.imu.spd.x = msg.data[0]   # 纵向速度 (m/s)
    mc.imu.spd.y = msg.data[1]   # 横向速度 (m/s)
    mc.imu.spd.z = msg.data[2]   # 垂直速度 (m/s)
    mc.imu.spd.rz = msg.data[3]  # 偏航角速度 (rad/s)
```

#### 3.2.4 推力反馈：`/zit6/state/thr` (Float32MultiArray)

数据格式：`[Fx, Fy, Fz, Mz]`，归一化推力 `[-1, 1]` 或牛顿。

```python
def thr_callback(msg):
    mc.thrust.thrust[0] = msg.data[0]  # Fx
    mc.thrust.thrust[1] = msg.data[1]  # Fy
    mc.thrust.thrust[2] = msg.data[2]  # Fz
    mc.thrust.thrust[5] = msg.data[3]  # Mz → yaw
```

#### 3.2.5 PID 状态：`/zit6/state/pid_status` (ZitPidStatus) → `PidControllers`

1Hz 回传全轴 PID 参数。数据格式为 4 元素数组，索引 0-3 对应 X/Y/Z/Yaw。

```python
def pid_status_callback(msg):
    pid = PidControllers()
    # 位置环
    pid.x.p = msg.pos_kp[0]
    pid.y.p = msg.pos_kp[1]
    pid.z.p = msg.pos_kp[2]
    pid.rz.p = msg.pos_kp[3]
    # 速度环
    pid.vx.p = msg.vel_kp[0]
    pid.vx.i = msg.vel_ki[0]
    pid.vx.d = msg.vel_kd[0]
    pid.vy.p = msg.vel_kp[1]
    pid.vy.i = msg.vel_ki[1]
    pid.vy.d = msg.vel_kd[1]
    # ... Z 和 Yaw 类似
```

#### 3.2.6 心跳：`/zit6/state/zithbt` (UInt32)

1Hz，值为 STM32 的 `HAL_GetTick()` 毫秒计数。用于确认固件在线。

---

## 4. 解锁（ARM）协议 — 全新流程

**这是新系统最大的变化。** 旧系统没有解锁概念，发送推力指令即可直接输出。新系统必须先完成解锁流程，否则所有控制指令被静默丢弃。

### 4.1 解锁流程

```
上电 → 固件初始化 → 上位机开始发心跳 → 等待 ≥10次 + ≥1秒 → 解锁成功 → 可发送控制指令
                                            ↓
                                     心跳不能停！
                                     500ms 超时 → 自动锁定
```

### 4.2 心跳发送

话题：`/zit6/cmd/agxhbt`
类型：`std_msgs/UInt32`
频率：**≥10Hz**（推荐 20Hz）
`data` 字段含义：
- `3`：遥控模式（绕过导航检查，调试/测试用）
- 其他非零值：正常模式（需要 INS 就绪）

```python
from std_msgs.msg import UInt32
import threading

class ArmHeartbeat:
    def __init__(self, node):
        self.pub = node.create_publisher(UInt32, '/zit6/cmd/agxhbt', 10)
        self.mode = 3  # 默认遥控模式（调试用）

    def start(self):
        """启动心跳线程，20Hz"""
        def _loop():
            msg = UInt32()
            msg.data = self.mode
            while rclpy.ok():
                self.pub.publish(msg)
                time.sleep(0.05)  # 20Hz
        self.thread = threading.Thread(target=_loop, daemon=True)
        self.thread.start()

    def set_mode(self, mode):
        """mode=3: 遥控模式(跳过导航检查), 其他: 正常模式"""
        self.mode = mode
```

### 4.3 解锁条件

| 条件 | 说明 |
|------|------|
| 心跳次数 ≥ 10 | 至少收到 10 次心跳 |
| 持续时间 ≥ 1秒 | 从第一次心跳起至少 1 秒 |
| `data == 3` **或** INS 就绪 | 遥控模式跳过导航检查 |

### 4.4 锁定触发条件

| 条件 | 结果 |
|------|------|
| 心跳停止 > 500ms | 自动锁定，推力归零 |
| 心跳停止 > 1000ms（未锁定时） | 心跳计数清零，需重新累积 |

### 4.5 状态监控

通过 `/zit6/state/status` 话题的 `is_armed` 字段确认解锁状态：

```python
from zit6_interfaces.msg import ZitStatus

def status_callback(msg):
    if msg.is_armed:
        # 已解锁，可发送控制指令
        pass
    else:
        # 未解锁，检查 error_flags 和 navigation_ready
        if msg.error_flags != 0:
            node.get_logger().warn(f"错误标志: {msg.error_flags}")
        if not msg.navigation_ready:
            node.get_logger().warn("导航未就绪，INS状态: %d" % msg.ins_state)
```

### 4.6 建议的上位机解锁状态机

```
[等待固件] → [发送心跳] → [等待解锁] → [已解锁/运行中]
     ↑              ↑            ↑              ↓
     └──────────────┴────────────┴── [心跳超时/错误] ─┘
```

```python
class ArmStateMachine:
    WAIT_BOOT = 0
    SENDING_HBT = 1
    WAIT_ARM = 2
    ARMED = 3

    def __init__(self):
        self.state = self.WAIT_BOOT
        self.hbt_count = 0
        self.first_hbt_time = None

    def update(self, status_msg, now):
        if self.state == self.WAIT_BOOT:
            # 等待收到第一个 status 消息
            self.state = self.SENDING_HBT

        elif self.state == self.SENDING_HBT:
            self.hbt_count += 1
            if self.first_hbt_time is None:
                self.first_hbt_time = now
            if self.hbt_count >= 10 and (now - self.first_hbt_time) >= 1.0:
                self.state = self.WAIT_ARM

        elif self.state == self.WAIT_ARM:
            if status_msg.is_armed:
                self.state = self.ARMED

        elif self.state == self.ARMED:
            if not status_msg.is_armed:
                # 被固件强制锁定，重新开始
                self.hbt_count = 0
                self.first_hbt_time = None
                self.state = self.SENDING_HBT
```

---

## 5. 桥接节点设计

### 5.1 方案选择

有两种适配方案：

**方案 A：直接替换（推荐）**
- 修改 `uv_hmu.py`，去掉串口通信，改为 ROS2 话题 pub/sub
- 上游节点（`uv_automaton` 等）无需修改，继续使用 `uv_msgs` 话题
- `uv_hmu` 负责 `uv_msgs` ↔ `zit6_interfaces` 的双向转换

**方案 B：新建桥接节点**
- 创建新的 `uv_zit6_bridge.py` 节点
- 与旧 `uv_hmu.py` 并存，通过 launch 文件切换
- 更安全，可逐步迁移

### 5.2 桥接节点结构（方案 A）

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time
import threading

# zit6 消息
from zit6_interfaces.msg import ZitSetpoint, ZitStatus, ZitPid, ZitPidStatus
from std_msgs.msg import UInt32, UInt8, Float32, Float32MultiArray

# uv_msgs 消息（供上游节点使用）
from uv_msgs.msg import (
    RobotMotionController, RobotDeviceManager, RobotAxis,
    PidParams, PidControllers, PidControllersState,
    ServoSet, TargetPosDown, ImuData, LedControllers,
    MotorThrust
)


class Zit6Bridge(Node):
    """ZIT6 micro-ROS 桥接节点

    职责：
    1. 接收上游 uv_msgs 话题，转换为 zit6_interfaces 话题发送给固件
    2. 接收固件 zit6_interfaces 状态反馈，转换为 uv_msgs 话题发布给上游
    3. 管理解锁心跳
    """

    def __init__(self):
        super().__init__('zit6_bridge')

        # ========== 解锁心跳 ==========
        self.hbt_pub = self.create_publisher(UInt32, '/zit6/cmd/agxhbt', 10)
        self.hbt_mode = 3  # 3=遥控模式(跳过导航检查)
        self._start_heartbeat()

        # ========== 发布到固件 ==========
        self.setpoint_pub = self.create_publisher(
            ZitSetpoint, '/zit6/cmd/setpoint', 10)
        self.pid_pub = self.create_publisher(
            ZitPid, '/zit6/cmd/pid', 10)
        self.ins_pub = self.create_publisher(
            UInt8, '/zit6/cmd/ins', 10)
        self.servo_pub = self.create_publisher(
            Float32, '/zit6/cmd/servo', 10)
        self.light_pub = self.create_publisher(
            UInt8, '/zit6/cmd/light', 10)

        # ========== 订阅固件状态 ==========
        self.create_subscription(
            ZitStatus, '/zit6/state/status', self._on_status, 10)
        self.create_subscription(
            Float32MultiArray, '/zit6/state/pos', self._on_pos, 10)
        self.create_subscription(
            Float32MultiArray, '/zit6/state/vel', self._on_vel, 10)
        self.create_subscription(
            Float32MultiArray, '/zit6/state/thr', self._on_thr, 10)
        self.create_subscription(
            ZitPidStatus, '/zit6/state/pid_status', self._on_pid_status, 10)

        # ========== 发布到上游（uv_msgs 话题） ==========
        self.mc_pub = self.create_publisher(
            RobotMotionController, 'motion_controller', 10)
        self.dm_pub = self.create_publisher(
            RobotDeviceManager, 'device_manager', 10)
        self.pid_ctrl_pub = self.create_publisher(
            PidControllers, 'pid_controllers', 10)

        # ========== 订阅上游（uv_msgs 话题） ==========
        self.create_subscription(
            TargetPosDown, 'target_pos_down', self._on_target_pos, 10)
        self.create_subscription(
            RobotAxis, 'openloop_thrust', self._on_openloop_thrust, 10)
        self.create_subscription(
            PidParams, 'pid_params_set', self._on_pid_params, 10)
        self.create_subscription(
            ServoSet, 'servo_control', self._on_servo, 10)
        self.create_subscription(
            LedControllers, 'led_controllers', self._on_led, 10)
        self.create_subscription(
            ImuData, 'dvl_set', self._on_dvl, 10)

        # 状态缓存
        self._mc = RobotMotionController()
        self._dm = RobotDeviceManager()

        self.get_logger().info('ZIT6 桥接节点已启动')

    # ==================== 心跳 ====================

    def _start_heartbeat(self):
        def _loop():
            msg = UInt32()
            msg.data = self.hbt_mode
            while rclpy.ok():
                self.hbt_pub.publish(msg)
                time.sleep(0.05)  # 20Hz
        threading.Thread(target=_loop, daemon=True).start()

    # ==================== 上游 → 固件 ====================

    def _on_target_pos(self, data: TargetPosDown):
        """TargetPosDown → ZitSetpoint (位置模式)"""
        msg = ZitSetpoint()
        # cs=0 世界系, cs=1 机体系
        msg.control_key = 0x00 | (0x10 if data.cs else 0x00)
        msg.type_mask = 0x0F  # 全轴
        msg.x = data.pos.x
        msg.y = data.pos.y
        msg.z = data.pos.z
        msg.yaw = data.pos.rz
        self.setpoint_pub.publish(msg)

    def _on_openloop_thrust(self, data: RobotAxis):
        """RobotAxis → ZitSetpoint (推力模式)"""
        msg = ZitSetpoint()
        msg.control_key = 0x02 | 0x10  # 推力模式 + 机体系
        msg.type_mask = 0x0F
        msg.x = data.x
        msg.y = data.y
        msg.z = data.z
        msg.yaw = data.rz
        self.setpoint_pub.publish(msg)

    def _on_pid_params(self, data: PidParams):
        """PidParams → ZitPid"""
        axis_map = {'x': 0, 'y': 1, 'z': 2, 'rz': 3,
                    'vx': 0, 'vy': 1, 'vz': 2, 'vyaw': 3}
        is_vel = data.name.startswith('v')

        msg = ZitPid()
        msg.axis = axis_map.get(data.name, 0)
        msg.is_pos_ring = not is_vel
        msg.kp = data.p
        msg.ki = data.i if is_vel else -1.0  # 位置环无 ki
        msg.kd = data.d if is_vel else -1.0  # 位置环无 kd
        msg.i_limit = data.i_limit if is_vel else -1.0
        msg.out_limit = data.output_limit
        msg.max_v = -1.0  # 不更新
        msg.max_a = -1.0  # 不更新
        self.pid_pub.publish(msg)

    def _on_servo(self, data: ServoSet):
        """ServoSet → Float32"""
        msg = Float32()
        msg.data = max(0.0, min(1.0, data.angle))
        self.servo_pub.publish(msg)

    def _on_led(self, data: LedControllers):
        """LedControllers → UInt8"""
        msg = UInt8()
        msg.data = data.led0  # 简化映射，需根据实际需求调整
        self.light_pub.publish(msg)

    def _on_dvl(self, data: ImuData):
        """ImuData → UInt8 (INS 命令)"""
        msg = UInt8()
        msg.data = 1 if data.dvl == 0x02 else 2  # 有效=开, 其他=关
        self.ins_pub.publish(msg)

    # ==================== 固件 → 上游 ====================

    def _on_status(self, s: ZitStatus):
        """ZitStatus → RobotMotionController + RobotDeviceManager"""
        ins_map = {0: 0x00, 1: 0x01, 2: 0x02, 3: 0x04, 4: 0x04, 5: 0x05}
        self._mc.imu.mode = ins_map.get(s.ins_state, 0x00)
        self._mc.thrust.thrust[0] = s.forces[0]
        self._mc.thrust.thrust[1] = s.forces[1]
        self._mc.thrust.thrust[2] = s.forces[2]
        self._mc.thrust.thrust[5] = s.forces[3]
        self._dm.vol = s.battery_voltage
        self.mc_pub.publish(self._mc)
        self.dm_pub.publish(self._dm)

    def _on_pos(self, msg: Float32MultiArray):
        """pos → RobotMotionController.pos"""
        self._mc.pos.x = msg.data[0]
        self._mc.pos.y = msg.data[1]
        self._mc.pos.z = msg.data[2]
        self._mc.pos.rz = msg.data[3]

    def _on_vel(self, msg: Float32MultiArray):
        """vel → RobotMotionController.imu.spd"""
        self._mc.imu.spd.x = msg.data[0]
        self._mc.imu.spd.y = msg.data[1]
        self._mc.imu.spd.z = msg.data[2]
        self._mc.imu.spd.rz = msg.data[3]

    def _on_thr(self, msg: Float32MultiArray):
        """thr → RobotMotionController.thrust"""
        self._mc.thrust.thrust[0] = msg.data[0]
        self._mc.thrust.thrust[1] = msg.data[1]
        self._mc.thrust.thrust[2] = msg.data[2]
        self._mc.thrust.thrust[5] = msg.data[3]

    def _on_pid_status(self, msg: ZitPidStatus):
        """ZitPidStatus → PidControllers"""
        pid = PidControllers()
        pid.x.p = msg.pos_kp[0]
        pid.y.p = msg.pos_kp[1]
        pid.z.p = msg.pos_kp[2]
        pid.rz.p = msg.pos_kp[3]
        pid.vx.p = msg.vel_kp[0]
        pid.vx.i = msg.vel_ki[0]
        pid.vx.d = msg.vel_kd[0]
        pid.vy.p = msg.vel_kp[1]
        pid.vy.i = msg.vel_ki[1]
        pid.vy.d = msg.vel_kd[1]
        self.pid_ctrl_pub.publish(pid)


def main(args=None):
    rclpy.init(args=args)
    node = Zit6Bridge()
    rclpy.spin(node)
    rclpy.shutdown()
```

### 5.3 QoS 注意事项

| 话题 | 建议 QoS | 说明 |
|------|----------|------|
| `/zit6/cmd/setpoint` | reliable, depth=10 | 控制指令不能丢 |
| `/zit6/cmd/agxhbt` | reliable, depth=10 | 心跳不能丢 |
| `/zit6/state/status` | reliable, depth=10 | 安全关键状态 |
| `/zit6/state/pos` | best_effort, depth=5 | 高频位置数据 |
| `/zit6/state/vel` | best_effort, depth=5 | 高频速度数据 |
| `/zit6/state/thr` | best_effort, depth=5 | 推力反馈 |

---

## 6. 默认 PID 参数

新系统固件内置默认 PID 参数（所有 4 轴相同）：

| 参数 | 位置环 | 速度环 |
|------|--------|--------|
| kp | 0.01 | 0.01 |
| ki | 0.0 | 0.005 |
| kd | 0.0 | 0.01 |
| i_limit | 1.0 | 1.0 |
| output_limit | 1.0 | 1.0 |
| max_v | 0.5 m/s | — |
| max_a | 0.2 m/s² | — |

**这些值非常保守**，仅作为安全初始值。实际使用前必须根据平台特性重新调参。

---

## 7. 控制模式详解

### 7.1 位置模式 (control_key & 0x03 == 0)

- 输入：目标位置 (m) 和航向 (rad)
- 控制链：目标位置 → 梯形规划器 → 位置P → 速度PID → 推力输出
- 坐标系：世界系(NED) 或 机体系（通过 Bit4 选择）
- 需要 `navigation_ready == true`
- `type_mask` 控制哪些轴生效

### 7.2 速度模式 (control_key & 0x03 == 1)

- 输入：目标速度 (m/s) 和角速度 (rad/s)
- 控制链：目标速度 → 速度PID → 推力输出
- 坐标系：世界系 或 机体系
- 需要 `navigation_ready == true`
- `type_mask` 不生效，4 轴全部应用

### 7.3 推力模式 (control_key & 0x03 == 2)

- 输入：归一化推力 [-1, 1] 或牛顿
- 控制链：直接输出到推进器
- 坐标系：世界系 或 机体系
- **不需要** `navigation_ready`
- `type_mask` 不生效，4 轴全部应用

### 7.4 模式切换行为

| 切换 | 行为 |
|------|------|
| 任意 → 位置 | 规划器对齐到当前状态；Z轴速度环积分预加载（防深度突变） |
| 任意 → 速度 | 规划器对齐；速度环积分清零 |
| 任意 → 推力 | 直接切换，无平滑过渡 |
| 任意 → NONE | 推力归零 |

---

## 8. 坐标系

### 8.1 世界系 (NED)

- X：北（North）
- Y：东（East）
- Z：下（Down，深度正值）
- Yaw：0=北，顺时针为正

### 8.2 机体系

- X：前进（Surge）
- Y：右移（Sway）
- Z：下潜（Heave）
- Yaw：顺时针旋转

### 8.3 变换公式

```
Body_X =  World_X * cos(yaw) + World_Y * sin(yaw)
Body_Y = -World_X * sin(yaw) + World_Y * cos(yaw)
```

---

## 9. 注意事项与常见问题

### 9.1 已知限制

| 项目 | 说明 |
|------|------|
| `battery_voltage` | 固件硬编码为 0.0，不反映实际电压 |
| `error_flags` | 固件硬编码为 0，不反映实际错误 |
| 推力通道数 | 新系统 4DOF [Fx,Fy,Fz,Mz]，旧系统 6DOF。Roll/Pitch 不可控 |
| 推力曲线 | 由下游运动控制板处理，新固件不管理推力曲线 |
| 磁铁控制 | 新固件未实现 `/zit6/cmd/magnet` 话题 |
| `target_speed_down` | 新系统无速度目标话题，需通过 setpoint 速度模式替代 |

### 9.2 调试命令

```bash
# 查看所有 zit6 话题
ros2 topic list | grep zit6

# 监控状态
ros2 topic echo /zit6/state/status

# 监控位置
ros2 topic echo /zit6/state/pos

# 手动发送心跳（测试解锁）
ros2 topic pub --rate 20 /zit6/cmd/agxhbt std_msgs/UInt32 "{data: 3}"

# 手动发送位置目标
ros2 topic pub /zit6/cmd/setpoint zit6_interfaces/ZitSetpoint \
  "{control_key: 0, type_mask: 15, x: 0.0, y: 0.0, z: -1.0, yaw: 0.0}"

# 手动发送推力
ros2 topic pub /zit6/cmd/setpoint zit6_interfaces/ZitSetpoint \
  "{control_key: 18, type_mask: 15, x: 0.3, y: 0.0, z: 0.0, yaw: 0.0}"

# 查看话题频率
ros2 topic hz /zit6/state/status
ros2 topic hz /zit6/state/pos
```

### 9.3 迁移检查清单

- [ ] 安装 `zit6_interfaces` 到上位机工作空间并编译通过
- [ ] 启动 micro-ROS agent，确认 `ros2 topic list` 可见所有 zit6 话题
- [ ] 实现心跳发送（≥10Hz），确认 `is_armed` 变为 true
- [ ] 替换 `target_pos_down` 为 `ZitSetpoint` 位置模式
- [ ] 替换 `openloop_thrust` 为 `ZitSetpoint` 推力模式
- [ ] 替换 PID 参数下发为 `ZitPid` 话题
- [ ] 替换 DVL 控制为 `/zit6/cmd/ins` 话题
- [ ] 适配状态反馈（`ZitStatus` → `RobotMotionController`）
- [ ] 上游节点（`uv_automaton` 等）回归测试
- [ ] 实际水池测试：解锁 → 位置控制 → 速度控制 → 推力控制

---

## 10. 完整话题速查表

### 固件订阅（上位机 → 固件）

| 话题 | 消息类型 | 频率建议 | 用途 |
|------|----------|----------|------|
| `/zit6/cmd/setpoint` | `ZitSetpoint` | 10-50Hz | 运动控制目标 |
| `/zit6/cmd/agxhbt` | `UInt32` | ≥10Hz | 解锁心跳（**必须持续发送**） |
| `/zit6/cmd/pid` | `ZitPid` | 按需 | PID 参数在线调优 |
| `/zit6/cmd/ins` | `UInt8` | 按需 | INS/DVL 控制命令 |
| `/zit6/cmd/servo` | `Float32` | 按需 | 舵机角度 |
| `/zit6/cmd/light` | `UInt8` | 按需 | 灯光控制 |

### 固件发布（固件 → 上位机）

| 话题 | 消息类型 | 频率 | 用途 |
|------|----------|------|------|
| `/zit6/state/status` | `ZitStatus` | 10Hz | 核心状态（解锁、错误、推力等） |
| `/zit6/state/pos` | `Float32MultiArray` | 30Hz | 位置 [x,y,z,yaw] NED |
| `/zit6/state/vel` | `Float32MultiArray` | 50Hz | 速度 [vx,vy,vz,vyaw] 机体系 |
| `/zit6/state/thr` | `Float32MultiArray` | 30Hz | 推力 [Fx,Fy,Fz,Mz] |
| `/zit6/state/zithbt` | `UInt32` | 1Hz | 固件心跳（毫秒时间戳） |
| `/zit6/state/pid_status` | `ZitPidStatus` | 1Hz | 全轴 PID 参数回传 |
