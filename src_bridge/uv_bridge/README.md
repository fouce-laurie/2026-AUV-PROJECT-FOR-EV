# Micro-ROS Bridge (传话筒节点)

## 概述

这个包提供了一个桥接节点，用于在原始的串口通信协议（`uv_msgs`）和新的 micro-ROS 话题通信协议（`zit6_interfaces`）之间进行转换。

## 功能

### 1. 命令转换（上位机 → micro-ROS）

| 原始话题 | 消息类型 | micro-ROS 话题 | 消息类型 | 说明 |
|---------|---------|---------------|---------|------|
| `openloop_thrust` | `RobotAxis` | `/zit6/cmd/setpoint` | `ZitSetpoint` | 开环推力控制（FORCE模式） |
| `target_pos_down` | `TargetPosDown` | `/zit6/cmd/setpoint` | `ZitSetpoint` | 目标位置控制（POS模式） |
| `Target_speed_down` | `TargetPosDown` | `/zit6/cmd/setpoint` | `ZitSetpoint` | 目标速度控制（VEL模式） |
| `pid_params_set` | `PidParams` | `/zit6/cmd/pid` | `ZitPid` | PID参数设置 |
| `servo_control` | `ServoSet` | `/zit6/cmd/servo` | `Float32` | 舵机控制 |
| `led_controllers` | `LedControllers` | `/zit6/cmd/light` | `UInt8` | LED灯光控制 |
| `dvl_set` | `ImuData` | `/zit6/cmd/ins` | `UInt8` | DVL/惯导控制 |

### 2. 状态反馈（micro-ROS → 上位机）

| micro-ROS 话题 | 消息类型 | 原始话题 | 消息类型 | 说明 |
|---------------|---------|---------|---------|------|
| `/zit6/state/status` | `ZitStatus` | `motion_controller` | `RobotMotionController` | 系统状态 |
| `/zit6/state/pos` | `Float32MultiArray` | `motion_controller` | `RobotMotionController` | 位置反馈 |
| `/zit6/state/vel` | `Float32MultiArray` | `motion_controller` | `RobotMotionController` | 速度反馈 |
| `/zit6/state/thr` | `Float32MultiArray` | `motion_controller` | `RobotMotionController` | 推力反馈 |
| `/zit6/state/pid_status` | `ZitPidStatus` | - | - | PID状态（仅记录） |

### 3. 心跳机制

- **AGX → micro-ROS**: 10Hz 心跳发送到 `/zit6/cmd/agxhbt`
- **micro-ROS → AGX**: 接收 `/zit6/state/zithbt` 心跳
- **自动解锁**: 心跳建立3秒后自动尝试解锁系统

## 控制模式映射

### ZitSetpoint control_key 编码

```
Bits 0-1: 控制模式
  00 (0x00) = POS  位置控制
  01 (0x01) = VEL  速度控制
  10 (0x02) = FORCE 推力控制
  11 (0x03) = 保留

Bit 4 (0x10): 机体坐标系标志
  0 = 世界坐标系
  1 = 机体坐标系

Bit 5 (0x20): 增量模式标志
  0 = 绝对值
  1 = 增量值
```

### type_mask 轴掩码

```
Bit 0 (0x01): X轴
Bit 1 (0x02): Y轴
Bit 2 (0x04): Z轴
Bit 3 (0x08): Yaw轴
```

## 使用方法

### 1. 编译

```bash
cd /path/to/workspace
colcon build --packages-select uv_bridge
source install/setup.bash
```

### 2. 运行

```bash
# 启动桥接节点
ros2 run uv_bridge microros_bridge

# 或使用 launch 文件
ros2 launch uv_bridge bridge.launch.py
```

### 3. 与原有系统集成

原有的 `uv_hmu.py` 节点可以继续运行，但需要：

1. **禁用串口通信部分**：不再直接通过串口发送数据
2. **保留话题接口**：继续订阅和发布原有的 ROS 话题
3. **启动桥接节点**：运行 `microros_bridge` 进行协议转换

修改后的启动顺序：
```bash
# 1. 启动 micro-ROS agent（如果需要）
# micro-ros-agent serial --dev /dev/ttyUSB0

# 2. 启动桥接节点
ros2 run uv_bridge microros_bridge

# 3. 启动原有的上位机节点（修改后的版本，不含串口通信）
ros2 run uv_hm uv_hmu_no_serial
```

## 系统架构

```
┌─────────────────┐
│  上位机应用层    │
│  (原有代码)      │
└────────┬────────┘
         │ uv_msgs topics
         │
┌────────▼────────┐
│  Bridge Node    │ ◄─── 本包
│  (传话筒节点)    │
└────────┬────────┘
         │ zit6_interfaces topics
         │
┌────────▼────────┐
│  micro-ROS      │
│  (STM32单片机)   │
└─────────────────┘
```

## 启动流程

1. **上电初始化** (0-1秒)
   - micro-ROS 节点启动
   - 传感器自检
   - 桥接节点启动

2. **心跳建立** (1-3秒)
   - AGX 开始发送心跳
   - micro-ROS 响应心跳
   - 链路健康检查

3. **系统解锁** (3秒后)
   - 检查导航就绪状态
   - 检查电池电压
   - 检查错误标志
   - 发送解锁命令

4. **正常运行**
   - 接收控制命令
   - 发布状态反馈
   - 维持心跳

## 调试

### 查看话题列表
```bash
ros2 topic list
```

### 监控桥接节点日志
```bash
ros2 run uv_bridge microros_bridge --ros-args --log-level debug
```

### 查看特定话题
```bash
# 查看 setpoint 命令
ros2 topic echo /zit6/cmd/setpoint

# 查看状态反馈
ros2 topic echo /zit6/state/status

# 查看心跳
ros2 topic echo /zit6/cmd/agxhbt
```

### 手动发送测试命令
```bash
# 发送位置命令
ros2 topic pub /target_pos_down uv_msgs/msg/TargetPosDown "{cs: 0, pos: {x: 1.0, y: 0.0, z: -2.0, rz: 0.0}}"

# 发送推力命令
ros2 topic pub /openloop_thrust uv_msgs/msg/RobotAxis "{x: 0.1, y: 0.0, z: 0.0, rz: 0.0}"
```

## 注意事项

1. **坐标系转换**: 确保原始系统和 micro-ROS 使用相同的坐标系定义
2. **单位转换**: 检查力、速度、位置的单位是否一致
3. **心跳超时**: micro-ROS 端会检测心跳超时，超时后自动解锁
4. **安全机制**: 保留原有的安全检查机制
5. **PID参数**: 轴索引映射需要仔细核对（原系统有6个轴，micro-ROS只有4个）

## 故障排查

### 问题：桥接节点无法启动
- 检查依赖包是否安装：`uv_msgs`, `zit6_interfaces`
- 检查 ROS2 环境是否正确 source

### 问题：micro-ROS 无响应
- 检查 micro-ROS agent 是否运行
- 检查串口连接是否正常
- 查看 `/zit6/state/zithbt` 是否有心跳

### 问题：控制命令无效
- 检查系统是否已解锁（`is_armed=true`）
- 检查导航是否就绪（`navigation_ready=true`）
- 查看 `/zit6/state/status` 中的 `error_flags`

## 开发者信息

- 版本: 1.0.0
- 协议版本: v5.0
- 参考文档: `AUV_zit6_cmake-master/doc/TOPIC_SPECIFICATION.md`
