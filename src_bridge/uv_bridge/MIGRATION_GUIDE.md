# Micro-ROS 通信协议迁移指南

## 概述

本文档说明如何从原始的串口通信协议迁移到基于 micro-ROS 的话题通信协议。

## 系统架构对比

### 原始架构（串口通信）

```
┌─────────────┐
│  上位机应用  │
└──────┬──────┘
       │ ROS Topics (uv_msgs)
       │
┌──────▼──────┐
│  uv_hmu.py  │
└──────┬──────┘
       │ Serial Protocol
       │ (0xFA 0xAF ... 0xFB 0xBF)
       │
┌──────▼──────┐
│  STM32 MCU  │
└─────────────┘
```

### 新架构（micro-ROS 通信）

```
┌─────────────┐
│  上位机应用  │
└──────┬──────┘
       │ ROS Topics (uv_msgs)
       │
┌──────▼──────────┐
│  microros_bridge │ ◄── 新增的桥接节点
└──────┬──────────┘
       │ ROS Topics (zit6_interfaces)
       │
┌──────▼──────┐
│  micro-ROS  │
│  (STM32)    │
└─────────────┘
```

## 协议映射表

### 1. 命令协议映射

#### 1.1 开环推力控制

**原始协议:**
```
串口帧: 0xFA 0xAF 0x02 + 6个float32 (x,y,z,rx,ry,rz) + 0xFB 0xBF
```

**新协议:**
```
Topic: /zit6/cmd/setpoint
Type: ZitSetpoint
Fields:
  control_key: 0x02 (FORCE mode)
  type_mask: 0x0F (all axes)
  x, y, z, yaw: float32
  seq: uint32
```

#### 1.2 位置控制

**原始协议:**
```
串口帧: 0xFA 0xAF 0x03 + uint8(cs) + 6个float32 (x,y,z,rx,ry,rz) + 0xFB 0xBF
```

**新协议:**
```
Topic: /zit6/cmd/setpoint
Type: ZitSetpoint
Fields:
  control_key: 0x00 (POS mode) | (cs==1 ? 0x10 : 0x00)
  type_mask: 0x0F
  x, y, z, yaw: float32
  seq: uint32
```

#### 1.3 PID控制器状态

**原始协议:**
```
串口帧: 0xFA 0xAF 0x01 + 6个uint8 (x,y,z,rx,ry,rz状态) + 0xFB 0xBF
```

**新协议:**
```
通过 /zit6/state/status 中的 control_level 字段反映
不再单独控制每个轴的开关
```

#### 1.4 DVL控制

**原始协议:**
```
串口帧: 0xFA 0xAF 0x04 + uint8(dvl状态) + 0xFB 0xBF
```

**新协议:**
```
Topic: /zit6/cmd/ins
Type: UInt8
Data: 1 (DVL on) / 2 (DVL off)
```

#### 1.5 舵机控制

**原始协议:**
```
串口帧: 0xFA 0xAF 0x06 + uint8(num) + float32(angle) + 0xFB 0xBF
```

**新协议:**
```
Topic: /zit6/cmd/servo
Type: Float32
Data: angle (0.0 - 1.0)
```

#### 1.6 LED控制

**原始协议:**
```
串口帧: 0xFA 0xAF 0x08 + uint8(led0) + uint8(led1) + 0xFB 0xBF
```

**新协议:**
```
Topic: /zit6/cmd/light
Type: UInt8
Data: led0 (主LED状态)
```

### 2. 状态反馈映射

#### 2.1 运动控制器状态

**原始协议:**
```
串口帧: 0xFA 0xAF 0x00 + 154字节数据 + 0xFB 0xBF
包含:
  - 位置 (24字节)
  - 目标位置base (24字节)
  - 目标位置world (24字节)
  - PID状态 (6字节)
  - IMU数据 (50字节)
  - 推力数据 (24字节)
  - LED数据 (2字节)
```

**新协议:**
```
分散到多个话题:
  /zit6/state/pos (Float32MultiArray): [x, y, z, yaw]
  /zit6/state/vel (Float32MultiArray): [vx, vy, vz, vyaw]
  /zit6/state/thr (Float32MultiArray): [f0, f1, f2, f3]
  /zit6/state/status (ZitStatus): 综合状态信息
```

#### 2.2 设备管理器状态

**原始协议:**
```
串口帧: 0xFA 0xAF 0x01 + 22字节数据 + 0xFB 0xBF
包含:
  - 漏水检测 (1字节)
  - 电压 (4字节)
  - 温度 (4字节)
  - 湿度 (1字节)
  - 电磁铁 (1字节)
  - 舵机角度 (8字节)
```

**新协议:**
```
/zit6/state/status (ZitStatus):
  battery_voltage: float32
  error_flags: uint32 (包含传感器故障标志)
```

## 迁移步骤

### 步骤 1: 编译新包

```bash
cd /path/to/workspace
colcon build --packages-select zit6_interfaces uv_bridge
source install/setup.bash
```

### 步骤 2: 测试桥接节点

```bash
# 终端 1: 启动桥接节点
ros2 run uv_bridge microros_bridge --ros-args --log-level debug

# 终端 2: 查看话题
ros2 topic list

# 终端 3: 监控状态
ros2 topic echo /zit6/state/status
```

### 步骤 3: 修改上位机代码

有两种方案:

**方案 A: 最小改动（推荐）**
- 保留原有的 `uv_hmu.py` 节点
- 移除串口通信部分
- 使用 `uv_hmu_no_serial.py` 替代
- 启动 `microros_bridge` 进行协议转换

**方案 B: 完全重写**
- 直接使用 `zit6_interfaces` 消息类型
- 不需要桥接节点
- 需要修改所有上位机代码

### 步骤 4: 更新启动脚本

**原始启动:**
```bash
ros2 run uv_hm uv_hmu --tty-path /dev/ttyUSB0
```

**新启动 (方案A):**
```bash
# 启动桥接节点
ros2 run uv_bridge microros_bridge &

# 启动上位机节点（无串口版本）
ros2 run uv_bridge uv_hmu_no_serial
```

**新启动 (方案B):**
```bash
# 直接使用新的上位机代码
ros2 run your_package new_control_node
```

### 步骤 5: 验证通信

```bash
# 1. 检查心跳
ros2 topic hz /zit6/cmd/agxhbt
ros2 topic hz /zit6/state/zithbt

# 2. 发送测试命令
ros2 topic pub --once /openloop_thrust uv_msgs/msg/RobotAxis \
  "{x: 0.1, y: 0.0, z: 0.0, rx: 0.0, ry: 0.0, rz: 0.0}"

# 3. 查看状态反馈
ros2 topic echo /zit6/state/status

# 4. 查看位置反馈
ros2 topic echo /zit6/state/pos
```

## 关键差异说明

### 1. 坐标系

**原始系统:**
- 使用 6-DOF (x, y, z, rx, ry, rz)
- 支持完整的6自由度控制

**新系统:**
- 使用 4-DOF (x, y, z, yaw)
- 仅支持 X, Y, Z 平移和 Yaw 旋转
- Roll 和 Pitch 由惯导自动稳定

### 2. PID控制

**原始系统:**
- 6个独立的PID控制器
- 每个轴可以单独开关

**新系统:**
- 4个轴，每个轴有位置环和速度环
- 通过 control_level 统一切换控制模式
- 支持在线PID参数调整

### 3. 心跳机制

**原始系统:**
- 无明确的心跳机制
- 依赖串口连接状态

**新系统:**
- 双向心跳 (AGX ↔ micro-ROS)
- 10Hz 心跳频率
- 心跳超时自动解锁
- 支持解锁模式选择 (DEFAULT/REMOTE)

### 4. 错误处理

**原始系统:**
- 串口通信错误难以检测
- 无统一的错误标志

**新系统:**
- `error_flags` 位域标志
- 明确的错误类型定义
- 支持错误恢复机制

## 调试技巧

### 1. 使用 rqt_graph 查看节点关系

```bash
rqt_graph
```

### 2. 使用 rqt_console 查看日志

```bash
rqt_console
```

### 3. 录制和回放数据

```bash
# 录制
ros2 bag record -a

# 回放
ros2 bag play <bag_file>
```

### 4. 监控话题频率

```bash
ros2 topic hz /zit6/cmd/setpoint
ros2 topic hz /zit6/state/pos
```

### 5. 查看消息定义

```bash
ros2 interface show zit6_interfaces/msg/ZitSetpoint
ros2 interface show zit6_interfaces/msg/ZitStatus
```

## 常见问题

### Q1: 桥接节点启动后无响应

**A:** 检查以下几点:
1. micro-ROS agent 是否运行
2. STM32 是否正确连接
3. 查看 `/zit6/state/zithbt` 是否有心跳

### Q2: 控制命令发送后无效

**A:** 检查:
1. 系统是否已解锁 (`is_armed=true`)
2. 导航是否就绪 (`navigation_ready=true`)
3. 查看 `error_flags` 是否有错误

### Q3: PID参数设置无效

**A:** 
1. 检查轴索引映射是否正确
2. 确认 `is_pos_ring` 标志是否正确
3. 查看 `/zit6/state/pid_status` 确认参数是否生效

### Q4: 位置反馈数据异常

**A:**
1. 检查惯导状态 (`ins_state`)
2. 确认 DVL 是否正常工作
3. 查看坐标系定义是否一致

## 性能优化建议

1. **降低不必要的话题频率**
   - 位置控制: 10-20Hz 足够
   - 速度控制: 20-50Hz
   - 推力控制: 50-100Hz

2. **使用 QoS 配置**
   - 控制命令: RELIABLE
   - 状态反馈: BEST_EFFORT (高频数据)

3. **减少日志输出**
   - 生产环境使用 `info` 级别
   - 调试时使用 `debug` 级别

4. **监控系统资源**
   ```bash
   top -p $(pgrep -f microros_bridge)
   ```

## 参考文档

- [TOPIC_SPECIFICATION.md](../../AUV_zit6_cmake-master/doc/TOPIC_SPECIFICATION.md) - micro-ROS 协议规范
- [MicroRosTask.cpp](../../AUV_zit6_cmake-master/UserApp/Application/MicroRosTask.cpp) - STM32 实现
- [zit6_interfaces](../../AUV_zit6_cmake-master/zit6_interfaces/) - 消息定义

## 版本历史

- v1.0.0 (2026-05-05): 初始版本，支持基本的协议转换
