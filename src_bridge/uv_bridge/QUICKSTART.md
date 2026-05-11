# 快速开始指南

## 1. 编译

```bash
cd /path/to/workspace
colcon build --packages-select uv_bridge
source install/setup.bash
```

## 2. 启动系统

### 方式一：使用 launch 文件（推荐）

```bash
ros2 launch uv_bridge bridge.launch.py
```

### 方式二：手动启动

```bash
# 终端 1: 启动桥接节点
ros2 run uv_bridge microros_bridge

# 终端 2: 启动上位机节点（如果需要）
ros2 run uv_bridge uv_hmu_no_serial \
  --pid-path /path/to/pid_parameters.json \
  --curve-path /path/to/thrust_curves.json
```

## 3. 验证通信

```bash
# 检查话题列表
ros2 topic list

# 应该看到以下话题:
# /zit6/cmd/setpoint
# /zit6/cmd/agxhbt
# /zit6/state/status
# /zit6/state/pos
# /zit6/state/vel
# /zit6/state/thr
# /zit6/state/zithbt
```

## 4. 测试控制

### 发送位置命令

```bash
ros2 topic pub --once /target_pos_down uv_msgs/msg/TargetPosDown \
  "{cs: 0, pos: {x: 1.0, y: 0.0, z: -2.0, rx: 0.0, ry: 0.0, rz: 0.0}}"
```

### 发送推力命令

```bash
ros2 topic pub --once /openloop_thrust uv_msgs/msg/RobotAxis \
  "{x: 0.1, y: 0.0, z: 0.0, rx: 0.0, ry: 0.0, rz: 0.0}"
```

### 查看状态

```bash
ros2 topic echo /zit6/state/status
```

## 5. 调试

### 启用调试日志

```bash
ros2 run uv_bridge microros_bridge --ros-args --log-level debug
```

### 监控心跳

```bash
# 终端 1: AGX 心跳
ros2 topic hz /zit6/cmd/agxhbt

# 终端 2: micro-ROS 心跳
ros2 topic hz /zit6/state/zithbt
```

### 查看节点图

```bash
rqt_graph
```

## 6. 常见问题

### 问题：找不到 uv_msgs 包

**解决:**
```bash
# 确保 uv_msgs 包已编译
colcon build --packages-select uv_msgs
source install/setup.bash
```

### 问题：找不到 zit6_interfaces 包

**解决:**
```bash
# 编译 zit6_interfaces
cd /path/to/AUV_zit6_cmake-master
colcon build --packages-select zit6_interfaces
source install/setup.bash
```

### 问题：micro-ROS 无响应

**检查:**
1. STM32 是否上电
2. 串口连接是否正常
3. micro-ROS agent 是否运行（如果需要）

### 问题：系统不解锁

**检查:**
1. 查看 `/zit6/state/status` 中的 `is_armed` 字段
2. 查看 `navigation_ready` 是否为 true
3. 查看 `error_flags` 是否有错误

## 7. 下一步

- 阅读 [README.md](README.md) 了解详细功能
- 阅读 [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md) 了解迁移细节
- 查看 [TOPIC_SPECIFICATION.md](../../AUV_zit6_cmake-master/doc/TOPIC_SPECIFICATION.md) 了解协议规范
