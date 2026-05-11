# AUV2026 文档导航

本文件夹包含 AUV2026 项目的系统架构、节点接口、消息定义等技术文档。

## 文档列表

- **ARCHITECTURE.md** - 系统架构与设计文档
- **NODES.md** - ROS2 节点总览与接口文档（ZIT6 协议）
- **MESSAGES.md** - 消息类型定义（zit6_interfaces + uv_msgs）
- **AUTOMATON_ACTIONS.md** - 自动机动作参考
- **UPPER_COMPUTER_ADAPTATION.md** - ZIT6 上位机适配指南
- **GETTING_STARTED.md** - 开发环境配置与快速入门

## 快速参考

### 坐标系

全系统统一使用 **NED (北东地)** 坐标系：
- 世界系：X=北(North), Y=东(East), Z=下(Down)
- 机体系：X=前进(Surge), Y=右移(Sway), Z=下潜(Heave)
- Yaw：0°=朝北，顺时针为正

### ZIT6 通信协议

核心话题：
- `/zit6/cmd/setpoint` - 运动控制目标
- `/zit6/cmd/agxhbt` - 解锁心跳
- `/zit6/state/status` - 核心状态反馈
- `/zit6/state/pos` - 位置反馈 [x_ned, y_ned, z, yaw_ned_rad]

### 常见问题

- `Package 'uv_launch_pkg' not found` → `colcon build --packages-select uv_launch_pkg`
- `rqt` 提示 `can not get message class` → 编译 `uv_msgs zit6_interfaces stonefish_ros2`
- 自定义消息不可见 → `source install/setup.bash`

详见 `GETTING_STARTED.md` 的"常见问题排查"章节。
