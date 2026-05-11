# AUV2026 开发环境配置与快速入门

## 1. 系统要求与环境依赖
- 操作系统：Ubuntu 24.04 (Linux)
- ROS版本：ROS 2 Jazzy Jalisco
- 仿真器依赖：OpenGL/Vulkan, `sdl2`, `pyqtgraph` 等
- Conda 环境管理 (`ros2_jazzy_env`): 预先安装 PyTorch, Ultralytics 相关库以供 YOLO 视觉推理。可在根目录提供 `environment.yml` 加以恢复。

## 2. 工作空间构建与编译
克隆并初始化项目：
```bash
# 安装必要依赖 (如有缺漏，请利用 rosdep install --from-paths src -y --ignore-src)
cd ~/AUV2026

# 使用 Conda 激活环境
source ~/miniconda3/etc/profile.d/conda.sh
conda activate ros2_jazzy_env

# 载入基础 ROS2 环境
source /opt/ros/jazzy/setup.bash

# 进行包编译 (推荐使用 symlink，避免反复构建 Python 脚本)
colcon build --symlink-install
```

## 3. 快速唤醒环境变量
AUV2026 预设了一系列便捷的 Aliases 用于配置路径：
```bash
source ~/AUV2026/scripts/env.sh
```
此脚本能够避免由于在非当前包运行 `.bash` 时引发的文件绝对路径混淆。

## 4. 运行仿真流程 (Stonefish Simulator)
在确保所有包已正确构建并配置环境变量的情况下：

### 启动默认空环境 / 资格赛场景 / 决赛场景
```bash
# 启动环境（默认读取 datas/ 中预设模型）
uuv_sim finals

# 若您需要手动指定启动：
ros2 launch uv_launch_pkg sim_launch.py scenario_desc:=<自定义绝对路径>
```

### 独立启动 AI 与状态机节点
在**不同的带 ROS2 conda 环境终端里**运行以下指令进行组合测试：
```bash
# 开启硬件桥接映射
ros2 run uv_hm uv_sim_bridge

# 开启 AUV AI 检测
ros2 run uv_ai uv_detect_demo

# 启动任务流状态机
ros2 run uv_ai uv_automation
```

## 5. 常见问题排查 (Troubleshooting)

1. `ros2 run` 提示找不到 `torch`？
检查当前 shell 是否激活了 `ros2_jazzy_env`；如果在无 Conda 的情况下强行 `colcon build`，相关可执行脚本的 `shebang` 指向系统 Python，会丢失环境。解决方案是要么重置环境，要么在终端再次执行 `colcon build --symlink-install`，详见 `doc/memories/env_ros2_torch.md`。

2. **缺少特定标定或数据？**
目前所有核心配置在 `datas/configs/`、`datas/calibrations/`和 `datas/models/`，代码启动前若使用 `scripts/env.sh` 则环境变量已配置正确，直接通过相应的 API `WORKSPACE_ROOT / 'datas'` 去寻址即可。
