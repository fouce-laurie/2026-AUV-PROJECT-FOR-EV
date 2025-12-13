# Repository Guidelines

## 项目结构与模块组织
- ROS2 工作区位于 `Cruise/`，源码在 `Cruise/src/`，主要包：`uv_ai`、`uv_hm`、`uv_vision`、`uv_control_py`、`uv_launch_pkg`、`utils`、`models/`。
- 构建产物 `build/`、`install/`、`log/` 为一次性输出，勿提交。
- 部署脚本在 `scripts/run/`，诊断脚本在 `scripts/test/`，重用数据在 `datas/`；大模型权重仅存放指针，文件置于仓库外。

## 构建、测试与开发命令
- 依赖：`rosdep install --from-paths src --ignore-src -r -y`。
- 全量构建：`colcon build --symlink-install`；构建后需 `call install\setup.bat`（或 `source install/setup.bash`）覆盖环境。
- 定向构建：`colcon build --packages-select <pkg>` 便于迭代。
- 启动：`ros2 launch uv_launch_pkg launch.py`。
- 自动化检查：`colcon test --event-handlers console_cohesion+`，可加 `--packages-select <pkg>` 聚焦。

## 代码风格与命名约定
- 4 空格缩进，模块小写，类用 CapWords，节点/可执行文件与包对齐（如 `uv_capture`、`uv_hmu`）。
- 为函数、类、参数添加类型注解与简洁 docstring，主题/参数命名具描述性（例 `/uv/vision/raw_image`）。
- 本地运行 `ament_flake8`、`ament_pep257` 清理告警；可在提交前用 `colcon test` 触发。

## 测试指南
- 各包 `test/` 采用 pytest 风格，命名 `test_<feature>.py`；保留现有 lint 检查。
- 可用 `colcon test --packages-select <pkg>` 进行定向验证；问题排查时关注 `log/latest_test/`。
- 优先模拟/Mock 设备；需要硬件时复用 `datas/` 的采集与 `scripts/test/` 模式。

## 提交与 Pull Request 规范
- 使用 Conventional Commits：`<type>(<scope>): <summary>`，祈使语气，示例 `feat(vision): add depth fusion node`，主题 ≤72 字符。
- 提交说明包含动机与关联 issue/ticket；PR 描述变更、附验证命令（如 `colcon build`、`colcon test`、目标 `ros2 launch`）及必要的感知证据。
- 根据 `package.xml` 的维护者请求评审；仅在 CI 通过后合并。

## 配置与安全提示
- 连接新串口/硬件前运行 `scripts/run/permissions.sh`（或 Windows 等效脚本）。
- 新增参数需同步到 launch 文件与 YAML，便于无重编译调优；大型 AI 权重注明来源哈希并存储仓库外。

####保持中文输出！！！