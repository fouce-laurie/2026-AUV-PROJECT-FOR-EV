# ? AUV “Cruise” 项目总体说明（AGX 上位机 + 单片机）

本文档详细介绍了 AUV "Cruise" 项目的整体架构、软件/硬件组件的职责、AGX 上的推荐操作流程以及 AGX 与两块单片机之间的串口通信协议。

---

## 1. ? 整体架构一览：核心组件及其职责

整个系统由一个上位机和两块单片机板组成，分别承担“大脑”、“肌肉”和“生命支持”功能。

| 组件 | 硬件/系统 | 核心职责概览 | 对应代码路径 |
| :--- | :--- | :--- | :--- |
| **? AGX 上位机** | Linux + ROS 2 | **大脑 + 眼睛**：感知、决策、硬件桥接。负责**“想干什么”**。 | `Ancestral_code/Cruise/src` |
| **? H743 运动控制板** | STM32H743 | **身体的肌肉和前庭**：姿态/位置计算、PID 闭环、推进器驱动。负责**“怎么游”**。 | `MCU_PROJECT/MotionControl-CSnew` |
| **? F407 设备管理板** | STM32F407 | **生命支持系统**：设备管理、环境监控（电池/温湿度/漏水）、外设控制。负责**“电够不够、漏不漏水、机械手收没收起来”**。 | `MCU_PROJECT/DeviceManager` |

### AGX 上位机（Linux + ROS 2）职责细分

AGX 运行在 ROS 2 工作区 `Ancestral_code/Cruise/src`，主要模块如下：

* **视觉采集：** `uv_vision/uv_capture.py` 负责采集摄像头图像。
* **AI 与决策：**
    * `uv_ai/uv_detect_demo.py`：运行目标检测（YOLO）。
    * `uv_ai/uv_automaton.py`：运行任务逻辑和状态机。
* **硬件桥接：** `uv_hm/uv_hm/uv_hmu.py` 负责将 ROS 话题转换为串口指令下发给单片机，并接收反馈状态。

### STM32H743 运动控制板（H743）职责

1.  读入 **IMU/DVL/深度传感器**数据，计算机器人当前姿态和位置。
2.  执行 **PID 闭环**，计算六个推进器的推力。
3.  通过 **PWM 驱动**推进器，周期性将状态打包发回 AGX。

### STM32F407 设备管理板（F407）职责

1.  采集电池电压、舱内温湿度、漏水探头状态。
2.  控制舵机、电磁铁、DVL 供电等外设。
3.  周期性将设备状态打包发回 AGX。

---

## 2. ?? 在 AGX 上的推荐操作顺序

典型的实船/试验操作流程（全部在 AGX 上执行）如下：

### 步骤 1: 准备串口权限

* **操作：** 第一次使用新设备时，运行脚本设置串口权限。
* **目的：** 确保普通用户可以访问串口设备（如 `/dev/ttyUSB0`, `/dev/ttyUSB1`）。
* **路径：** `Ancestral_code/Cruise/scripts/run/permissions.sh`

### 步骤 2: 启动硬件桥接节点 `uv_hmu.py`

* **路径：** `Ancestral_code/Cruise/src/uv_hm/uv_hm/uv_hmu.py`
* **核心功能：**
    * 使用 `uv_control_py/Serial.py` 打开两路串口（默认 H743 在 `/dev/ttyUSB1`，F407 在 `/dev/ttyUSB0`）。
    * 从 JSON 文件加载 PID 参数和推力曲线，并通过串口写入 H743。
    * 在 **ROS 话题**和**串口二进制帧**之间做“翻译”（双向）。
* **启动示例（路径按实际机器调整）：**
    ```bash
    ros2 run uv_hm uv_hmu.py \
    --h750-path /dev/ttyUSB1 --f407-path /dev/ttyUSB0 \
    --pid-path /home/nvidia/Workspace/Cruise/datas/pid_parameters.json \
    --curve-path /home/nvidia/Workspace/Cruise/datas/thrust_cureves.json
    ```

### 步骤 3: 打开 `rqt` 做调试和监控

* **常用接收话题：**
    * `motion_controller`：来自 H743 的运动状态（位置、姿态、推力、PID 状态等）。
    * `device_manager`：来自 F407 的设备状态（电压、温湿度、漏水、舵机/磁铁等）。
* **常用发送话题（控制）：**
    * `openloop_thrust`, `target_pos_down`, `pid_params_set`, `thrust_curve_set` 等。
* **优势：** 通过 `rqt` 可以在不修改代码的情况下快速修改 PID 参数、推力曲线或直接发送开环推力进行简单测试。

### 步骤 4: 启动视觉采集与检测

* **采集：** `uv_vision/uv_capture.py` 采集前视/下视摄像头画面，发布为 ROS 图像话题。
* **检测：** `uv_ai/uv_detect_demo.py` 订阅图像话题，使用 **YOLO 模型**做目标检测，发布检测结果（框、类别、置信度等）。

### 步骤 5: 运行完整任务逻辑 `uv_automaton.py`

* **路径：** `Ancestral_code/Cruise/src/uv_ai/uv_ai/uv_automaton.py`
* **作用：**
    1.  综合视觉结果、当前姿态和深度。
    2.  决定下一步动作（如“向前 1 米”、“对准目标”、“启动电磁铁”）。
    3.  通过发布 `target_pos_down`, `openloop_thrust`, `pid_controllers_set`, `magnet_controller` 等话题，将决策结果传递给 `uv_hmu.py`，再由其发送给单片机。

---

## 3. ? 串口通信协议：单片机和 AGX 的“对话”

### 3.1 物理与底层设置

* **物理链路：** 两个 **USB-UART 转换器**接到 AGX，分别连接 H743（主要上行状态）和 F407（双向指令和状态）。
* **串口配置：**
    * 上位机（`uv_control_py/Serial.py`）使用 **termios** 配置 **8N1** 模式，无流控。
    * H743/F407 的波特率在单片机工程的 `usart.c` 中配置（H743 上行状态通常为高波特率，F407 的 UART5 256000 用于 DVL 控制）。
    * 上位机 `TtyReader/TtyWriter` 封装了底层读写，仅暴露“读/写多少字节”的接口。

### 3.2 帧格式（共同约定）

AGX 和两块 MCU 统一采用以下固定帧结构（在 `Pid.py`, `Curve.py`, `comm.c` 中可见）：

| 字段 | 长度 | 值 | 描述 |
| :--- | :--- | :--- | :--- |
| **帧头** | 2 字节 | `0xFA 0xAF` | 用于帧同步，接收端检测到此即重置帧头。 |
| **命令字（cmd）** | 1 字节 | - | 决定该帧是哪类命令或状态。 |
| **负载（payload）** | 定长/变长 | `float` 或 `uint8_t`（小端） | 实际数据内容（如 PID 参数、传感器读数）。 |
| **帧尾** | 2 字节 | `0xFB 0xBF` | 标志一帧数据的结束，接收端据此解析中间的 `cmd`。 |

**示例（PID 写入帧）：** `FA AF 05 00 [5 个 float] FB BF`

### 3.3 H743 ? AGX：运动控制与状态

#### ?? 下行指令（AGX → H743）

| 命令字 (cmd) | 负载内容 | 作用 |
| :--- | :--- | :--- |
| `0x01` | 6 × `uint8_t` | PID 控制器开关（x, y, z, rx, ry, rz）。 |
| `0x02` | 6 × `float` | 开环推力（6 个推进器的期望推力）。 |
| `0x03` | 子类型 + 6 × `float` | 目标位姿（绝对/相对/叠加）。 |
| `0x05` | 轴编号 + 5 × `float` | **PID 参数写入**：`kp, ki, kd, i_limit, output_limit`。 |
| `0x07` | 电机编号 + 8 × `float` | **推进器推力曲线写入**（4 个 PWM 点和 4 个对应推力值）。 |
| `0x08` | 2 × `uint8_t` | LED 控制（两路信号灯开关）。 |
| `0x04` | 占位指令 | IMU/DVL 状态设置（主要用于切换 DVL 工作状态）。 |

> H743 端在 `comm.c` 的 `CmdSolve()` 中解析并修改内部状态，在定时器中断中用这些参数计算 PWM 输出。

#### ?? 上行状态（H743 → AGX）

* **使用命令字：** 固定 `cmd = 0x00`
* **发送频率：** 约 **16 Hz**（`TIM2`）
* **内容（约 159 字节）：**
    * 当前位姿（base 坐标）：6×`float`
    * 目标位姿（base/world 坐标）：各 6×`float`
    * PID 轴状态：6×`uint8_t`
    * IMU/DVL 状态、实际推力、LED 状态等。
* **AGX 处理：** `uv_hmu.py` 的串口线程解析 `cmd=0x00` 的帧，用 `struct.unpack` 转换为 **RobotMotionController** 消息并发布到 **`motion_controller`** 话题。

### 3.4 F407 ? AGX：设备管理与环境监控

#### ?? 下行指令（AGX → F407）

| 命令字 (cmd) | 负载内容 | 作用 |
| :--- | :--- | :--- |
| `0x06` | 1 × `uint8_t` 索引 + 1 × `float` 角度 | 舵机控制（映射到 PWM 500-2500us）。 |
| `0x09` | 1 × `uint8_t` 状态 | 电磁铁控制（输出不同占空比控制开关）。 |
| `0x04` | 1 × `uint8_t` 状态 | DVL 上/下电（通过 UART5 向 DVL 发送固定开关机帧）。 |

#### ?? 上行状态（F407 → AGX）

* **使用命令字：** 固定 `cmd = 0x01`（短状态帧）
* **发送频率：** 主循环不断发送。
* **内容：**
    * `Leak`：1B 漏水标志
    * `Voltage`：`float`（MCU 内乘以 10000 传输，以避免 C→Python 浮点精度损失）
    * `Temp / Hum`：SHTC3 温湿度
    * `State`：电磁铁状态
    * `angle[2]`：两路舵机当前角度
* **AGX 处理：** `uv_hmu.py` 将其解析为 **RobotDeviceManager** 消息，发布到 **`device_manager`** 话题。

---

## 4. ? 建议给新同学的上手路径

通过以下三个阶段，新同学可以快速掌握整个系统：

### 阶段 1: 跑通“最小闭环”

1.  **启动：** 仅启动 `uv_hmu.py` + H743 + F407。
2.  **监控：** 使用 `rqt` 检查 `motion_controller` 和 `device_manager` 话题是否有数据输出。
3.  **测试：** 手动发送 `openloop_thrust`，观察推进器响应，确认控制链路通畅。

### 阶段 2: 接上视觉链路

1.  **启动：** 启动 `uv_capture.py` 和 `uv_detect_demo.py`。
2.  **确认：** 确认图像话题和检测结果话题输出正常。
3.  **观察：** 观察 `uv_automaton.py` 如何根据检测结果修改目标位姿。

### 阶段 3: 调试任务逻辑

1.  **阅读：** 阅读 `uv_automaton.py`，理解任务状态机的设计。
2.  **调整：** 根据比赛/任务需求调整决策逻辑或参数。
3.  **原则：** 尽量少修改底层通信（`uv_hmu.py`）和控制（H743/F407）代码。

通过这三步，即使不了解底层细节的新同学也能快速在 AGX 上跑起整套系统，并理解 **AGX 与两块单片机之间“在说什么、怎么说、什么时候说”**。

## 5. MCU 学习摘记

> 汇总 MCU_PROJECT 两套 STM32 工程里值得复用的工程化实践。

- 统一 UART 帧同步：帧头 `0xFAAF` + 帧尾 `0xFBBF`，接收中断里重同步和计数防溢出，可参考 DeviceManager/Core/Src/main.c 与 MotionControl-CSnew/Device/Src/comm.c。
- 定时器分层调度：H743 用 TIM3(40Hz) 跑姿态/推力控制，TIM2(16Hz) 跑上报，TIM1 作为看门狗重启中断或 UART；保障控制与 I/O 解耦。
- 在线调参/标定链路：协议支持 PID 参数、推力曲线、目标位置、开环推力在线下发，配合 AGX 端 `uv_hmu.py` 快速迭代。
- 推力到 PWM 的完整路径：坐标变换→推力分配矩阵→分段推力曲线拟合→PWM 输出，结构清晰，便于替换模型或重新标定（`thrust.c`、`motor.c`）。
- 坐标系抽象：齐次矩阵 + 函数指针实现世界/机体系互转与姿态反解（`coordinate_system.c`），在 C 中实现轻量“对象”组织，便于扩展。
- 传感器可靠性：MS5837 含 CRC4 校验与温度补偿、深度卡尔曼滤波；F407 端漏水去抖、电压多次采样求平均，强调实际工况的稳健性。
