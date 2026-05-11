# 双 MCU 工程源码拆解与用户导读

## 1. 文档目标
本文件面向 NEW_MUC_PROJECT 下两套 STM32H7 工程，给出：
1. 工程角色划分（deviceMoniter 与 motionController）
2. Core/Src 与 Class 的逐文件作用说明
3. 模块级执行链路拆解
4. 新用户上手阅读路径
5. 关键风险与改进建议

## 2. 工程映射说明
- deviceMoniter 对应 AUV_zit6
- motionController 对应 AUV_vit6

推断依据：
- AUV_zit6 以多串口转发、环境传感（AHT20）和导航数据采集（NAV300）为主，表现为设备状态采集与上报。
- AUV_vit6 以推力分配、BLDC/舵机执行、控制算法为主，表现为运动控制执行侧。

---

## 3. deviceMoniter（AUV_zit6）深度拆解

### 3.1 启动与运行主线
1. main 入口完成 HAL、时钟、GPIO、I2C、USART 初始化。
2. Class/user_main.cpp 中实例化 AHT20 与 NAV300，启动 UART7 中断收包。
3. 主循环周期采集温湿度，串口广播调试信息；收到导航数据标志后解析 IMU 并上报。
4. UART7 中断回调置位 rx_flag，并重新开启单字节中断接收。


### 3.2 Class 逐文件作用

| 文件 | 作用 |
|---|---|
| AUV_zit6/Class/user_main.cpp | 监控侧应用主逻辑。调用 AHT20、NAV300、MCU_UART 完成数据采集和串口转发；实现 UART7 接收回调。 |

### 3.4 关键模块拆解

#### 模块 A：串口监控链路
- 入口：HAL_UART_RxCpltCallback
- 功能：UART7 收到字节后置位 rx_flag，触发后续解析流程
- 数据流：UART7 -> NAV300 数据缓存 -> IMU 解包 -> USART2 输出
- 适用场景：上位机调试、在线监视传感与导航状态

#### 模块 B：环境状态采样
- 入口：主循环内 m_AHT20.Read
- 功能：读取温度与湿度，格式化为文本串口输出
- 作用：提供舱内环境监测基础数据

### 3.5 新用户导读（deviceMoniter）
建议阅读顺序：
1. AUV_zit6/Class/user_main.cpp
2. AUV_zit6/Core/Src/usart.c
3. AUV_zit6/Core/Src/i2c.c
4. AUV_zit6/Core/Src/stm32h7xx_it.c
5. AUV_zit6/Core/Src/main.c

快速定位改动点：
- 想改采样周期和输出频率：user_main.cpp 主循环
- 想改串口波特率和口线：usart.c
- 想增删 I2C 设备：i2c.c + user_main.cpp

---

## 4. motionController（AUV_vit6）深度拆解

### 4.1 启动与运行主线
1. main 入口完成 MPU、HAL、时钟、GPIO/TIM 初始化。
2. Class/user_main.cpp 中构造 8 路 BLDC_Driver、6 路 PID、1 路 Servo、1 个 ThrusterAllocModel_8。
3. 启动电机与舵机对象后进入循环，当前示例逻辑为舵机 0-90 度往返。
4. 若接入上层控制量，可通过 ThrusterAllocModel_8::ExecuteClosedLoopControl 或 ThrustAllocate 实现推力下发。

### 4.2 Core/Src 逐文件作用

| 文件 | 作用 |
|---|---|
| AUV_vit6/Core/Src/main.c | CubeMX 基础入口，执行 TIM/GPIO 初始化并进入空循环骨架。 |
| AUV_vit6/Core/Src/tim.c | TIM2/TIM3/TIM4 PWM 初始化与对应通道 GPIO 复用。提供舵机与 8 路电机 PWM 输出基座。 |
| AUV_vit6/Core/Src/gpio.c | GPIO 端口时钟开启。 |
| AUV_vit6/Core/Src/stm32h7xx_it.c | Cortex 异常与 SysTick 处理，当前无额外外设 ISR 业务逻辑。 |
| AUV_vit6/Core/Src/stm32h7xx_hal_msp.c | HAL MSP 全局初始化骨架。 |
| AUV_vit6/Core/Src/system_stm32h7xx.c | CMSIS 系统初始化与系统时钟变量维护。 |

### 4.3 Class 逐文件作用（逐文件全量）

| 文件 | 作用 |
|---|---|
| AUV_vit6/Class/user_main.cpp | 运动控制应用入口，实例化推力分配模型、PID、电机与舵机并执行主循环。 |
| AUV_vit6/Class/TIM_Class.h | 定义 MCU_TIM 抽象接口（PWM 启停、比较值设置、中断控制）。 |
| AUV_vit6/Class/TIM_Class.cpp | MCU_TIM 的 HAL 封装实现。 |
| AUV_vit6/Class/GPIO_Class.h | 定义 MCU_GPIO 抽象接口（高低电平写入、引脚读取）。 |
| AUV_vit6/Class/GPIO_Class.cpp | MCU_GPIO 的 HAL 封装实现。 |
| AUV_vit6/Class/I2C_Class.h | 定义 MCU_I2C 抽象接口（读写寄存器、主机收发）。 |
| AUV_vit6/Class/I2C_Class.cpp | MCU_I2C 的 HAL 封装实现，统一错误处理。 |
| AUV_vit6/Class/SPI_Class.h | 定义 MCU_SPI 抽象接口（片选控制、收发字节/数据块）。 |
| AUV_vit6/Class/SPI_Class.cpp | MCU_SPI 的 HAL 封装实现，包含 CS 拉低拉高流程。 |
| AUV_vit6/Class/UART_Class.h | 定义 MCU_UART 帧协议/透传通信类，支持回调、CRC/XOR、同步头尾。 |
| AUV_vit6/Class/UART_Class.cpp | MCU_UART 收发和状态机实现，含帧头窗口匹配与尾标识识别。 |
| AUV_vit6/Class/Servo_Class.h | 舵机类定义，角度到脉宽映射参数与接口。 |
| AUV_vit6/Class/Servo_Class.cpp | 舵机 PWM 驱动实现，执行角度限幅和比较值换算。 |
| AUV_vit6/Class/BLDC_Driver_Class.h | 无刷电机驱动类定义，支持分段线性/二次曲线推力映射。 |
| AUV_vit6/Class/BLDC_Driver_Class.cpp | 推力转 PWM 计算与初始化实现。 |
| AUV_vit6/Class/ThrusterAlloc_Model.h | 8 推进器分配模型定义，包含姿态/位置结构体和闭环接口。 |
| AUV_vit6/Class/ThrusterAlloc_Model.cpp | 6DOF 到 8 电机分配、限幅缩放、双速率平滑、姿态变换与 PID 调度核心。 |
| AUV_vit6/Class/Control_Algorithm.h | PID 与 KalmanFilter 类接口与参数定义。 |
| AUV_vit6/Class/Control Algorithm.cpp | PID 与 KalmanFilter 运算实现。 |
| AUV_vit6/Class/NAV300_Class.h | NAV300 导航设备类定义，继承 UART 帧通信并定义 IMU 数据结构。 |
| AUV_vit6/Class/NAV300_Class.cpp | NAV300 指令下发与 IMU 数据解包实现。 |
| AUV_vit6/Class/MS5837_Class.h | 深度传感器类定义（MS5837），含转换命令和数据结构。 |
| AUV_vit6/Class/MS5837_Class.cpp | 深度计采样、补偿计算、深度/高度计算实现。 |
| AUV_vit6/Class/AHT20_Class.h | 温湿度传感器类接口定义。 |
| AUV_vit6/Class/AHT20_Class.cpp | AHT20 初始化与温湿度读取实现。 |
| AUV_vit6/Class/AD7799_Class.h | 高精度 ADC AD7799 寄存器与驱动接口定义。 |
| AUV_vit6/Class/AD7799_Class.cpp | AD7799 复位、寄存器读写、单次转换与电压换算实现。 |
| AUV_vit6/Class/INA228_Class.h | 电压电流功率计 INA228 寄存器与驱动接口定义。 |
| AUV_vit6/Class/INA228_Class.cpp | INA228 配置、分流校准及电参量读取实现。 |
| AUV_vit6/Class/BQ40ZXX_Class.h | 电池管理芯片 BQ40ZXX 读数接口与结构体定义。 |
| AUV_vit6/Class/BQ40ZXX_Class.cpp | SMBus 风格 CRC 校验、word/block 读取、电芯电压读取实现。 |
| AUV_vit6/Class/SC8815_Class.h | 电源路径管理芯片 SC8815 的寄存器、配置结构和控制接口定义。 |
| AUV_vit6/Class/SC8815_Class.cpp | SC8815 初始化、寄存器配置、ADC 读值和 OTG/保护控制实现。 |
| AUV_vit6/Class/SSD1306_Class.h | OLED 显示类定义（I2C/SPI 双后端），含图元和文本接口。 |
| AUV_vit6/Class/SSD1306_Class.cpp | SSD1306 初始化、缓冲刷新、绘图与文本渲染实现。 |
| AUV_vit6/Class/SSD1306_fonts.h | OLED 字体对象声明。 |
| AUV_vit6/Class/SSD1306_fonts.cpp | 字体点阵数据表（资源文件）。 |
| AUV_vit6/Class/LED_Class.h | LED 抽象类定义，支持 GPIO 或 PWM 呼吸灯模式。 |
| AUV_vit6/Class/LED_Class.cpp | 单灯/多灯呼吸效果实现。 |

### 4.4 模块级功能拆解

#### 模块 A：执行器抽象层
- 组成：TIM_Class、GPIO_Class、I2C_Class、SPI_Class、UART_Class
- 职责：统一 HAL 接口，向上提供稳定类接口
- 价值：上层业务可不直接操作 HAL 细节

#### 模块 B：运动执行层
- 组成：Servo_Class、BLDC_Driver_Class、ThrusterAlloc_Model
- 职责：将角度/推力/六自由度控制量转换为具体 PWM 输出
- 核心流程：目标力矩向量 -> 8 推进器分配 -> 限幅 -> 平滑 -> compare 下发

#### 模块 C：控制算法层
- 组成：Control_Algorithm（PID + Kalman）
- 职责：状态误差闭环计算与状态估计
- 典型配合：ThrusterAlloc_Model::ExecuteClosedLoopControl

#### 模块 D：感知与电源管理层
- 组成：NAV300、MS5837、AHT20、INA228、AD7799、BQ40ZXX、SC8815
- 职责：姿态/深度/环境/电池/电源路径等状态获取与控制

#### 模块 E：显示与交互层
- 组成：SSD1306_Class + SSD1306_fonts、LED_Class
- 职责：状态可视化与状态灯反馈

### 4.5 新用户导读（motionController）
建议阅读顺序：
1. AUV_vit6/Class/user_main.cpp
2. AUV_vit6/Class/ThrusterAlloc_Model.h
3. AUV_vit6/Class/ThrusterAlloc_Model.cpp
4. AUV_vit6/Class/BLDC_Driver_Class.h
5. AUV_vit6/Class/BLDC_Driver_Class.cpp
6. AUV_vit6/Class/Control_Algorithm.h
7. AUV_vit6/Class/Control Algorithm.cpp
8. AUV_vit6/Core/Src/tim.c
9. AUV_vit6/Core/Src/main.c

快速定位改动点：
- 想改电机布局与力矩映射：ThrusterAlloc_Model.cpp 的 allocationMatrix_data
- 想改电机推力曲线：BLDC_Driver_Class.cpp 的 refresh_pwm
- 想改 PID 参数与模式：user_main.cpp 的 PID 数组初始化
- 想改舵机角度映射：Servo_Class.h / Servo_Class.cpp

---

## 5. 关键问题与风险分析（重点）

### 5.1 逻辑缺陷风险
1. AD7799 类中多处成员未正确写回
- SetMode/SetRate/SetChannel/SetGain/SetPolarity 使用形参同名赋值，存在 self-assignment 风险，成员状态可能未更新。

2. AD7799 电压换算函数存在变量遮蔽问题
- RawToVolt 内部将 gain 同名定义为局部变量，可能导致换算结果异常。

3. BQ40ZXX block 读函数的数据复制与 CRC 判断流程可疑
- 当前代码在 CRC 分支中的 copy/return 路径与常规逻辑不一致，存在“校验正确却不拷贝数据”风险。

4. UART_Class 发送校验类型与配置不一致
- transmit 在帧模式下固定调用 CRC8，未按 XOR 配置动态切换。

5. SC8815 中断状态读取寄存器可能使用错误
- ReadInterrupStatus 读取的是 MASK 寄存器而非 STATUS 寄存器，可能读不到真实中断状态。

6. motionController 的闭环 PID 默认模式为 MANUAL
- 若未显式切到 AUTOMATIC，Compute 不会输出有效控制量，闭环控制不生效。

### 5.2 架构与维护风险
1. AUV_zit6 仅有 user_main.cpp，但引用了多个 Class 组件
- 说明公共 Class 可能依赖跨工程包含路径，迁移或独立构建时易出现缺失。

2. user_main 中调试输出大量固定长度发送
- 统一发送 100 字节可能带来无效字节和串口占用，建议改为按 strlen 发送。

3. UART7 回调内调用 HAL_UART_IRQHandler
- 中断回调再次调用 IRQ 处理函数可能导致重复分发或不可预期行为。

---

## 6. 两板协同关系（建议理解模型）
- deviceMoniter：偏“感知采集与状态分发”，面向设备侧监控和通信桥接。
- motionController：偏“控制计算与执行输出”，面向推进器与执行机构控制。

推荐协同接口抽象：
1. 输入：姿态、位置、电池、电源状态
2. 输出：6DOF 控制目标、执行器状态、故障与降级标志
3. 协议：统一帧头/帧尾、校验策略、序列号与时间戳

---

## 7. 维护建议清单
1. 先修复 AD7799/BQ40ZXX/UART/SC8815 的明显逻辑问题，再做功能扩展。
2. 在两工程中统一 Class 来源，减少跨工程隐式依赖。
3. 为关键类增加最小化自检用例：PID、推力分配、串口帧解析、电池读数。
4. 建议将 user_main 与 CubeMX 生成 main.c 的职责边界文档化，避免多人协作冲突。
5. 为每个传感器和电源模块定义故障码与重试策略，便于海试排障。

---

## 8. 快速上手命令建议
1. 先打开对应工程 ioc，确认时钟树与引脚复用。
2. 从 user_main.cpp 入手看主流程，再跳到被调用类。
3. 优先打点串口与 PWM 输出链路，确认真实外设动作再调算法。

本说明文档基于当前仓库源码静态分析生成，适用于首次接手项目、模块拆分重构前评估、以及联调前的角色边界梳理。
