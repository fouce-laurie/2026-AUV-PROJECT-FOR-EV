这份架构设计文档的基础非常扎实。为了将我们之前深入探讨的**“内置运动学平滑器”、“前馈控制”、“无扰切换（特别是Z轴积分继承）”**以及**“微积分变频拆分协议”**完美融合进去，我们需要对 **第 3、4、5、6 节** 进行重构。

我将纯 C++ 类设计的职责划分得更加明确，将控制算法与 ROS 2 通信解耦。以下是为你注入了全部高级逻辑的完整 Markdown 文档，你可以直接覆盖原文件：

***

# AUV 固件 C++ 架构设计方案 (Optimized & Feedforward Ready)

本文档基于现有 C 语言工程封装经验与现代多旋翼/AUV控制理论，设计了 UserApp 下的模块化 C++ 架构，旨在实现逻辑解耦、前馈轨迹跟踪与极致的代码复用。

## 1. Porting (硬件适配层)
负责硬件外设的底层驱动与数据搬运。

* **`UartDmaTransport`**: 
    * 专用于 micro-ROS (USART2)，处理 Cache 一致性及 ORE 自愈。
* **`SerialPort`**: 
    * 通用串口 DMA 类，用于 **UART7 (INS)** 和 **UART4 (VIT6)**。
    * 支持循环缓冲读取，减少 CPU 中断频率。

## 2. Peripherals (外设驱动与模型层)
负责协议解析与物理特性建模。

* **`INS_Driver`**:
    * **职责**：封装原 `ImuSolve` 逻辑，解析惯导协议包。
    * **输出**：更新标准的 `NavState` 结构体（含全局位姿与机体系瞬态速度）。
* **`VIT6_Link`**:
    * **职责**：执行下位机动态定长帧协议（ID 0x01, 0x02, 0x03），包含 CRC-16 校验。
    * **控制功能**：高频下发推力矩阵（将上位机 -1.0~1.0 归一化浮点映射为 -1000~1000 整型）与舵机角度。
    * **配置功能**：转发静态配置参数（推力曲线、舵机零位）与状态指令。


---

### 3. Chassis (数学与底盘控制层) - *[Core Algorithm]*
本仓库当前实现的控制核心由 `KinematicProfile`、`PID_Controller` 与 `ChassisManager` 组成。实际代码中，`ChassisManager` 集成了设计稿中原先划分为 `SetpointRouter` 与 `CascadeController` 的大部分职责；下文以代码实现为主并标注与原设计的差异。

- `KinematicProfile` (影子平滑器)
    - 在代码中类名为 `KinematicProfile`，按轴实例化 4 次（X,Y,Z,Yaw）。以 10ms 为期望循环步长进行更新，基于 `max_v`、`max_a` 生成 $P_d,V_d,A_d$。
    - 注意：实现中 `update(target_p, dt)` 使用基于距离的理想速度计算并积分位置/速度，未显式导出三阶连续但在常规工况下能保证平滑性。

- `ChassisManager`（整合路由 + 级联控制）
    - `ChassisManager` 在本工程承担：设定点路由（根据 incoming 控制字决定 POSITION/VELOCITY/FORCE）、影子平滑器对齐、位置→速度→推力的级联控制流程以及 actuator 偏置写入接口。
    - 位置环使用 `pos_pids_`（P 结构）、速度环使用 `vel_pids_`（PI/PID 结构）。位置环输出与影子速度 `d.v` 相加形成速度参考（见代码：v_ref = pos_pid + d.v）。速度环输出叠加加速度前馈 `d.a` 作为推力基底。
    - Z 轴积分继承（Trim Pre-loading）：已实现为在从其他层级切回 `POSITION` 时，调用 `vel_pids_[2].setIntegral(last_output_forces_[2])` 以保留上周期的 Z 推力状态（注意：这里使用的是上周期最终输出快照 `last_output_forces_`，而非文档中“反向代入”的描述）。
    - ACTUATOR 层级行为修正：代码变更后 `ACTUATOR` 层级不会自动清空 `target_forces_`，允许外部直接写入并立即生效（便于仿真或诊断）。

- `CoordinateManager`
    - 提供 `worldToBody` / `bodyToWorld` 的二维旋转转换函数，代码实现与设计意图一致。注意：函数使用 yaw 进行平面旋转，符号约定已在实现中固定，使用时请保持一致。

- `KinematicsSolver` / VIT6 下发
    - 原设计提到的 `KinematicsSolver` 与 `VIT6_Link` 在代码中未作为独立模块完整实现（仓库中存在 `setActuatorForces`、`VIT6` 相关抽象在未来可扩展）。目前 `ChassisManager::update` 输出的是 4-DOF 归一化力矢量，由上层（或未来的 `VIT6_Link`）负责映射为具体推进器命令与通信下发。

差异小结：
- 设计稿把路由与级联控制拆分为独立类，代码中以 `ChassisManager` 合并实现；若要恢复模块化，可按职责拆分 `SetpointRouter`（负责 message → target 更新与层级切换）与 `CascadeController`（只做数值控制），二者通过清晰接口交互。
- 文档中描述的动力学前馈（Mass·A_d、Drag·V_d|V_d|）在当前实现中仅保留简单加速度前馈 `d.a`，完整模型可在 `ChassisManager` 中按需求增强。


## 4. Application (应用业务与调度层)
当前代码以 `AppMain.cpp` 为入口，创建两类主要运行任务：`UserApp_ControlTask`（控制主循环）与 `UserApp_MicroRosTask`（micro-ROS 通信）。下列为实现要点与与原设计的对应关系：

- `Navigator` / `INS_Driver`：仓库中存在 `INS_Driver`，负责解析惯导并提供 `NavState`。`AppMain` 直接通过 `ins_driver.getNavState()` 获取最新观测。

- `UrosNode`（在代码中为 `UserApp_MicroRosTask`）
    - 实现细节：在发现 agent 后初始化 `rcl` 节点并创建 publisher/subscription。
    - 发布频率（代码实现）：
        - 位置 `/zit6/state/pos`：约 33ms（≈30Hz）
        - 速度 `/zit6/state/vel`：约 16ms（≈62.5Hz）
        - 推力 `/zit6/state/thr`：约 33ms（≈30Hz）
        - 状态 `/zit6/state/status`：100ms（10Hz）
        - 解锁镜像 `/zit6/state/isarm`：100ms（10Hz）
        - 节点心跳 `/zit6/state/zithbt`：1000ms（1Hz）
    - 订阅话题：`/zit6/cmd/setpoint`、`/zit6/cmd/agxhbt`（arm heartbeat）与 `/zit6/cmd/ins`。

- `CommandInterpreter`（合入 `onZitSetpoint` 与 `UserApp_ControlTask`）
    - `onZitSetpoint` 中负责解析 `ZitSetpoint` 的 `control_key`，并据此将消息映射为：POSITION、VELOCITY 或 FORCE（ACTUATOR）三种行为。
    - 注意点（已在代码中强化）：未解锁状态下仅接受 FORCE（level==2）用于诊断/仿真；对 POSITION/VELOCITY 层级的切换均以真实传感器值作为对齐输入传入 `ChassisManager::setControlLevel`（避免把 target_p 误当作实际值）。

实现差异与建议：
- 原设计中的 `CommandInterpreter` 与 `UrosNode` 功能已经实现，但主题命名与频率与原文档不同（原文档使用 `/auv/...` 命名，代码采用 `/zit6/...` 前缀），请以代码为准或统一替换命名空间。
- 安全 ARM 的“双重连续确认”时序：代码中采用“≥10 次心跳且持续 ≥1s（1000ms）”，而原文档写 2s，可视需要调整为更严格的 2s。


---

## 5. 任务线程与调度 (Task Scheduling)
当前实现的调度模型较为简单，主要任务为：

1. `UserApp_ControlTask`（在代码里实现，期望 10ms 周期） — 执行观测读取、平滑器更新、级联控制计算与 failsafe/arm 状态机。
   - 代码细节：`ChassisManager::update` 内部计算 dt，且对 dt 做了边界限定（$dt \in [1\,ms, 100\,ms]$），防止异常 tick 导致数值不稳定。
2. `UserApp_MicroRosTask`（micro-ROS 代理与 publisher/subscriber） — 发现 agent 后初始化并异步 spin。发布/订阅在该任务中执行。

实现差异：
- 设计稿的三任务划分（Control / UROS / Monitor）在代码中被合并：监控与 failsafe（心跳超时、上锁切换、断联处理）由 `UserApp_ControlTask` 内的逻辑实现，而不是单独的 `Monitor_Failsafe_Task`。

建议：若希望职责更清晰，可将监控逻辑按 10Hz 抽象为单独任务，降低主控任务内的分支复杂度并提升可观测性。


---

## 6. 系统运行工作流 (Operational Workflow)
以下为与代码一致的实际运行流程与参数（已将实现细节对齐并列出关键阈值）：

1. INIT（初始化与同步）
    - `UserApp_ControlTask` 启动后首先调用 `ins_driver.init()` 做传感器自检/启动。
    - `UserApp_MicroRosTask` 负责与 ROS2 Agent 握手，发现 Agent 后初始化 node 并开始发布/订阅。

2. ARMING（解锁校验序列）
    - 订阅话题：`/zit6/cmd/agxhbt` 被视为 arm 心跳。
    - 当前实现阈值：需要 **≥10 次心跳且持续 ≥1000ms** 才会触发解锁（代码中 `arm_heartbeat_count >= 10 && now - arm_start_ms >= 1000`）。
    - 解锁立即调用 `ins_driver.resetPosition()`（对齐位姿），并将控制层级切为 `POSITION`（使用真实传感器值作为对齐输入）。
    - 上锁条件：在已解锁状态下若 **200ms** 未收到心跳，则立即上锁并切入 `NONE` 层级（安全停机）。心跳计数的窗口重置时间为 **600ms**（超过此间隔将清零计数）。

3. OFFBOARD（动态轨迹追踪）
    - 控制主循环期望运行于 100Hz（任务以 `vTaskDelayUntil` 固定 10ms 周期），`ChassisManager::update` 使用 `HAL_GetTick()` 计算真实 dt，并将 dt 限定在 [1ms, 100ms] 以避免异常采样导致的不稳定。
    - 位置→速度→推力的级联逻辑中，位置环输出与影子速度 `d.v` 相加生成速度参考，速度环输出叠加加速度前馈 `d.a`。
    - Z 轴的积分继承在层级切换时已实现：切换到 POSITION 会将 `last_output_forces_[2]` 作为速度 PID 的积分初值。

4. FAILSAFE（安全超时与降级保护）
    - 代码在 ControlTask 中实现基础的 arm 心跳超时与重置逻辑；除此以外，若长时间没有上位机命令/心跳，可由上层策略触发 DISARM（代码中对全局断联超 3s 的终极断联处理留有扩展点）。

5. DISARMING（锁定序列）
    - 上锁时会清空 `target_forces_`（在 `ChassisManager::setControlLevel` 中对 `NONE` 层级执行清零），并将系统置于被动安全态。

综上：我已把文档中与代码不一致的命名、topic、频率与阈值对齐；并把原设计的可选增强点（如完整前馈模型、独立 Monitor 任务、独立 KinematicsSolver 模块）以建议形式保留在文末以便未来拆分。

---

## 7. 控制逻辑审查报告（按当前代码实现）

这一节不是“理想设计”，而是对当前实现的真实审查结果。目标是让你清楚知道控制链路现在到底怎么跑、哪些地方已经修过、哪些地方仍需继续收紧。

### 7.1 控制链路总览

当前控制链路是：

1. `UserApp_MicroRosTask` 接收 `/zit6/cmd/setpoint`、`/zit6/cmd/agxhbt` 和 `/zit6/cmd/ins`。
2. `onZitSetpoint()` 解析消息，更新目标位姿或目标推力。
3. `UserApp_ControlTask` 以 10ms 周期读取 `INS_Driver::NavState`。
4. `ChassisManager::update()` 对 4 个自由度分别执行：
     - 影子轨迹生成
     - 位置环或速度环 PID
     - 前馈叠加
     - 输出限幅
5. 输出结果由状态发布器上报到 `/zit6/state/thr`、`/zit6/state/status`、`/zit6/state/isarm` 和 `/zit6/state/zithbt`。
6. **参数更新流**：`onZitPid()` 订阅并解析 `/zit6/cmd/pid`，调用 `ChassisManager::configurePID()` 实时重载算法增益，无需停机或重新烧录。

这条链路的关键是：`ChassisManager` 现在不是单纯“算数”，它同时承担了状态机、路由器和级联控制器三件事。

### 7.2 已识别并修复的漏洞

- 漏洞 1：未解锁时允许 FORCE 直接进入执行器路径
    - 风险：这会让“未解锁”状态失去安全意义。
    - 处理：已改成未解锁时完全拒收会影响推进器的 setpoint。

- 漏洞 2：`setControlLevel()` 在 ACTUATOR 层级下误清空 `target_forces_`
    - 风险：外部直接写入执行器输出会被无意抹掉。
    - 处理：已改为 ACTUATOR 仅切层级，不清空偏置。

- 漏洞 3：`setControlLevel()` 传入了错误的“实际值”参数
    - 风险：无扰切换会被错误的对齐输入破坏。
    - 处理：已改为传入传感器真实值 `actual_p/actual_v`。

- 漏洞 4：共享状态无保护
    - 风险：`target_p`、ARM 计数、ARM 状态可能在任务切换中出现竞态。
    - 处理：已加入轻量临界区保护关键状态。

- 漏洞 5：轨迹平滑器对非法参数缺少保护
    - 风险：负限值或零步长会让轨迹数学失真。
    - 处理：已对 `max_v/max_a/dt` 做合法性保护。

### 7.3 仍需注意的设计风险

- `VELOCITY` 模式下仍复用了 `KinematicProfile` 的影子轨迹接口，虽然不直接决定最终控制输出，但它仍在内部演进。这个实现是“够用但不最干净”的，最好后续把速度模式和位置模式分离成不同数据流。
- `target_p` 这个变量名仍然过于宽泛，它既承载位置目标，也承载速度目标。建议后续重命名为 `target_state[4]` 或拆成 `target_position` / `target_velocity` / `target_force`。
- `last_output_forces_[2]` 作为 Z 轴积分继承初值，是一个工程上好用的近似，但它不是严格的物理反算。它能减少切层时的突跳，但不是完整的配平模型。

### 7.4 安全结论

当前代码的安全边界已经比原始设计更保守，但仍建议把下面三条作为后续硬约束：

1. 未解锁状态下，任何会改变推进器输出的消息都必须被丢弃。
2. 所有跨任务共享变量都应该加锁或改成消息队列/双缓冲。
3. 所有控制输入都必须做 NaN/Inf 和范围检查。

---

## 8. 数学实现说明

这一节专门把控制数学讲清楚，便于你后续继续调参或者把控制器拆出去做独立模块。

### 8.1 影子运动学平滑器 `KinematicProfile`

平滑器的目标不是“精确追上目标位置”，而是给控制器提供一个物理可达、无突变的参考轨迹。

设当前影子状态为：

$$
S = [p, v, a]
$$

设目标位置为 $p_t$，剩余距离为：

$$
\Delta p = p_t - p
$$

代码里用“刹停距离”的思路生成理想速度：

$$
v_{ideal} = \operatorname{sgn}(\Delta p) \cdot \sqrt{2 a_{max} |\Delta p|}
$$

然后再进行最大速度限制：

$$
v_{cmd} = \operatorname{sat}(v_{ideal}, -v_{max}, v_{max})
$$

加速度用速度误差计算：

$$
a_{raw} = \frac{v_{cmd} - v}{dt}
$$

再进行加速度限幅：

$$
a = \operatorname{sat}(a_{raw}, -a_{max}, a_{max})
$$

最终离散积分：

$$
v_{k+1} = v_k + a_k dt
$$

$$
p_{k+1} = p_k + v_{k+1} dt
$$

这意味着它是一个“物理约束下的离散轨迹生成器”，不是严格的三阶连续多项式轨迹。它的优势是简单、稳定、容易调；代价是数学光滑性不如高阶轨迹规划。

### 8.2 位置环 PID

位置环只在 `POSITION` 层级启用，形式上是一个比例环：

$$
v_{ref} = K_{p,pos}(p_d - p) + v_d
$$

其中：
- $p_d$ 是平滑器输出的位置参考。
- $p$ 是真实传感器位置。
- $v_d$ 是平滑器给出的速度前馈。

所以位置环并不是“直接输出推力”，而是把位置误差转化为速度修正量，再交给速度环处理。

### 8.3 速度环 PID

速度环输出的是最终控制量的主干：

$$
f_{base} = PID_{vel}(v_{ref} - v) + a_d
$$

其中速度 PID 的实现为：

$$
PID_{vel}(e) = K_p e + I + K_d \dot e
$$

但这里的微分项并不是对误差求导，而是使用外部传入的导数信号，目的是减少噪声放大。当前代码里最常见的导数输入就是实际速度项对应的导数近似。

积分项离散更新为：

$$
I_{k+1} = \operatorname{sat}(I_k + K_i e_k dt, -I_{max}, I_{max})
$$

总输出再做限幅：

$$
f = \operatorname{sat}(f_{base}, -f_{max}, f_{max})
$$

### 8.4 Z 轴积分继承

Z 轴切层时的目标不是“数学正确”，而是“工程上不跳”。当前实现采用：

$$
I_z \leftarrow F_{z,last}
$$

也就是把上一周期输出的 Z 轴推力快照直接作为积分初值。它的作用是保住悬停配平，不让切换瞬间出现突然掉高或者上窜。

这是一种工程型近似，不是严格逆模型。若后续要做得更完整，可以用以下形式替代：

$$
I_z \leftarrow F_{z,hover} - K_p e_z - K_d \dot e_z
$$

不过这就要求你能较准确地估计悬停力和扰动项。

### 8.5 无扰切换的数学意义

无扰切换的核心不是“切换时输出不变”，而是“切换瞬间状态连续”。所以代码里做了两件事：

1. 让平滑器状态与真实状态对齐：

$$
p_d \leftarrow p, \quad v_d \leftarrow v
$$

2. 让积分器不清零，而是继承上一次的有效输出。

这样可以尽量避免控制层级从 VELOCITY 回到 POSITION 时的跳变。

### 8.6 坐标变换

`CoordinateManager` 采用标准二维旋转矩阵：

$$
\begin{bmatrix}
x_b \\
y_b
\end{bmatrix}
=
\begin{bmatrix}
\cos\psi & \sin\psi \\
-\sin\psi & \cos\psi
\end{bmatrix}
\begin{bmatrix}
x_w \\
y_w
\end{bmatrix}
$$

反变换为：

$$
\begin{bmatrix}
x_w \\
y_w
\end{bmatrix}
=
\begin{bmatrix}
\cos\psi & -\sin\psi \\
\sin\psi & \cos\psi
\end{bmatrix}
\begin{bmatrix}
x_b \\
y_b
\end{bmatrix}
$$

这里要特别注意符号约定，否则世界系/机体系的方向会整体反掉。

### 8.7 建议的后续数学增强

如果你想把控制器继续做强，下一步最值得加的是：

1. 质量矩阵前馈 $M A_d$。
2. 阻尼/阻力项 $D(V_d)|V_d|$。
3. 推进器分配矩阵 $B^{-1}$，把 4-DOF 力映射到实际推进器。

但在没有稳定模型参数之前，当前“平滑器 + PID + 简单加速度前馈”的方案是更稳的工程选择。