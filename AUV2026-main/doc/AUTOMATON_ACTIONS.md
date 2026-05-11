# uv_automaton.py 动作汇总

本文档总结 `src/uv_ai/uv_ai/uv_automaton.py` 中对外可调用或作为任务单元的主要动作（方法），便于查看与调用说明。

## 坐标系说明

全系统统一使用 **NED (北东地)** 坐标系：
- 世界系：X=北(North), Y=东(East), Z=下(Down)
- 机体系：X=前进(Surge), Y=右移(Sway), Z=下潜(Heave)
- Yaw：0°=朝北，顺时针为正

## 控制方法

所有控制方法通过 `_send_setpoint()` 发送 `ZitSetpoint` 消息到 `/zit6/cmd/setpoint` 话题。

### 核心控制函数

- `_send_setpoint(control_key, type_mask, x, y, z, rz_deg)`
  - 说明：核心控制方法，发送 ZIT6 setpoint 消息
  - 参数：
    - `control_key`: 控制模式编码（见下表）
    - `type_mask`: 轴掩码（0x01=X, 0x02=Y, 0x04=Z, 0x08=Yaw）
    - `x, y, z`: 位置/速度/推力目标值
    - `rz_deg`: 偏航角度（度数，内部转换为弧度）

**常用 control_key 值：**
- `0x00` = 位置模式 + 世界系 + 绝对量
- `0x20` = 位置模式 + 世界系 + 增量模式（机体系偏移）
- `0x10` = 位置模式 + 机体系
- `0x01` = 速度模式 + 世界系
- `0x11` = 速度模式 + 机体系

---

## 基础移动动作

### 相对移动（增量模式，机体系）

- `movex(x)`
  - 说明：机体系前进移动（NED body X），分步执行
  - 参数：`x` (m) - 前进距离（正值=前进，负值=后退）
  - 使用：`_send_setpoint(0x20, 0x01, x, 0.0, 0.0, 0.0)`

- `movey(y)`
  - 说明：机体系右移（NED body Y），分步执行
  - 参数：`y` (m) - 右移距离（正值=右移，负值=左移）
  - 使用：`_send_setpoint(0x20, 0x02, 0.0, y, 0.0, 0.0)`

- `movez(z)`
  - 说明：深度调整（世界系 Z），分步执行
  - 参数：`z` (m) - 深度增量（正值=下潜，负值=上浮）
  - 使用：`_send_setpoint(0x20, 0x04, 0.0, 0.0, z, 0.0)`

- `moverz(rz)`
  - 说明：偏航旋转（世界系 Yaw），分步执行
  - 参数：`rz` (deg) - 旋转角度（正值=顺时针，负值=逆时针）
  - 使用：`_send_setpoint(0x20, 0x08, 0.0, 0.0, 0.0, rz)`

- `movexy(x, y)`
  - 说明：XY 平面移动（机体系），分步执行
  - 参数：`x`, `y` (m) - 前进和右移距离
  - 使用：`_send_setpoint(0x20, 0x03, x, y, 0.0, 0.0)`

- `movexyz(x, y, z)`
  - 说明：3D 移动（机体系），分步执行
  - 参数：`x`, `y`, `z` (m)
  - 使用：`_send_setpoint(0x20, 0x07, x, y, z, 0.0)`

### 快速移动（不分步）

- `fast_movex(x)` - 快速前进
- `fast_movey(y)` - 快速右移
- `fast_movez(z)` - 快速深度调整
- `fast_moverz(rz)` - 快速旋转

### 绝对位置设置

- `setz(z)`
  - 说明：设置绝对深度（世界系）
  - 参数：`z` (m) - 目标深度（正值=下潜）
  - 使用：`_send_setpoint(0x00, 0x04, 0.0, 0.0, z, 0.0)`

- `setrz(rz)`
  - 说明：设置绝对航向角（世界系）
  - 参数：`rz` (deg) - 目标航向（0°=朝北）
  - 使用：`_send_setpoint(0x00, 0x08, 0.0, 0.0, 0.0, rz)`

---

## 复合移动动作

### 世界坐标系移动

- `move_world_step(dx, dy, dz)`
  - 说明：世界坐标系步进移动
  - 参数：`dx`, `dy`, `dz` (m) - 世界系增量

- `mttpos(x, y, z, rz, dx)`
  - 说明：移动至指定世界坐标位置
  - 参数：
    - `x, y, z` (m) - 目标世界坐标（NED）
    - `rz` (deg) - 目标航向
    - `dx` (m) - X 方向微调偏移

- `mttzpos(x, y, z, dx)`
  - 说明：移动至指定世界坐标位置，最后不旋转
  - 参数：同 `mttpos`

### 寄存点操作

- `setp()`
  - 说明：保存当前位置到 `backpoint`（寄存点）
  - 参数：无

- `back()` / `fast_back()`
  - 说明：回到先前 `backpoint` 寄存点
  - 参数：无

---

## 视觉搜索动作

- `search(name, cam)`
  - 说明：目标检测/定位流程，调用检测服务，获取目标三维位置并写入 `self.target`
  - 参数：
    - `name` (str) - 目标名称（如 "gate", "blue_drump"）
    - `cam` (str) - 摄像头标识 ("front" 或 "down")
  - 返回：目标是否检测成功

- `search2(name, cam, timeout)`
  - 说明：多帧融合搜索，提高定位精度
  - 参数：同 `search`，增加 `timeout` 超时时间

- `search4(name, cam, timeout)`
  - 说明：更多帧融合搜索，最高精度
  - 参数：同 `search2`

---

## 任务控制动作

- `start()`
  - 说明：初始化任务运行环境，发送心跳，等待解锁
  - 参数：无

- `end()`
  - 说明：清理结束，停止发送心跳
  - 参数：无

- `run(task)`
  - 说明：执行单个任务字典（由 `uv_tasks.json` 指定）
  - 参数：`task` (dict) - 任务定义字典

---

## 特殊动作

### 抓取相关

- `graball(color, depth, timeout, pr, k, step_time)`
  - 说明：抓取球类目标的高层序列
  - 参数：颜色、深度、超时等

- `grab_golf(kind, dx, dy, down_depth, up_depth)`
  - 说明：抓高尔夫球类动作

- `throw_golf(dy, depth)`
  - 说明：投掷高尔夫相关

### 门/障碍相关

- `pass_door(depth)`
  - 说明：通过门时的深度控制与通过策略
  - 参数：`depth` (m) - 通过时的深度

- `pass_gate_avoiding_obstacles(goal_x, goal_y, goal_z)`
  - 说明：避障过门，使用 A* 路径规划
  - 参数：目标世界坐标

### 巡线相关

- `line_qd()`
  - 说明：巡线起领点动作

- `line(ys_dep)`
  - 说明：巡线任务
  - 参数：`ys_dep` - 巡线深度

### 辅助动作

- `stop()`
  - 说明：停止当前运动
  - 参数：无

- `move_wait()`
  - 说明：等待并监测当前位置误差直到到达目标或超时
  - 参数：无

- `led(led0, led1)`
  - 说明：控制 LED 灯
  - 参数：`led0`, `led1` - LED 状态

- `delay(t)`
  - 说明：延迟等待
  - 参数：`t` (s) - 延迟时间

- `pow(s)`
  - 说明：机械爪（夹爪）控制
  - 参数：`s` - 状态/命令

---

## 任务 JSON 格式

任务定义文件位于 `datas/` 目录（如 `uv_tasks.json`），格式示例：

```json
{
  "tasks": [
    {
      "name": "task1",
      "actions": [
        {"action": "movex", "params": [1.0]},
        {"action": "move_wait"},
        {"action": "search", "params": ["gate", "front"]},
        {"action": "moverz", "params": [90]}
      ]
    }
  ]
}
```

---

## 调试命令

通过 `get_act()` 方法支持交互式调试：

```bash
# 启动 automaton 后，在终端输入命令
movex 1.0      # 前进 1m
movey -0.5     # 左移 0.5m
movez -1.0     # 上浮 1m
moverz 90      # 顺时针旋转 90°
setz -2.0      # 设置深度为 2m
search gate front  # 搜索前视门
```

---

## 注意事项

1. **增量模式**：`movex`、`movey` 等相对移动使用增量模式（`control_key & 0x20`），bridge 会自动将机体系偏移旋转到世界系
2. **分步执行**：大部分移动动作会分步执行，每次移动距离由 `Step` 字典控制
3. **超时处理**：`move_wait()` 有超时机制，超时后会继续执行下一步
4. **坐标系**：所有动作统一使用 NED 坐标系
5. **解锁状态**：发送控制指令前必须确保已解锁（`is_armed == true`）
