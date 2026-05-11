# 从串口协议迁移到 micro-ROS 指南

本文档说明如何将原有的串口协议代码迁移到新的 micro-ROS 接口。

## 快速对照表

### 1. 推力控制

**原始代码（串口）：**
```python
# uv_hmu.py
def openloop_thrust_callback(self, data):
    buff = b"\xfa\xaf\x02" + \
        struct.pack("<ffffff", data.x, data.y, data.z,
                    data.rx, data.ry, data.rz) + b"\xfb\xbf"
    self.usb_writer.write(buff)
```

**新代码（micro-ROS）：**
```python
# 使用 Zit6Controller
controller.set_force(
    fx=data.x,
    fy=data.y,
    fz=data.z,
    fyaw=data.rz,
    frame='body'
)
```

---

### 2. 位置控制

**原始代码（串口）：**
```python
# uv_hmu.py
def target_pos_down_callback(self, data):
    buff = b"\xfa\xaf\x03" + \
        struct.pack("<Bffffff", data.cs, data.pos.x, data.pos.y,
                    data.pos.z, data.pos.rx, data.pos.ry, data.pos.rz) + b"\xfb\xbf"
    self.usb_writer.write(buff)
```

**新代码（micro-ROS）：**
```python
# 世界坐标系
controller.move_to_position(
    x=data.pos.x,
    y=data.pos.y,
    z=data.pos.z,
    yaw=data.pos.rz
)

# 机体坐标系（增量）
controller.move_relative(
    dx=data.pos.x,
    dy=data.pos.y,
    dz=data.pos.z,
    dyaw=data.pos.rz
)
```

---

### 3. 速度控制

**原始代码（串口）：**
```python
# uv_hmu.py
def target_speed_down_callback(self, data):
    buff = b"\xfa\xaf\x08" + \
        struct.pack("<ff", data.pos.x, data.pos.y) + b"\xfb\xbf"
    self.usb_writer.write(buff)
```

**新代码（micro-ROS）：**
```python
controller.set_velocity(
    vx=data.pos.x,
    vy=data.pos.y,
    vz=0.0,
    vyaw=0.0
)
```

---

### 4. PID 控制器开关

**原始代码（串口）：**
```python
# uv_hmu.py
def pid_controllers_set_callback(self, data):
    buff = b"\xfa\xaf\x01" + \
        struct.pack("<BBBBBB", data.x, data.y, data.z,
                    data.rx, data.ry, data.rz) + b"\xfb\xbf"
    self.usb_writer.write(buff)
```

**新代码（micro-ROS）：**
```python
# 不再需要手动控制 PID 开关
# 控制层级会根据 control_key 自动切换
# POSITION 模式 -> 位置环 + 速度环
# VELOCITY 模式 -> 速度环
# FORCE 模式 -> 直接推力
```

---

### 5. DVL 控制

**原始代码（串口）：**
```python
# uv_hmu.py
def dvl_set_callback(self, data):
    buff = b"\xfa\xaf\x04" + \
        struct.pack("<B", data.dvl) + b"\xfb\xbf"
    self.usb_writer.write(buff)
```

**新代码（micro-ROS）：**
```python
# 开启 DVL
controller.dvl_on()

# 关闭 DVL
controller.dvl_off()

# 或使用通用接口
controller.send_ins_command(INSCommand.DVL_ON)
```

---

### 6. 舵机控制

**原始代码（串口）：**
```python
# uv_hmu.py
def servo_control_callback(self, data):
    buff = b"\xfa\xaf\x06" + \
        struct.pack("<Bf", data.num, data.angle) + b"\xfb\xbf"
    self.usb_writer.write(buff)
```

**新代码（micro-ROS）：**
```python
# 当前固件未实现舵机控制话题
# 如需添加，可扩展 /zit6/cmd/servo 话题
```

---

### 7. IMU 数据读取

**原始代码（串口）：**
```python
# 从串口读取二进制数据并解析
success, buff = self.usb_reader.read(length)
# 手动解析 struct
```

**新代码（micro-ROS）：**
```python
# 订阅话题自动接收
pos = controller.get_position()  # [x, y, z, yaw]
vel = controller.get_velocity()  # [vx, vy, vz, vyaw]

# 或直接订阅原始话题
def pos_callback(msg):
    x, y, z, yaw = msg.data[:4]

controller.create_subscription(
    Float32MultiArray, '/zit6/state/pos', pos_callback, 10)
```

---

## 完整迁移示例

### 原始 uv_automaton.py 节点

```python
class CoreNode(Node):
    def __init__(self, name, opt):
        super().__init__(name)
        
        # 串口初始化
        self.usb_reader = Serial.TtyReader('/dev/ttyUSB0')
        self.usb_writer = Serial.TtyWriter('/dev/ttyUSB0')
        
        # 订阅控制话题
        self.create_subscription(
            RobotAxis, 'openloop_thrust', self.thrust_callback, 10)
        
        # 读取 IMU 线程
        self.imu_thread = threading.Thread(target=self.read_imu)
        self.imu_thread.start()
    
    def thrust_callback(self, data):
        # 打包二进制数据
        buff = b"\xfa\xaf\x02" + struct.pack("<ffffff", ...) + b"\xfb\xbf"
        self.usb_writer.write(buff)
    
    def read_imu(self):
        while True:
            success, buff = self.usb_reader.read(128)
            # 解析数据...
```

### 迁移后的代码

```python
from uv_control_py.Zit6Controller import Zit6Controller, AxisMask

class CoreNode(Node):
    def __init__(self, name, opt):
        super().__init__(name)
        
        # 使用 Zit6Controller（内部处理所有通信）
        self.controller = Zit6Controller('zit6_ctrl')
        
        # 订阅控制话题（保持原有接口）
        self.create_subscription(
            RobotAxis, 'openloop_thrust', self.thrust_callback, 10)
        
        # 不再需要 IMU 读取线程
        # 数据通过 controller.get_position() 获取
    
    def thrust_callback(self, data):
        # 直接调用控制接口
        self.controller.set_force(
            fx=data.x, fy=data.y, fz=data.z, fyaw=data.rz
        )
    
    def get_current_position(self):
        # 获取位置数据
        return self.controller.get_position()
```

---

## 初始化流程对比

### 原始流程

```python
# 1. 打开串口
usb_reader = Serial.TtyReader('/dev/ttyUSB0')
usb_writer = Serial.TtyWriter('/dev/ttyUSB0')

# 2. 下发 PID 参数
pid.hwinit()

# 3. 下发推力曲线
curve.hwinit()

# 4. 等待 IMU 数据
while not imu_ready:
    read_and_parse_imu()

# 5. 开始控制
send_control_command()
```

### 新流程

```python
# 1. 创建控制器（自动连接 micro-ROS）
controller = Zit6Controller('my_controller')

# 2. 等待导航就绪
while not controller.is_navigation_ready():
    rclpy.spin_once(controller, timeout_sec=0.1)

# 3. 位置清零
controller.zero_position()

# 4. 解锁
controller.arm()

# 5. 开始控制
controller.move_to_position(1.0, 0.0, 0.0, 0.0)
```

---

## 常见问题

### Q1: 如何保持与原有代码的兼容性？

**方案 1：适配器模式**

创建一个适配器类，将原有的 ROS 话题转换为 Zit6Controller 调用：

```python
class Zit6Adapter(Node):
    def __init__(self):
        super().__init__('zit6_adapter')
        self.controller = Zit6Controller('zit6_ctrl')
        
        # 订阅原有话题
        self.create_subscription(
            RobotAxis, 'openloop_thrust', self.thrust_adapter, 10)
        self.create_subscription(
            TargetPosDown, 'target_pos_down', self.pos_adapter, 10)
        
        # 发布原有话题（从 micro-ROS 数据转换）
        self.motion_pub = self.create_publisher(
            RobotMotionController, 'motion_controller', 10)
        
        self.create_timer(0.1, self.publish_motion)
    
    def thrust_adapter(self, msg):
        self.controller.set_force(msg.x, msg.y, msg.z, msg.rz)
    
    def pos_adapter(self, msg):
        self.controller.move_to_position(
            msg.pos.x, msg.pos.y, msg.pos.z, msg.pos.rz)
    
    def publish_motion(self):
        # 将 micro-ROS 数据转换为原有消息格式
        pos = self.controller.get_position()
        vel = self.controller.get_velocity()
        
        msg = RobotMotionController()
        msg.x, msg.y, msg.z = pos[:3]
        msg.vx, msg.vy, msg.vz = vel[:3]
        self.motion_pub.publish(msg)
```

**方案 2：逐步迁移**

1. 保留原有节点
2. 新增 Zit6Controller 节点
3. 通过 ROS 话题桥接
4. 逐步替换原有代码

---

### Q2: PID 参数如何配置？

新固件的 PID 参数在 STM32 端配置，不再通过串口下发。

如需修改参数：
1. 修改 `ChassisManager.cpp` 中的 PID 参数
2. 重新编译固件
3. 烧录到 STM32

未来可扩展参数配置话题（如 `/zit6/cmd/pid_config`）。

---

### Q3: 推力曲线如何配置？

同 PID 参数，推力曲线在固件端配置。

如需修改：
1. 修改 `VIT6_Link` 或相关模块
2. 重新编译固件

---

### Q4: 如何调试通信问题？

```bash
# 1. 检查 micro-ROS Agent 连接
micro-ros-agent serial --dev /dev/ttyUSB0 -b 115200

# 2. 查看话题列表
ros2 topic list

# 3. 监控心跳
ros2 topic echo /zit6/heartbeat

# 4. 监控位置
ros2 topic echo /zit6/state/pos

# 5. 手动发送控制指令
ros2 topic pub /zit6/cmd/setpoint zit6_interfaces/msg/ZitSetpoint \
  "{control_key: 0, type_mask: 15, x: 1.0, y: 0.0, z: 0.0, yaw: 0.0}"

# 6. 运行快速测试
python3 zit6_quick_test.py
```

---

## 迁移检查清单

- [ ] 安装 zit6_interfaces 包
- [ ] 编译 uv_control_py 包
- [ ] 启动 micro-ROS Agent
- [ ] 运行 zit6_quick_test.py 验证连接
- [ ] 修改原有代码，替换串口调用为 Zit6Controller
- [ ] 测试基本控制功能
- [ ] 测试 ARM/DISARM 流程
- [ ] 测试 DVL 控制
- [ ] 测试位置清零
- [ ] 完整系统测试

---

## 性能对比

| 指标 | 串口协议 | micro-ROS |
|-----|---------|-----------|
| 延迟 | ~5-10ms | ~5-10ms |
| 可靠性 | 中等（需手动校验） | 高（DDS 保证） |
| 调试难度 | 高（二进制数据） | 低（可读消息） |
| 扩展性 | 低（需修改协议） | 高（添加话题） |
| 学习曲线 | 陡峭 | 平缓（ROS2 标准） |

---

## 总结

micro-ROS 迁移的主要优势：

1. **标准化** - 使用 ROS2 生态系统
2. **易调试** - 可视化工具丰富
3. **类型安全** - 消息定义明确
4. **易扩展** - 添加新功能简单
5. **社区支持** - ROS2 社区庞大

建议：
- 新项目直接使用 micro-ROS
- 旧项目可通过适配器逐步迁移
- 保留原有代码作为备份
