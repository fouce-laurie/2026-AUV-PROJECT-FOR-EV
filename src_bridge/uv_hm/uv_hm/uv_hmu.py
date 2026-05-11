# 水下机器人硬件管理单元
# "Underwater Vehicle Hardware Management Unit


import rclpy
from rclpy.node import Node
import time
import struct
import threading
import argparse
import sys

from rclpy.utilities import remove_ros_args  # 导入ROS参数过滤工具

from uv_msgs.msg import RobotDeviceManager  # 机器人设备管理器
from uv_msgs.msg import RobotMotionController  # 机器人运动控制器


from uv_msgs.msg import PidParams
from uv_msgs.msg import PidControllers
from uv_msgs.msg import PidControllersState

from uv_msgs.msg import ThrustCurve
from uv_msgs.msg import ThrustCurves

from uv_msgs.msg import RobotAxis
from uv_msgs.msg import ServoSet
from uv_msgs.msg import TargetPosDown
from uv_msgs.msg import ImuData
from uv_msgs.msg import LedControllers
from uv_msgs.msg import MagnetController


from uv_control_py import Pid
from uv_control_py import Curve
from uv_control_py import Serial

#                              _ooOoo_
#                             o8888888o
#                             88" . "88
#                             (| -_- |)
#                              O\ = /O
#                           ____/`---'\____
#                        .   ' \\| |// `.
#                         / \\||| : |||// \
#                        / _||||| -:- |||||- \
#                         | | \\\ - /// | |
#                       | \_| ''\---/'' | |
#                        \ .-\__ `-` ___/-. /
#                    ___`. .' /--.--\ `. . __
#                  ."" '< `.___\_<|>_/___.' >'"".
#                 | | : `- \`.;`\ _ /`;.`/ - ` : | |
#                    \ \ `-. \_ __\ /__ _/ .-` / /
#           ======`-.____`-.___\_____/___.-`____.-'======
#                              `=---='
#
#           .............................................
#                     佛祖保佑             永无BUG


class CoreNode(Node):
    def __init__(self, name, maintty, pid_path, curve_path):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)

        self.MotionController = RobotMotionController()
        self.DeviceManager = RobotDeviceManager()

        # 话题发布
        # 创建话题发布 motion_controller ，定义其中的消息类型为 RobotMotionController
        self.motion_controller_pub = self.create_publisher(
            RobotMotionController, "motion_controller", 10)
        # 创建话题发布 device_manager ，定义其中的消息类型为 ImuData
        self.device_manager_pub = self.create_publisher(
            RobotDeviceManager, "device_manager", 10)

        # 创建话题发布 pid_controllers ，定义其中的消息类型为 PidParameters
        self.pid_controllers_pub = self.create_publisher(
            PidControllers, "pid_controllers", 10)
        # 创建话题发布 pid_controllers ，定义其中的消息类型为 PidParameters
        self.curves_pub = self.create_publisher(
            ThrustCurves, "curves", 10)

        # 话题接收
        # 创建话题接收 openloop_thrust ，定义其中的消息类型为 RobotAxis
        self.create_subscription(
            RobotAxis, 'openloop_thrust', self.openloop_thrust_callback, 10)
        # 创建话题接收 servo_control ，定义其中的消息类型为 ServoSet
        self.create_subscription(
            ServoSet, 'servo_control', self.servo_control_callback, 10)
        # 创建话题接收 pid_param_set ，定义其中的消息类型为 PidParams
        self.create_subscription(
            PidParams, 'pid_params_set', self.pid_params_set_callback, 10)
        # 创建话题接收 pid_controllers_set ，定义其中的消息类型为 PidControllersState
        self.create_subscription(
            PidControllersState, 'pid_controllers_set', self.pid_controllers_set_callback, 10)
        # 创建话题接收 target_pos_down ，定义其中的消息类型为 TargetPosDown
        self.create_subscription(
            TargetPosDown, 'target_pos_down', self.target_pos_down_callback, 10)
        # 创建话题接收 dvl_set ，定义其中的消息类型为 ImuData
        self.create_subscription(
            ImuData, 'dvl_set', self.dvl_set_callback, 10)
        # 创建话题接收 thrust_curve_set ，定义其中的消息类型为 ThrustCurve
        self.create_subscription(
            ThrustCurve, 'thrust_curve_set', self.thrust_curve_set_callback, 10)
        # 创建话题接收 Target_speed_down ，定义其中的消息类型为 TargetPosDown
        self.create_subscription(
            TargetPosDown, 'Target_speed_down', self.target_speed_down_callback, 10)
        #创建话题接收 led_controllers , 定义其中的消息类型为 LedControllers
        self.create_subscription(
            LedControllers, 'led_controllers', self.led_controllers_callback, 10)
        # 创建话题接收 magnet_controller ，定义其中的消息类型为 MagnetController
        self.create_subscription(
            MagnetController, 'magnet_controller', self.magnet_controller_callback, 10)           


        # 串口读写（单MCU）
        self.usb_reader = Serial.TtyReader(maintty)
        self.usb_writer = Serial.TtyWriter(maintty)
        # pid参数
        self.pid = Pid.PID(self.usb_writer, pid_path)
        # 推力曲线
        self.curve = Curve.CURVE(self.usb_writer, curve_path)

    def openloop_thrust_callback(self, data):
        buff = b"\xfa\xaf\x02" + \
            struct.pack("<ffffff", data.x, data.y, data.z,
                        data.rx, data.ry, data.rz)+b"\xfb\xbf"
        self.usb_writer.write(buff)

    def servo_control_callback(self, data):
        if data.angle > 1:
            data.angle = 1.0
        elif data.angle < 0:
            data.angle = 0.0

        buff = b"\xfa\xaf\x06" + \
            struct.pack("<Bf", data.num, data.angle)+b"\xfb\xbf"
        self.usb_writer.write(buff)

    def led_controllers_callback(self, data):
        buff = b"\xfa\xaf\x08" + \
            struct.pack("<BB", data.led0, data.led1)+b"\xfb\xbf"
        self.usb_writer.write(buff)

    def magnet_controller_callback(self, data):
        buff = b"\xfa\xaf\x09" + \
            struct.pack("<B", data.state)+b"\xfb\xbf"
        self.usb_writer.write(buff)


    def pid_params_set_callback(self, data):  # pid参数设置
        self.pid.topicrec(data)

        # 参数下行
        self.pid.hwinit()
        # 参数保存
        self.pid.filesave()
        self.get_logger().info("已保存PID参数信息")

    def thrust_curve_set_callback(self, data):
        self.curve.topicrec(data)

        # 参数下行
        self.curve.hwinit()
        # 参数保存
        self.curve.filesave()
        self.get_logger().info("已保存推力曲线")

    def target_pos_down_callback(self, data):  # 设置目标位置
        buff = b"\xfa\xaf\x03"+struct.pack("<Bffffff", data.cs, data.pos.x, data.pos.y,
                                           data.pos.z, data.pos.rx, data.pos.ry, data.pos.rz)+b"\xfb\xbf"
        self.usb_writer.write(buff)

    def pid_controllers_set_callback(self, data):  # 设置PID控制器工作状态
        buff = b"\xfa\xaf\x01" + \
            struct.pack("<BBBBBB", data.x, data.y, data.z,
                        data.rx, data.ry, data.rz)+b"\xfb\xbf"
        self.usb_writer.write(buff)
        
        
        
    def dvl_set_callback(self, data):  # 设置dvl工作状态
        buff = b"\xfa\xaf\x04" + \
            struct.pack("<B", data.dvl,)+b"\xfb\xbf"
        self.usb_writer.write(buff)
    
    def target_speed_down_callback(self, data): #设置目标速度
        buff =  b"\xfa\xaf\x08"+struct.pack("<ff",  data.pos.x, data.pos.y)+b"\xfb\xbf"
        self.usb_writer.write(buff)
    
    
    def parameters_init(self):
        # pid控制器硬件初始化
        self.pid.hwinit()
        self.get_logger().info("pid 参数已写入")
        # 推力曲线初始化
        self.curve.hwinit()
        self.get_logger().info("推力曲线已写入")


def TTyRead(tty_reader, node, name):
    node.get_logger().info("%s串口监听已启动" % name)
    while rclpy.ok():
        s, buff = tty_reader.read(1)
        # 校验帧头
        if s and buff == b"\xfa":
            s, buff = tty_reader.read(1)
            if s and buff == b"\xaf":
                count, buff = 0, bytes()
                while rclpy.ok():
                    count += 1
                    s, b = tty_reader.read(1)
                    if s:
                        buff += b
                        # 校验帧尾
                        if len(buff) > 1 and buff[-1] == 0xbf and buff[-2] == 0xfb:
                            # 接收成功后解码
                            # 设备管理器
                            if buff[0] == 0x01 and len(buff[1:-2]) == 22:  # 校验长度
                                node.DeviceManager.leak, node.DeviceManager.vol, node.DeviceManager.tem, \
                                    node.DeviceManager.hum, node.DeviceManager.magnet, node.DeviceManager.angle[0],  node.DeviceManager.angle[1]= struct.unpack(
                                        "<BfffBff", buff[1:-2])
                                node.device_manager_pub.publish(
                                    node.DeviceManager)
                                # node.get_logger().info(name+":接收到设备管理器信息")
                            # 运动控制器
                            elif buff[0] == 0x00 and len(buff[1:-2]) >= 154:  # 校验长度
                                # 机器人位置 1~24
                                node.MotionController.pos.x, node.MotionController.pos.y, node.MotionController.pos.z, \
                                    node.MotionController.pos.rx, node.MotionController.pos.ry, node.MotionController.pos.rz = struct.unpack(
                                        "<ffffff", buff[1:25])
                                # 目标位置(base) 25~48
                                node.MotionController.tpos_inbase.x, node.MotionController.tpos_inbase.y, node.MotionController.tpos_inbase.z, \
                                    node.MotionController.tpos_inbase.rx, node.MotionController.tpos_inbase.ry, node.MotionController.tpos_inbase.rz = struct.unpack(
                                        "<ffffff", buff[25:49])
                                # 目标位置(world) 49~72
                                node.MotionController.tpos_inworld.x, node.MotionController.tpos_inworld.y, node.MotionController.tpos_inworld.z, \
                                    node.MotionController.tpos_inworld.rx, node.MotionController.tpos_inworld.ry, node.MotionController.tpos_inworld.rz = struct.unpack(
                                        "<ffffff", buff[49:73])
                                # PID控制器状态 73~78
                                node.MotionController.pidstate.x, node.MotionController.pidstate.y, node.MotionController.pidstate.z, \
                                    node.MotionController.pidstate.vx, node.MotionController.pidstate.vy, node.MotionController.pidstate.rz, = struct.unpack(
                                        "<BBBBBB", buff[73:79])
                                # IMU数据 79~128
                                node.MotionController.imu.mode, node.MotionController.imu.dvl,\
                                    node.MotionController.imu.pos.x, node.MotionController.imu.pos.y, node.MotionController.imu.pos.z, \
                                    node.MotionController.imu.pos.rx, node.MotionController.imu.pos.ry, node.MotionController.imu.pos.rz, \
                                    node.MotionController.imu.spd.x, node.MotionController.imu.spd.y, node.MotionController.imu.spd.z, \
                                    node.MotionController.imu.spd.rx, node.MotionController.imu.spd.ry, node.MotionController.imu.spd.rz = struct.unpack(
                                        "<BBffffffffffff", buff[79:129])
                                # 推力数据 129~152
                                node.MotionController.thrust.thrust[0], node.MotionController.thrust.thrust[1], node.MotionController.thrust.thrust[2], \
                                    node.MotionController.thrust.thrust[3], node.MotionController.thrust.thrust[4], node.MotionController.thrust.thrust[5], = struct.unpack(
                                        "<ffffff", buff[129:153])
                                node.motion_controller_pub.publish(
                                    node.MotionController)
                                #LED数据 153~154
                                node.DeviceManager.led[0], node.DeviceManager.led[1] = struct.unpack(
                                        "<BB", buff[153:155])
                                node.device_manager_pub.publish(
                                    node.DeviceManager)
                                    
                                # node.get_logger().info(name+":接收到运动控制器信息")

                            # 接收成功后退出
                            buff = bytes()
                            break
                    if count > 200:  # 接收超时后退出
                        buff = bytes()
                        break


def ParameterBroadcast(node):  # 参数广播
    while rclpy.ok():
        node.pid_controllers_pub.publish(node.pid.pid)
        time.sleep(0.05)
        node.curves_pub.publish(node.curve.curves)
        time.sleep(0.05)


def main(args=None):

    ros_args = remove_ros_args(sys.argv)
    # 加载参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--pid-path', nargs='+', type=str, default=[
                        '/home/nvidia/Workspace/Cruise/datas/pid_parameters.json'], help='PID参数路径')
    parser.add_argument('--curve-path', nargs='+', type=str, default=[
                        '/home/nvidia/Workspace/Cruise/datas/thrust_cureves.json'], help='推力曲线路径')
    parser.add_argument('--tty-path', nargs='+', type=str,
                        default=['/dev/ttyUSB0'], help='串口路径')
    opt = parser.parse_args(ros_args[1:])

    rclpy.init(args=args)  # 初始化rclpy
    node = CoreNode("uv_core", opt.tty_path[0],
                    opt.pid_path[0], opt.curve_path[0])  # 新建一个节点

    node.parameters_init()  # 写入参数

    # 创建串口接收线程
    thread_usb_read = threading.Thread(
        target=TTyRead, args=(node.usb_reader, node, "ttyUSB0"))
    # 创建参数发布线程
    thread_parameter_broadcast = threading.Thread(
        target=ParameterBroadcast, args=(node,))

    # 启动线程
    thread_usb_read.start()
    thread_parameter_broadcast.start()

    rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+Z）
    rclpy.shutdown()  # 关闭rclpy
