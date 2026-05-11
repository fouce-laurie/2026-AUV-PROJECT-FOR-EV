# 任务/视觉/控制相关技能（在此聚合，便于后续扩展）
import time
from math import atan2, sqrt

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from filterpy.kalman import KalmanFilter

from uv_msgs.msg import LedControllers
from uv_msgs.msg import MagnetController
from uv_msgs.msg import PidControllersState
from uv_msgs.msg import ServoSet
from uv_msgs.msg import TargetPosDown
from uv_msgs.srv import DetectRequest
from sensor_msgs.msg import Image

from .config import RAD2DEG


class TaskSkills:
    def search(self, name, cam):
        """Calls detect service and updates self.target with averaged pose."""
        # 定位目标
        # 等待服务段上线
        wait = True
        while rclpy.ok() and self.detect_request_client.wait_for_service(0.1) == False:
            if wait:
                wait = False
                self.get_logger().info("Info:等待服务端上线....")
        self.get_logger().info("Info:服务端已启动")

        request = DetectRequest.Request()
        request.stero = cam
        request.target = name
        responce = None

        pos_list = []
        fail_cnt = 0

        image = False

        while True:
            if cam == "front" and self.front_cam_Image_data != None:
                request.imagein = self.front_cam_Image_data
                image = True
            if cam == "down" and self.down_cam_Image_data != None:
                request.imagein = self.down_cam_Image_data
                image = True

            if image == True:
                responce = self.detect_request_client.call(request)
                if responce.s == 1:
                    if cam == "front":
                        self.front_cam.target_inbase.vector.x = responce.x
                        self.front_cam.target_inbase.vector.y = responce.y
                        self.front_cam.target_inbase.vector.z = responce.z
                        #相机坐标系到机器人坐标系
                        self.front_cam.base2world()

                        self.robot.target_inbase.vector.x = self.front_cam.target_inworld.vector.x
                        self.robot.target_inbase.vector.y = self.front_cam.target_inworld.vector.y
                        self.robot.target_inbase.vector.z = self.front_cam.target_inworld.vector.z
                        self.robot.target_inbase.vector.rx = self.front_cam.target_inworld.vector.rx
                        self.robot.target_inbase.vector.ry = self.front_cam.target_inworld.vector.ry
                        self.robot.target_inbase.vector.rz = self.front_cam.target_inworld.vector.rz
                        #机器人坐标系到世界坐标系
                        self.robot.base2world()

                    if cam == "down":
                        self.down_cam.target_inbase.vector.x = responce.x
                        self.down_cam.target_inbase.vector.y = responce.y
                        self.down_cam.target_inbase.vector.z = responce.z
                        self.down_cam.base2world()

                        self.robot.target_inbase.vector.x = self.down_cam.target_inworld.vector.x
                        self.robot.target_inbase.vector.y = self.down_cam.target_inworld.vector.y
                        self.robot.target_inbase.vector.z = self.down_cam.target_inworld.vector.z
                        self.robot.target_inbase.vector.rx = self.down_cam.target_inworld.vector.rx
                        self.robot.target_inbase.vector.ry = self.down_cam.target_inworld.vector.ry
                        self.robot.target_inbase.vector.rz = self.down_cam.target_inworld.vector.rz
                        self.robot.base2world()

                    # self.get_logger().info(
                    #     f"Info:目标在机器人坐标系中的位置: x: {self.robot.target_inbase.vector.x: .3f} y: {self.robot.target_inbase.vector.y: .3f} z: {self.robot.target_inbase.vector.z: .3f}")
                    self.get_logger().info(
                        f"Info:目标在相机坐标系中的位置: x: {responce.x: .3f} y: {responce.y: .3f} z: {responce.z: .3f}")

                    pos_list.append([self.robot.target_inworld.vector.x,
                                    self.robot.target_inworld.vector.y, self.robot.target_inworld.vector.z])

                    if len(pos_list) >= 5:
                        break
                else:
                    fail_cnt += 1
                    time
                    if fail_cnt >= 1000 & len(pos_list) < 5:
                        self.get_logger().info(f"确认无目标，退出函数")
                        return responce.s

            else:
                self.get_logger().info("无图像传入")

            image = False
        x = y = z = 0
        for i in pos_list:
            x += i[0]
            y += i[1]
            z += i[2]
        x /= len(pos_list)
        y /= len(pos_list)
        z /= len(pos_list)

        self.robot.base2world()
        self.target["name"] = name
        self.target["x"] = x
        self.target["y"] = y
        self.target["z"] = z

        self.get_logger().info(
            f"目标位置: x : {self.target['x']:.2f} y: {self.target['y']:.2f} z: {self.target['z']:.2f}")
        return responce.s

    def search2(self, name, cam,dy,dz,z_target,times):
        """Repeated search with averaging and step adjustments."""
        # 定位目标
        # 等待服务段上线
        wait = True
        while rclpy.ok() and self.detect_request_client.wait_for_service(0.1) == False:
            if wait:
                wait = False
                self.get_logger().info("Info:等待服务端上线....")
        self.get_logger().info("Info:服务端已启动")

        request = DetectRequest.Request()
        request.stero = cam
        request.target = name

        pos_list = []

        image = False
        times_none = 0 #
        flag_to_next = 0 #

        while True:
            if cam == "front" and self.front_cam_Image_data != None:
                request.imagein = self.front_cam_Image_data
                image = True
            if cam == "down" and self.down_cam_Image_data != None:
                request.imagein = self.down_cam_Image_data
                image = True

            if image == True:
                responce = self.detect_request_client.call(request)
                if responce.s == 1:
                    if cam == "front":
                        self.front_cam.target_inbase.vector.x = responce.x
                        self.front_cam.target_inbase.vector.y = responce.y
                        self.front_cam.target_inbase.vector.z = responce.z
                        #相机坐标系到机器人坐标系
                        self.front_cam.base2world()

                        self.robot.target_inbase.vector.x = self.front_cam.target_inworld.vector.x
                        self.robot.target_inbase.vector.y = self.front_cam.target_inworld.vector.y
                        self.robot.target_inbase.vector.z = self.front_cam.target_inworld.vector.z
                        self.robot.target_inbase.vector.rx = self.front_cam.target_inworld.vector.rx
                        self.robot.target_inbase.vector.ry = self.front_cam.target_inworld.vector.ry
                        self.robot.target_inbase.vector.rz = self.front_cam.target_inworld.vector.rz
                        #机器人坐标系到世界坐标系
                        self.robot.base2world()

                    if cam == "down":
                        self.down_cam.target_inbase.vector.x = responce.x
                        self.down_cam.target_inbase.vector.y = responce.y
                        self.down_cam.target_inbase.vector.z = responce.z
                        self.down_cam.base2world()

                        self.robot.target_inbase.vector.x = self.down_cam.target_inworld.vector.x
                        self.robot.target_inbase.vector.y = self.down_cam.target_inworld.vector.y
                        self.robot.target_inbase.vector.z = self.down_cam.target_inworld.vector.z
                        self.robot.target_inbase.vector.rx = self.down_cam.target_inworld.vector.rx
                        self.robot.target_inbase.vector.ry = self.down_cam.target_inworld.vector.ry
                        self.robot.target_inbase.vector.rz = self.down_cam.target_inworld.vector.rz
                        self.robot.base2world()

                    self.get_logger().info(
                        f"Info:目标在世界坐标系中的位置: x: {self.robot.target_inworld.vector.x: .3f} y: {self.robot.target_inworld.vector.y: .3f} z: {self.robot.target_inworld.vector.z: .3f}")

                    pos_list.append([self.robot.target_inworld.vector.x,
                                    self.robot.target_inworld.vector.y, self.robot.target_inworld.vector.z])

                    if len(pos_list) >= 100:
                        break
                else:
                    if (times > 0):
                        times_none += 1
                        self.get_logger().info(f"第{times_none}次无目标")
                        if (times_none >= 50) & (len(pos_list) < 10):
                            flag_to_next = 1
                            self.get_logger().info(f"确认无目标，进行下一步")
                            break
            else:
                self.get_logger().info("无图像传入")

            image = False

        times += 1
        if flag_to_next == 0:
            x = y = z = 0
            for i in pos_list:
                x += i[0]
                y += i[1]
                z += i[2]
            x /= len(pos_list)
            y /= len(pos_list)
            z /= len(pos_list)

            self.robot.base2world()
            self.target["name"] = name
            self.target["x"] = x
            self.target["y"] = y
            self.target["z"] = z

            self.get_logger().info(
                f"目标位置: x : {self.target['x']:.2f} y: {self.target['y']:.2f} z: {self.target['z']:.2f}")
            
            self.mttz(dy,dz)
            self.setz(z_target)
            self.get_logger().info(
                f"开始第{times}次搜查是否抓球成功")            
            self.search2(self, name, cam,dy,dz,z_target,times)
    
    def search4(self, name1, name2,dx,dy,dz,z_target,rz_target,times,distance):#1为圈，2为T插
        """Composite search + normal estimation flow (mission-specific)."""
       
        times_none = 0
        flag_to_next = 0
        
        T_cnt = 0
        while True:
            
            s = self.search(self, "scaffolding", "front")
            if s == 0:
                self.get_logger().info("======无目标======")
                self.led(0, 0)
                self.get_logger().info("======旋转寻找目标目标======")
                self.moverz(40)
                T_cnt += 1
            else :
                self.get_logger().info("======发现基架======")  
                self.target["y"] -= 0.50  
                self.mttz(dy,dz)
                self.setrz(rz_target)
                break
            if T_cnt > 10:
                self.get_logger().info("======未找到目标======")
                break
        
        """
        计算法线方向
        """
        # 解包坐标
        x1, y1, z1 = self.cam2robot(2, 4,"front")
        x2, y2, z2 = self.cam2robot(2, 4,"front")
        x3, y3, z3 = self.cam2robot(2, 4,"front")
            
            
        # 计算向量AC的分量
        a = x1 - x3
        b = y1 - y3
            
        # 计算向量模长的平方
        length_squared = a**2 + b**2
            
            
        # 计算u和v的可能值
        if  abs(b) >= 0.001:
            # 一般情况
            factor = distance * abs(b) / (length_squared ** 0.5)
            u1 = factor
            v1 = -a * u1 / b
                
            u2 = -factor
            v2 = -a * u2 / b
        else:
            # b为0的特殊情况（AC垂直于x轴）
            u1 = 0
            v1 = distance
                
            u2 = 0
            v2 = -distance
            
            # 计算点D的坐标
        d1 = (x2 + u1, y2 + v1, z3)
        d2 = (x2 + u2, y2 + v2, z3)
            
        real_d = min(d1, d2)    
             
        
              
    # 机械爪控制
    def pow(self, s):
        """Controls the gripper servo angle (open/close)."""
        servo = ServoSet()
        servo.num = 0
        if s == 1:
            servo.angle = 0.35
            self.servo_control_pub.publish(servo)
        if s == 0:
            servo.angle = 0.92#推球，夹T插
            self.servo_control_pub.publish(servo)
        time.sleep(0.1)


    # 任务启动
    def start(self):
        """Initializes task: save pose, LED cues, gripper, enable PID."""
        time.sleep(5)
        #  载入当前目标值
        tpos = TargetPosDown()
        tpos.cs = 0
        self.start_pos.base.vector.x = tpos.pos.x = self.MotionController.pos.x
        self.start_pos.base.vector.y = tpos.pos.y = self.MotionController.pos.y
        tpos.pos.z = self.MotionController.pos.z
        self.start_pos.base.vector.z = 0.0
        self.start_pos.base.vector.rz = tpos.pos.rz = self.MotionController.pos.rz
        self.start_pos.base.vector.ry = self.start_pos.base.vector.rx = tpos.pos.rx = tpos.pos.ry =  0.0
        self.start_pos.base.extract()
        self.get_logger().info(f"载入当前世界坐标 x:{tpos.pos.x:.3f} y:{tpos.pos.y:.3f} z:{tpos.pos.z:.3f} rz:{tpos.pos.rz:.3f}")
        self.led(1,0)#绿灯
        time.sleep(3.0)
        self.led(0,1)#黄灯
        time.sleep(3.0)
        self.led(1,1)#红灯
        time.sleep(3.0)
        self.led(0,0)
        self.target_pos_down_pub.publish(tpos)
        # 机械爪加紧T插
        self.pow(0) 
        
        # 开启PID控制器
        pid = PidControllersState()
        pid.x = pid.y = pid.z = pid.rz = 1
        pid.rx = pid.ry = 0
        self.pid_controllers_set_pub.publish(pid)
        time.sleep(0.1)

    # 任务结束
    def end(self):
        """Disables PID controllers at the end of a mission."""
        # 关闭PID控制器
        pid = PidControllersState()
        pid.x = pid.y = pid.z = pid.rz = 0
        pid.rx = pid.ry = 0
        self.pid_controllers_set_pub.publish(pid)
        time.sleep(0.1)

    def run(self, task: dict):
        """Dispatches a task dict to the corresponding action method."""
        if task["name"] == "movexyz":
            self.movexyz(task["params"]["x"], task["params"]
                         ["y"], task["params"]["z"])

        elif task["name"] == "throw_golf":
            self.throw_golf(task["params"]["dy"],
                            task["params"]["depth"])
            
        elif task["name"] == "movexy":
            self.movexy(task["params"]["x"], task["params"]["y"])

            
        elif task["name"] == "movex":
            self.movex(task["params"]["x"])

        elif task["name"] == "movey":
            self.movey(task["params"]["y"])

        elif task["name"] == "movez":
            self.movez(task["params"]["z"])

        elif task["name"] == "moverz":
            self.moverz(task["params"]["rz"])

        elif task["name"] == "setz":
            self.setz(task["params"]["z"])

        elif task["name"] == "setrz":
            self.setrz(task["params"]["rz"])

        elif task["name"] == "search":
            self.search(task["params"]["name"], task["params"]["cam"])

        elif task["name"] == "mtty":
            self.mtty(task["params"]["y"], task["params"]["z"])
        elif task["name"] == "mttxf":
            self.mttxf(task["params"]["dx"],task["params"]["dy"])
        elif task["name"] == "mttz":
            self.mttz(task["params"]["y"], task["params"]["z"])
        elif task["name"] == "mttzxy":
            self.mttzxy(task["params"]["dz"], task["params"]["dx"], task["params"]["dy"])
        elif task["name"] == "setp":
            self.setp()

        elif task["name"] == "back":
            self.back()
            
        elif task["name"] == "backy":
            self.backy()
            
        elif task["name"] == "pow":
            self.pow(task["params"])
        
        elif task["name"] == "line":
            self.line(task["params"]["ys_dep"])

        elif task["name"] == "graball":
            self.graball(task["params"]["color"], task["params"]["depth"], task["params"]["timeout"],task["params"]["pr"],task["params"]["k"],task["params"]["step_time"])

        elif task["name"] == "thrball":
            self.thrball(task["params"]["pr"], task["params"]["timeout"],task["params"]["k"],task["params"]["step_time"])
        
        elif task["name"] == "pass_door":
            self.pass_door(task["params"]["num"])

        elif task["name"] == "mttpos":
            self.mttpos(task["params"]["x"], task["params"]["y"], task["params"]["z"], task["params"]["rz"], task["params"]["dy"])
            
        elif task["name"] == "mttzpos_amend":
            self.mttzpos_amend(task["params"]["x"], task["params"]["y"], task["params"]["z"], task["params"]["dy"])
        
        elif task["name"] == "mttpos_amend":
            self.mttpos_amend(task["params"]["x"], task["params"]["y"], task["params"]["z"], task["params"]["rz"], task["params"]["dy"])
            
        elif task["name"] == "mttzpos_":
            self.mttzpos(task["params"]["x"], task["params"]["y"], task["params"]["z"], task["params"]["dy"])
        
        elif task["name"] == "delay":
            self.delay(task["params"])

        elif task["name"] == "led":
            self.led(task["params"]["led0"],
                     task["params"]["led1"])

        elif task["name"] == "grab_golf":
            self.grab_golf(task["params"]["kind"],
                           task["params"]["dx"],
                           task["params"]["dy"],
                           task["params"]["down_depth"],
                           task["params"]["up_depth"])
        
        elif task["name"] == "put_t":
            self.put_t(task["params"]["num"],
                           task["params"]["dy"],
                           task["params"]["dz"])
        
        elif task["name"] == "strike_ball":
            self.strike_ball(task["params"]["num"])
        
        elif task["name"] == "strike_ball3":
            self.strike_ball3(task["params"]["num1"],
                       task["params"]["num2"],
                       task["params"]["num3"],)
        
        elif task["name"] == "line_qd":
            # 无参数，直接调用line_qd方法
            self.line_qd()
            
        elif task["name"] == "endfloat":
            # 无参数，直接调用line_qd方法
            self.endfloat()

        else:
            self.get_logger().info("Info:非法任务名:  " + task["name"])
    
    def pid_update(self, error):
        """Computes PID output with integral and derivative terms."""
        self.dt = self.dt + 1 if abs(error-self.previous_error) < 2 else 1
        p_value = self.pid_parameters.p * error
        self.i_error += error * self.dt
        i_value = self.pid_parameters.i * self.i_error
        d_value = self.pid_parameters.d * (error - self.previous_error) / self.dt
        self.previous_error = error
        output = p_value + i_value + d_value
        if output > self.pid_parameters.output_limit:
            output = self.pid_parameters.output_limit
        elif output < -self.pid_parameters.output_limit:
            output = -self.pid_parameters.output_limit
        return output
    

    def cam2robot(self, clas, timeout,cam):
        """Averages detected target pose within a timeout window."""
        pos_list = []
        start_time = time.time()

        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # 检查超时
            if elapsed_time >= timeout:
                self.get_logger().info("超时退出")
                return 0.0, 0.0, 0.0
            if cam == "down":
                if self.yolov8_data_down.targets[clas].tpos_inworld.x != 0 or self.yolov8_data_down.targets[clas].tpos_inworld.y != 0 or self.yolov8_data_down.targets[clas].tpos_inworld.z != 0:
                    self.down_cam.target_inbase.vector.x = self.yolov8_data_down.targets[clas].tpos_inworld.x
                    self.down_cam.target_inbase.vector.y = self.yolov8_data_down.targets[clas].tpos_inworld.y
                    self.down_cam.target_inbase.vector.z = self.yolov8_data_down.targets[clas].tpos_inworld.z
                    #相机坐标系到机器人坐标系
                    self.down_cam.base2world()

                    t_x = self.robot.target_inbase.vector.x = self.down_cam.target_inworld.vector.x
                    t_y = self.robot.target_inbase.vector.y = self.down_cam.target_inworld.vector.y
                    t_z =  self.robot.target_inbase.vector.z = self.down_cam.target_inworld.vector.z
                    t_rx = self.robot.target_inbase.vector.rx = self.down_cam.target_inworld.vector.rx
                    t_ry = self.robot.target_inbase.vector.ry = self.down_cam.target_inworld.vector.ry
                    t_rz = self.robot.target_inbase.vector.rz = self.down_cam.target_inworld.vector.rz
                    self.robot.base2world()
                    time.sleep(0.015)
                                    # 检查返回的坐标是否为 0
                    if t_x != 0 or t_y != 0 or t_z != 0:
                        pos_list.append([t_x, t_y, t_z])
                        self.get_logger().info(f"t_x : {t_x:.2f} t_y: {t_y:.2f} t_z: {t_z:.2f}")
            if cam == "front":
                if self.yolov8_data_front.targets[clas].tpos_inworld.x != 0 or self.yolov8_data_front.targets[clas].tpos_inworld.y != 0 or self.yolov8_data_front.targets[clas].tpos_inworld.z != 0:
                    self.front_cam.target_inbase.vector.x = self.yolov8_data_front.targets[clas].tpos_inworld.x
                    self.front_cam.target_inbase.vector.y = self.yolov8_data_front.targets[clas].tpos_inworld.y
                    self.front_cam.target_inbase.vector.z = self.yolov8_data_front.targets[clas].tpos_inworld.z
                    #相机坐标系到机器人坐标系
                    self.front_cam.base2world()
                    t_x = self.robot.target_inbase.vector.x = self.front_cam.target_inworld.vector.x
                    t_y = self.robot.target_inbase.vector.y = self.front_cam.target_inworld.vector.y
                    t_z = self.robot.target_inbase.vector.z = self.front_cam.target_inworld.vector.z
                    t_rx = self.robot.target_inbase.vector.rx = self.front_cam.target_inworld.vector.rx
                    t_ry = self.robot.target_inbase.vector.ry = self.front_cam.target_inworld.vector.ry
                    t_rz = self.robot.target_inbase.vector.rz = self.front_cam.target_inworld.vector.rz
                        #机器人坐标系到世界坐标系
                    self.robot.base2world()
                    time.sleep(0.015)
                    if t_x != 0 or t_y != 0 or t_z != 0:
                        pos_list.append([t_x, t_y, t_z])
                        self.get_logger().info(f"t_x : {t_x:.2f} t_y: {t_y:.2f} t_z: {t_z:.2f}")

            if len(pos_list) >= 80:
                break
        x = y = z = 0.0
        for i in pos_list:
            x += i[0]
            y += i[1]
            z += i[2]
        x /= len(pos_list)
        y /= len(pos_list)
        z /= len(pos_list)
        self.get_logger().info(
            f"目标在机器人坐标系下的位置: x : {x:.2f} y: {y:.2f} z: {z:.2f}")

        return x, y, z
    

    def cam2robot_fast(self, clas, timeout,cam):
        """Fast target pose query for control loops."""
        pos_list = []
        start_time = time.time()

        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # 检查超时
            if elapsed_time >= timeout:
                self.get_logger().info("超时退出")
                return 0.0, 0.0, 0.0
            if cam == "down":
                if self.yolov8_data_down.targets[clas].tpos_inworld.x != 0 or self.yolov8_data_down.targets[clas].tpos_inworld.y != 0 or self.yolov8_data_down.targets[clas].tpos_inworld.z != 0:
                    self.down_cam.target_inbase.vector.x = self.yolov8_data_down.targets[clas].tpos_inworld.x
                    self.down_cam.target_inbase.vector.y = self.yolov8_data_down.targets[clas].tpos_inworld.y
                    self.down_cam.target_inbase.vector.z = self.yolov8_data_down.targets[clas].tpos_inworld.z
                    #相机坐标系到机器人坐标系
                    self.down_cam.base2world()

                    t_x = self.robot.target_inbase.vector.x = self.down_cam.target_inworld.vector.x
                    t_y = self.robot.target_inbase.vector.y = self.down_cam.target_inworld.vector.y
                    t_z =  self.robot.target_inbase.vector.z = self.down_cam.target_inworld.vector.z
                    t_rx = self.robot.target_inbase.vector.rx = self.down_cam.target_inworld.vector.rx
                    t_ry = self.robot.target_inbase.vector.ry = self.down_cam.target_inworld.vector.ry
                    t_rz = self.robot.target_inbase.vector.rz = self.down_cam.target_inworld.vector.rz
                    self.robot.base2world()
                                    # 检查返回的坐标是否为 0
                    if t_x != 0 or t_y != 0 or t_z != 0:
                        pos_list.append([t_x, t_y, t_z])
                        self.get_logger().info(f"t_x : {t_x:.2f} t_y: {t_y:.2f} t_z: {t_z:.2f}")
            if cam == "front":
                if self.yolov8_data_front.targets[clas].tpos_inworld.x != 0 or self.yolov8_data_front.targets[clas].tpos_inworld.y != 0 or self.yolov8_data_front.targets[clas].tpos_inworld.z != 0:
                    self.front_cam.target_inbase.vector.x = self.yolov8_data_front.targets[clas].tpos_inworld.x
                    self.front_cam.target_inbase.vector.y = self.yolov8_data_front.targets[clas].tpos_inworld.y
                    self.front_cam.target_inbase.vector.z = self.yolov8_data_front.targets[clas].tpos_inworld.z
                    #相机坐标系到机器人坐标系
                    self.front_cam.base2world()
                    t_x = self.robot.target_inbase.vector.x = self.front_cam.target_inworld.vector.x
                    t_y = self.robot.target_inbase.vector.y = self.front_cam.target_inworld.vector.y
                    t_z = self.robot.target_inbase.vector.z = self.front_cam.target_inworld.vector.z
                    t_rx = self.robot.target_inbase.vector.rx = self.front_cam.target_inworld.vector.rx
                    t_ry = self.robot.target_inbase.vector.ry = self.front_cam.target_inworld.vector.ry
                    t_rz = self.robot.target_inbase.vector.rz = self.front_cam.target_inworld.vector.rz
                        #机器人坐标系到世界坐标系
                    self.robot.base2world()
                    if t_x != 0 or t_y != 0 or t_z != 0:
                        pos_list.append([t_x, t_y, t_z])
                        self.get_logger().info(f"t_x : {t_x:.2f} t_y: {t_y:.2f} t_z: {t_z:.2f}")
            if len(pos_list) >= 50:
                break
        x = y = z = 0.0
        for i in pos_list:
            x += i[0]
            y += i[1]
            z += i[2]
        x /= len(pos_list)
        y /= len(pos_list)
        z /= len(pos_list)
        self.get_logger().info(
            f"目标在机器人坐标系下的位置: x : {x:.2f} y: {y:.2f} z: {z:.2f}")

        return x, y, z
    
    #抓球 pr为百分比距离
    def graball(self,color,depth,timeout,pr,k,step_time):
        """Composite grasping routine using vision and motion."""
        led = LedControllers()
        start_time = time.time()
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time >= timeout:
                self.get_logger().info("超时退出")
                break
            if all(x==0 for x in self.yolov8_data_down.state):
                self.led(0,0)
                self.get_logger().info(
                f"没有找到任何物体")
            elif self.yolov8_data_down.state[5] == 1 and self.yolov8_data_down.state[4] == 1: #5是黄色，4是pink
                self.led(1,0)
                self.get_logger().info(
                f"发现高尔夫球")
                if color == "pink":
                    a, b, c = self.cam2robot(4, 5,"down")
                    self.movex(a)
                    self.movey(b-0.175)
                    self.movez(depth)
                    self.movez(-depth)
                    break
                elif color == "yellow":
                    a, b, c = self.cam2robot(5, 5,"down")
                    self.movex(a)
                    self.movey(b-0.175)
                    self.movez(depth)
                    self.movez(-depth)
                    break
            elif self.yolov8_data_down.state[6] == 1:
                led.led0 = 0
                led.led1 = 1
                r = sqrt((self.yolov8_data_down.targets[6].tpos_inpic.y -  480)**2 + (self.yolov8_data_down.targets[6].tpos_inpic.x -  640)**2)/sqrt(480**2+640**2)
                a, b, c = self.cam2robot_fast(6, 5,"down")
                if r >= pr:
                    self.led_controllers_pub.publish(led)
                    self.get_logger().info(
                    f"只发现陈列框")
                    vx = k*a
                    vy = k*b
                    if abs(vx) < 0.01:
                        vx = 0.01 if vx >= 0 else -0.01
                    if abs(vy) < 0.01:
                        vy = 0.01 if vy >= 0 else -0.01
                    if abs(vx) > 0.04:
                        vx = 0.04 if vx >= 0 else -0.04
                    if abs(vy) > 0.04:
                        vy = 0.04 if vy >= 0 else -0.04
                    self.fast_movex(vx)
                    self.fast_movey(vy)
                    time.sleep(step_time)
                else:
                    self.get_logger().info(
                    f"陈列框已经进入视野中心")
            else:
                self.led(1,1)
                self.get_logger().info(
                f"检测到非必要物体")       
        self.led(0,0)
    #投球
    def thrball(self,pr,timeout,k,step_time):
        """Aligns to basket and performs throwing routine."""
        led = LedControllers()
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(self.down_cam_Image_data,"bgr8")
        row_index = img.shape[0] // 2
        column_index = img.shape[1] // 4
        self.get_logger().info(f"row: {row_index} , column: {column_index}")
        start_time = time.time()
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time >= timeout:
                self.get_logger().info("超时退出")
                break
            if all(x==0 for x in self.yolov8_data_down.state):
                led.led0 = 0
                led.led1 = 0
                self.led_controllers_pub.publish(led)
                self.get_logger().info(
                f"没有找到任何物体")
            elif self.yolov8_data_down.state[5] == 1:
                r = sqrt((self.yolov8_data_down.targets[5].tpos_inpic.y -  row_index)**2 + (self.yolov8_data_down.targets[5].tpos_inpic.x -  column_index)**2)/sqrt(480**2+640**2)
                led.led0 = 1
                led.led1 = 0
                a, b, c = self.cam2robot_fast(5, 5,"down")
                if r >= pr:
                    self.led_controllers_pub.publish(led)
                    self.get_logger().info(
                                            f"发现收集框")
                    vx = k*a
                    vy = k*b
                    if abs(vx) < 0.01:
                        vx = 0.01 if vx >= 0 else -0.01
                    if abs(vy) < 0.01:
                        vy = 0.01 if vy >= 0 else -0.01
                    if abs(vx) > 0.04:
                        vx = 0.04 if vx >= 0 else -0.04
                    if abs(vy) > 0.04:
                        vy = 0.04 if vy >= 0 else -0.04
                    self.fast_movex(vx)
                    self.fast_movey(vy)
                    time.sleep(step_time)
                else:
                    self.get_logger().info(f"收集框已进入视野中心")
                    break
            else:
                led.led0 = 1
                led.led1 = 1
                self.led_controllers_pub.publish(led)
                self.get_logger().info(
                f"检测到非必要物体")
        a, b, c = self.cam2robot(5, 5,"down")
        led.led0 = 1
        led.led1 = 0
        self.led_controllers_pub.publish(led)
        self.movex(a)
        self.movey(b-0.175)   
        self.pow(0) 
        time.sleep(1)      
        led.led0 = 0
        led.led1 = 0
        self.led_controllers_pub.publish(led)

    def pass_door(self,num):
        """Aligns to gate and passes through, then rotates."""
        self.setz(0.5)
        #self.setrz(0)
        start_time = time.time()
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time >= 30:
                self.get_logger().info("超时,采用planB")
                self.movey(1.0)
                break
            if self.yolov8_data_front.state[num] == 1:
                self.get_logger().info(f"检测到资格门")
                time.sleep(2)
                a, b, c = self.cam2robot_fast(num, 5,"front")
                b = b 
                rz = atan2(b, a)*RAD2DEG - 90
                self.get_logger().info(f"Info: ======转向目标点,旋转{rz:.2f}°======")
                self.moverz(rz)
                cnt = 0
                cnt2 = 0
                cnt3 = 0
                cnt4 = 0
                cmd = TargetPosDown()
                cmd.cs = 2
                cmd.pos.rx = 0.0
                cmd.pos.ry = 0.0
                cmd.pos.x = 0.0
                cmd.pos.y = 0.01
                cmd.pos.z = 0.0
                cmd.pos.rz = 0.0
                time.sleep(0.5)
                while True:
                    cnt+=1
                    if self.yolov8_data_front.state[num] == 0 or cnt > 300:
                        cnt4 += 1
                        time.sleep(0.1)
                        if self.yolov8_data_front.state[num] == 0 or cnt > 300:
                            cnt4 += 1#0.1秒后再进行一次检测
                    if cnt4 > 20 or cnt > 300:    
                        while True:
                            cnt3+=1
                            self.target_pos_down_pub.publish(cmd)
                            time.sleep(0.08)
                            if cnt3 > 60 :
                                break              
                        break
                    self.target_pos_down_pub.publish(cmd)
                    time.sleep(0.1)
                break

        cmd.cs = 2
        cmd.pos.rx = 0.0
        cmd.pos.ry = 0.0
        cmd.pos.x = 0.0
        cmd.pos.y = 0.0
        cmd.pos.z = 0.0
        time.sleep(1.0)
        if num == 5:#红，顺时针
            self.get_logger().info("开始旋转")
            cmd.pos.rz = 0.55
            while True:
                cnt2+=1
                self.target_pos_down_pub.publish(cmd)
                time.sleep(0.025)
                if cnt2 > 680:
                    break
            
        elif num == 11:#蓝
            self.get_logger().info("开始旋转")
            cmd.pos.rz = -0.55
            while True:
                cnt2+=1
                self.target_pos_down_pub.publish(cmd)
                time.sleep(0.025)
                if cnt2 > 680:
                    break
        #self.movey(0.5)
        time.sleep(1.5)
        self.movey(0.5)
        self.get_logger().info(f"过门任务完成")

    def led(self, led0, led1):
        """Publishes LED controller state."""
        led = LedControllers()
        led.led0 = led0
        led.led1 = led1
        self.led_controllers_pub.publish(led)
    
    def delay(self, t):
        """Sleeps for a given duration."""
        time.sleep(t)
    
    def grab_golf(self, kind, dx, dy, down_depth, up_depth):
        """Finds and grasps the specified golf ball."""
        
        num = 0
        
        if kind == "blue_golf":
            num = 4
        if kind == "red_golf":
            num = 3

        
        cmd = TargetPosDown()
        cmd.cs = 2
        cmd.pos.rx = 0.0
        cmd.pos.ry = 0.0
        cmd.pos.x = 0.0
        cmd.pos.y = 0.0
        cmd.pos.z = 0.0
        cmd.pos.rz = 0.1
        
        sample_flag = 0
        golf_cnt = 0
        
        self.pow(1)
        self.led(0, 0)
        s = 1
        while True:
            if all(x == 0 for x in self.yolov8_data_front.state) and \
               all(y == 0 for y in self.yolov8_data_down.state):
                # pass
                self.target_pos_down_pub.publish(cmd)
                time.sleep(0.01)
                golf_cnt+=1
                
            else:
                if self.yolov8_data_front.state[6]!= 0 :
                    self.get_logger().info("前视发现置物台")
                    time.sleep(2)
                    a, b, c = self.cam2robot(6, 5,"front")
                    a = a + 0.1
                    b = b - 0.1                    
                    self.movex(a)
                    self.movey(b)
                    self.setz(0.2)
                    time.sleep(2)
                    break
                elif self.yolov8_data_down.state[6]!= 0 :
                    self.get_logger().info("下视发现置物台")
                    time.sleep(2)
                    a, b, c = self.cam2robot(6, 5,"down")
                    a = a + 0.1
                    b = b - 0.1                      
                    self.movex(a)
                    self.movey(b)
                    self.setz(0.2)
                    time.sleep(2)
                    break
                else:
                    self.target_pos_down_pub.publish(cmd)
                    time.sleep(0.01)
                    golf_cnt+=1 
            if  golf_cnt>4000 :
                self.get_logger().info("任务失败")
                self.get_logger().info("======上浮======")
                self.setz(-0.5)
                return
                     
        '''s = self.search(kind, "down")
        if s == 0:
            self.get_logger().info("======下无目标======")
            self.get_logger().info("======第一次平移寻找目标目标======")
            self.movex(0.2)
            self.movey(0.2)
            s = self.search( kind, "down")
            if s == 0:
                self.get_logger().info("======下无目标======")
                self.get_logger().info("======第二次平移寻找目标目标======")
                self.movey(-0.4)
                s = self.search( kind, "down")
                if s == 0:
                    self.get_logger().info("======下无目标======")
                    self.get_logger().info("======第三次平移寻找目标目标======")
                    self.movex(-0.4)
                    s = self.search( kind, "down")
                    if s == 0:
                        self.get_logger().info("======下无目标======")
                        self.get_logger().info("======第四次平移寻找目标目标======")
                        self.movey(0.4)
                        s = self.search(kind, "down")
                        if s == 0:
                            self.get_logger().info("======下无目标======")
                            self.get_logger().info("======任务失败======")
                            return
        if s != 0 :
            self.get_logger().info("======发现球======")  
            self.get_logger().info("======正在移向球======")   
            self.mttxf(0,0) '''
        golf_cnt = 0
        while True: 
            if all(x == 0 for x in self.yolov8_data_front.state) and \
               all(y == 0 for y in self.yolov8_data_down.state):
                golf_cnt+=1
                time.sleep(0.01)
                if golf_cnt>2000:
                    self.get_logger().info("======确认无目标======")
                    self.get_logger().info("======上浮======")
                    self.setz(-0.5)
                    time.sleep(5.0)
                    self.get_logger().info("=====上浮结束======")
                    return                  
            elif self.yolov8_data_down.state[num] !=0:
                self.get_logger().info("======发现目标球=====")
                a, b, c = self.cam2robot(num, 5,"down")
                a = a - dx
                b = b - dy 
                self.movex(a)
                self.movey(b)

                self.delay(1)
                self.led(1, 0)
                self.movez(down_depth)
                self.get_logger().info("=====抓球结束，开始上浮======")
                self.setz(0.15)
                time.sleep(8.0)
                self.led(0, 0)
                if self.yolov8_data_down.state[6]!= 0 :
                    self.get_logger().info("下视发现置物台")
                    time.sleep(2)
                    a, b, c = self.cam2robot(6, 5,"down")
                    a = a + 0.1
                    b = b - 0.1                      
                    self.movex(a)
                    self.movey(b)
                    time.sleep(2)
                    break
                self.setz(-0.5)
                time.sleep(8.0)
                self.get_logger().info("=====上浮结束======")
                return
            else:
                golf_cnt+=1
                time.sleep(0.01)
                if golf_cnt>2000:
                    self.get_logger().info("======确认无目标======")
                    self.get_logger().info("======上浮======")
                    self.setz(-0.5)
                    time.sleep(5)
                    self.get_logger().info("=====上浮结束======")
                    return  
    
    
    def put_t(self, num,dy,dz):
        """Aligns and inserts the T plug."""
        T_cnt = 0
        T_cnt2 = 0
        T_cnt3 = 0
        cmd = TargetPosDown()
        cmd.cs = 2
        cmd.pos.rx = 0.0
        cmd.pos.ry = 0.0
        cmd.pos.x = 0.0
        cmd.pos.y = 0.0
        cmd.pos.z = 0.0
        cmd.pos.rz = 0.01

        while True:
            if self.yolov8_data_front.state[num] == 0: 
                self.target_pos_down_pub.publish(cmd)
                time.sleep(0.001)
                T_cnt+=1
                

            elif  self.yolov8_data_front.state[num] != 0:
                time.sleep(2)
                T_cnt = 0
                self.get_logger().info("发现目标")
                a, b, c = self.cam2robot(num, 5,"front")
                a=a-dy
                c=c-dz
                self.movex(a)
                self.movez(c)
                time.sleep(1)
                cmd.pos.rz = 0.0
                while True:
                    T_cnt2+=1
                    if self.yolov8_data_front.state[num] != 0 and T_cnt2 % 600 == 0: 
                        time.sleep(2)
                        a, b, c = self.cam2robot(num, 5,"front")
                        a=a-dy
                        c=c-dz
                        if (abs(a) > 0.03 or abs(c) > 0.03) and b > 0.25:
                            self.get_logger().info("======目标偏移======")
                            cmd.pos.x = float(a)
                            cmd.pos.z = float(c)
                            cmd.pos.y = 0.0
                            self.target_pos_down_pub.publish(cmd)
                            time.sleep(2)
                        else:
                            cmd.pos.y = 0.001
                            cmd.pos.x = 0.0
                            cmd.pos.z = 0.0
                            self.target_pos_down_pub.publish(cmd)
                            self.get_logger().info("======正在前进======")
                            T_cnt+=1
                            time.sleep(0.01)

                    else:
                        cmd.pos.y = 0.001
                        cmd.pos.x = 0.0
                        cmd.pos.z = 0.0
                        self.target_pos_down_pub.publish(cmd)
                        T_cnt+=1
                        time.sleep(0.01)
                    if T_cnt > 1000*(b+0.7):
                            self.pow(1)
                            self.get_logger().info("======任务完成======")
                            return
            if T_cnt>4000:
                self.get_logger().info("======未发现目标 任务失败======")            
                
    def endfloat(self):
        """Endgame float/target approach routine."""
   
        num = 5
        cmd = TargetPosDown()
        cmd.cs = 2
        cmd.pos.rx = 0.0
        cmd.pos.ry = 0.0
        cmd.pos.x = 0.0
        cmd.pos.y = 0.0
        cmd.pos.z = 0.0
        cmd.pos.rz = 0.2
        plate_cnt = 0
        while True:
            if self.yolov8_data_front.state[num] ==0 and \
               self.yolov8_data_down.state[num] ==0 :
                # pass
                self.target_pos_down_pub.publish(cmd)
                time.sleep(0.01)
                golf_cnt+=1
                
            else:
                if self.yolov8_data_front.state[num] ==1:
                    self.get_logger().info("前视发现目标")
                    time.sleep(2)
                    a, b, c = self.cam2robot(num, 5,"front")
                    a = a - 0.3
                    b = b - 0.5                    
                    self.movex(a)
                    self.movey(b)
                    time.sleep(1)
                    self.setz(-0.1)
                    self.get_logger().info("任务成功")
                    break
                else:
                    self.get_logger().info("下视发现目标")
                    a, b, c = self.cam2robot(num, 5,"down")
                    a = a - 0.3
                    b = b - 0.5                    
                    self.movex(a)
                    self.movey(b)
                    time.sleep(1)
                    self.setz(-0.1)
                    self.get_logger().info("任务成功")
                    break
            if  plate_cnt>2000 :
                self.get_logger().info("任务失败")
                return


    def throw_golf(self, dy, depth):
        """Throws ball after aligning to the basket."""
        self.get_logger().info("======开始投球======")
        self.led(0, 1)
        self.search('col_basket', 'down')
        self.led(1, 1)
        self.mttxf(0, dy)
        self.movez(depth)
        self.led(1, 0)
        self.pow(0)
        self.delay(1)
        self.led(0, 0)
        self.get_logger().info("======投球结束======")
        
    def strike_ball(self,num):
        """Hits a ball by aligning and pushing."""
        
        ball_cnt = 0
        cmd = TargetPosDown()
        cmd.cs = 2
        cmd.pos.rx = 0.0
        cmd.pos.ry = 0.0
        cmd.pos.x = 0.0
        cmd.pos.y = 0.0
        cmd.pos.z = 0.0
        cmd.pos.rz = 0.05

        while True:
            if self.yolov8_data_front.state[num] == 0: 
                self.target_pos_down_pub.publish(cmd)
                time.sleep(0.01)
                ball_cnt+=1
            elif  self.yolov8_data_front.state[num] != 0:
                time.sleep(2)
                ball_cnt = 0
                self.get_logger().info("发现目标")
                a, b, c = self.cam2robot(num, 5,"front")
                #a = a 
                c = c - 0.32
                b = b - 0.2
                self.fast_movez(c)
                time.sleep(1)
                self.movexy(a,b)
                time.sleep(1)
                self.get_logger().info("======任务完成 正在返回======")
                self.fast_movey(-b)
                self.move_wait()
                time.sleep(4)
                self.get_logger().info("======返回完成======")
                               
                return
            if ball_cnt>75000:
                self.get_logger().info("======未发现目标 任务失败======")
                return
                
    def strike_ball3(self,num1,num2,num3):
        """Sequentially hits three balls."""
        self.get_logger().info("======撞球任务开始======")
        #self.setp()
        self.get_logger().info("======撞第一个球======")
        self.strike_ball(num1)
        #self.fast_moverz(180.0)
        #self.get_logger().info("======正在返回======")
        #self.fast_back()
        #time.sleep(4)
        self.get_logger().info("======撞第二个球======")
        self.strike_ball(num2)
        #self.fast_moverz(180.0)
        #self.get_logger().info("======正在返回======")
        #self.fast_back()
        #time.sleep(4)
        self.get_logger().info("======撞第三个球======")
        self.strike_ball(num3)
        #self.get_logger().info("======正在返回======")
        #self.fast_back()
        
    def line_qd(self):
        """Line-following routine with PID and image processing."""
        
            
        self.led(0,0)
        
        self.pid_parameters.p = 0.0009
        self.pid_parameters.i = 0.0
        self.pid_parameters.d = 0.08
        self.pid_parameters.output_limit = 5.0

        cmd = TargetPosDown()
        cmd.cs = 2
        cmd.pos.rx = 0.0
        cmd.pos.ry = 0.0

        drz = 0.0

        timesleep = 0.025
        
        self.task_lock = [0,0,0]
        
        magnet = MagnetController()
        led = LedControllers()
        bridge = CvBridge()
    
        window_size = 3  # 滑动窗口的大小
        last_positions = []  # 使用列表来存储最近的位置
        cnt = 0
        cnt_1 = 0
        #等待准备就绪
        time.sleep(1.0)
        while True:
            #try:
            # 转化为opencv图像
            img1 = bridge.imgmsg_to_cv2(self.down_cam_Image_data, "mono8")
            img = bridge.imgmsg_to_cv2(self.segment_img, "mono8")
            if img is None:
                self.get_logger().error("bridge.imgmsg_to_cv2 returned None")
                return
            img = cv2.flip(img, -1)

            row_index = img.shape[0] // 3
            row_index1 = img.shape[0] // 2
            column_index = img.shape[1] // 2
            row = img[row_index, :]
            white_pixels = np.where(row == 255)[0]

            # 计算白点的平均位置作为线的中心
            if white_pixels.size > 0:
                line_center = np.mean(white_pixels).astype(int)
                last_positions.append(line_center)
                if len(last_positions) > window_size:  # 如果列表长度超过窗口大小，删除最旧的元素
                    last_positions.pop(0)
            else:
                if last_positions:
                    line_center = last_positions[-1]  # 如果没有检测到新的，则使用上一个

            # 计算滑动平均
            if len(last_positions) > 1:
                smoothed_center = int(np.mean(last_positions))
                cv2.circle(img, (smoothed_center, row_index), 10, (0, 0, 0), -1)  # 标记平滑后的中心点
            else:
                smoothed_center = img.shape[1] // 2
                cv2.circle(img, (smoothed_center, row_index), 10, (0, 0, 0), -1)

            # 在图像上添加文字
            cv2.putText(img, "output: {:.2f}".format(drz), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)

            img_msg = bridge.cv2_to_imgmsg(img, "mono8")
            self.line_patrol_img_pub.publish(img_msg)

            angle_error = smoothed_center - column_index
            drz = self.pid_update(angle_error)

            if all(x==0 for x in self.yolov8_data_down.state):
                # pass
                cmd.pos.x = 0.0
                cmd.pos.y = 0.002
                cmd.pos.z = 0.0
                cmd.pos.rz = drz
                self.target_pos_down_pub.publish(cmd)
                time.sleep(timesleep)
                cnt_1 += 1
                
            elif not all(x==0 for x in self.yolov8_data_down.state):
                if  self.yolov8_data_down.state[0] != 0 and  self.yolov8_data_down.targets[0].tpos_inpic.y > row_index1 :
                    if self.task_lock[0] == 0 :
                        self.get_logger().info("Info:检测A，开始执行任务") 
                        #self.setp()
                        #a, b, c = self.cam2robot(1, 5,"down")
                        #self.movex(a)
                        #self.movey(b)
                    
                        # 保存任务结束时的图片
                        task1_end_img_path = f"A.png"
                        cv2.imwrite(task1_end_img_path, img1)
                        self.get_logger().info(f"Info:保存图片 {task1_end_img_path}")

                        self.get_logger().info("Info:任务执行完毕")
                        #self.back()
                        self.task_lock[0] = 1
                    else:
                        #pass
                        cmd.pos.x = 0.0
                        cmd.pos.y = 0.002
                        cmd.pos.z = 0.0
                        cmd.pos.rz = drz
                        self.target_pos_down_pub.publish(cmd)
                        cnt_1 += 1
                        time.sleep(timesleep)
                elif self.yolov8_data_down.state[1] != 0 and self.yolov8_data_down.targets[1].tpos_inpic.y > row_index1:
                    if self.task_lock[1] == 0 :
                        self.get_logger().info("Info:检测B，开始执行任务")
                        #self.setp()
                        #a, b, c = self.cam2robot(2, 4,"down")
                        #self.movex(a)
                        #self.movey(b)
                        
                        # 保存任务结束时的图片
                        task2_end_img_path = f"B.png"
                        cv2.imwrite(task2_end_img_path, img1)
                        self.get_logger().info(f"Info:保存图片 {task2_end_img_path}")

                        #self.back()
                        self.get_logger().info("Info:任务执行完毕")
                        self.task_lock[1] = 1
                    else:
                        #pass
                        cmd.pos.x = 0.0
                        cmd.pos.y = 0.002
                        cmd.pos.z = 0.0
                        cmd.pos.rz = drz
                        self.target_pos_down_pub.publish(cmd)
                        cnt_1 += 1
                        time.sleep(timesleep)
                elif self.yolov8_data_down.state[2] != 0 and self.yolov8_data_down.targets[2].tpos_inpic.y > row_index1:
                    if self.task_lock[2] == 0 :
                        self.get_logger().info("Info:检测C，开始执行任务") 
                        #self.setp()
                        #a, b, c = self.cam2robot(0, 4,"down")
                        #self.movex(a)
                       # self.movey(b)
                        # 保存任务结束时的图片
                        task3_end_img_path = f"C.png"
                        cv2.imwrite(task3_end_img_path, img1)
                        self.get_logger().info(f"Info:保存图片 {task3_end_img_path}")

                        self.get_logger().info("Info:任务执行完毕")
                        self.task_lock[2] = 1
                    else:
                        #pass
                        cmd.pos.x = 0.0
                        cmd.pos.y = 0.002
                        cmd.pos.z = 0.0
                        cmd.pos.rz = drz
                        self.target_pos_down_pub.publish(cmd)
                        cnt_1 += 1
                        time.sleep(timesleep)

                else:
                    #pass
                    cmd.pos.x = 0.0
                    cmd.pos.y = 0.002
                    cmd.pos.z = 0.0
                    cmd.pos.rz = drz
                    self.target_pos_down_pub.publish(cmd)
                    cnt_1 += 1
                    time.sleep(timesleep)
            if cnt_1 > 1200:
                self.get_logger().info("Info:任务全部执行完毕")
                return
            if all(x == 1 for x in self.task_lock):
                cmd.pos.x = 0.0
                cmd.pos.y = 0.004
                cmd.pos.z = 0.0
                cmd.pos.rz = drz
                self.target_pos_down_pub.publish(cmd)
                time.sleep(timesleep)
                cnt+=1
                if cnt > 200:
                    self.get_logger().info("Info:任务全部执行完毕")
                    return

          
    #巡线
    def line(self, ys_dep):
        """Line-following routine with continuous corrections."""
        self.led(0, 0)
        
        self.pid_parameters.p = 0.0009
        self.pid_parameters.i = 0.0
        self.pid_parameters.d = 0.08
        self.pid_parameters.output_limit = 5.0

        cmd = TargetPosDown()
        cmd.cs = 2
        cmd.pos.rx = 0.0
        cmd.pos.ry = 0.0

        drz = 0.0
        depth = ys_dep
        timesleep = 0.025
        
        self.task_lock = [0, 0, 0]
        
        magnet = MagnetController()
        bridge = CvBridge()
        
        # 初始化1D卡尔曼滤波器（仅跟踪位置）
        kf = KalmanFilter(dim_x=1, dim_z=1)
        kf.x = np.array([320.])  # 初始位置（图像中心附近）
        kf.F = np.array([[1.]])  # 状态转移矩阵（简单恒速模型）
        kf.H = np.array([[1.]])  # 观测矩阵
        kf.P *= 100.  # 初始协方差
        kf.R = 3.0    # 观测噪声
        kf.Q = np.array([[1.0]])  # 过程噪声
        
        # 图像状态跟踪变量
        last_valid_image_time = None
        has_valid_image = False
        last_filtered_center = 320  # 初始中心位置
        img_shape = (480, 640)  # 预设图像尺寸，根据实际情况调整
        row_index = img_shape[0] // 3  # 预设行索引
        row_index1 = img_shape[0] // 2  # 预设任务检测行索引
        column_index = img_shape[1] // 2  # 预设列中心
        
        # 新增：图像初始化等待机制
        init_wait_time = 0.1  # 初始化等待0.1秒
        start_time = time.time()
        self.get_logger().info(f"等待图像初始化，最多等待{init_wait_time}秒...")
        
        # 等待图像或超时
        while not has_valid_image and (time.time() - start_time) < init_wait_time:
            try:
                if self.segment_img is not None:
                    img = bridge.imgmsg_to_cv2(self.segment_img, "mono8")
                    img_shape = img.shape
                    row_index = img_shape[0] // 3
                    row_index1 = img_shape[0] // 2
                    column_index = img_shape[1] // 2
                    has_valid_image = True
                    last_valid_image_time = time.time()
                    self.get_logger().info("成功获取初始图像")
            except Exception as e:
                self.get_logger().debug(f"初始化阶段图像获取失败: {str(e)}")
            time.sleep(0.1)  # 短时间等待后重试
        
        if not has_valid_image:
            self.get_logger().warn(f"初始化超时，未获取到图像，将使用默认参数运行")
            
        cnt = 0
        cnt_1 = 0
        while True:
            cnt_1 = cnt_1 + 1
            if cnt_1 > 2000:
                self.get_logger().warn(f"时间结束自动退出")
                return
                
            # 尝试获取和处理图像
            try:
                if self.segment_img is not None:
                    img = bridge.imgmsg_to_cv2(self.segment_img, "mono8")
                    img = cv2.flip(img, -1)
                    img_shape = img.shape
                    row_index = img_shape[0] // 3
                    row_index1 = img_shape[0] // 2
                    column_index = img_shape[1] // 2
                    last_valid_image_time = time.time()
                    has_valid_image = True
                    
                    # 处理图像获取线中心
                    row = img[row_index, :]
                    white_pixels = np.where(row == 255)[0]

                    # 卡尔曼滤波处理
                    kf.predict()
                    
                    # 有观测值时更新
                    if white_pixels.size > 0:
                        line_center = np.mean(white_pixels).astype(int)
                        kf.update(line_center)
                    
                    # 使用滤波后的值
                    filtered_center = int(kf.x[0])
                    last_filtered_center = filtered_center
                    
                    # 绘制标记并发布图像
                    cv2.circle(img, (filtered_center, row_index), 10, (0, 0, 0), -1)
                    cv2.putText(img, "output: {:.2f}".format(drz), (50, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
                    img_msg = bridge.cv2_to_imgmsg(img, "mono8")
                    self.line_patrol_img_pub.publish(img_msg)
                    

                        
            except Exception as e:
                # 图像处理失败时的处理
                self.get_logger().warn(f"无分割图像: {str(e)}")
                
                # 仅使用预测
                kf.predict()
                filtered_center = int(kf.x[0])
                last_filtered_center = filtered_center
                
                # 状态判断与警示
                if not has_valid_image:
                    self.get_logger().warn("持续未接收到图像，使用默认控制策略")
                elif (time.time() - last_valid_image_time) > 1.0:
                    self.get_logger().warn("图像已丢失超过1秒，使用预测值控制")
    
            
            # 计算控制量
            angle_error = filtered_center - column_index
            # 无图像时减小控制增益，采用更保守的控制
            if not has_valid_image:
                drz = self.pid_update(angle_error) * 0.5  # 降低控制强度
                dy = 0.004  # 降低前进速度
            else:
                drz = self.pid_update(angle_error)
                dy = 0.005 - 0.0004 * abs(drz)
            
            # 控制逻辑
            if all(x == 0 for x in self.yolov8_data_down.state):
                cmd.pos.x = 0.0
                cmd.pos.y = dy
                cmd.pos.z = 0.0
                cmd.pos.rz = drz
                self.target_pos_down_pub.publish(cmd)
                time.sleep(timesleep)
                
            elif not all(x == 0 for x in self.yolov8_data_down.state):
                if self.yolov8_data_down.state[1] != 0 and self.yolov8_data_down.targets[1].tpos_inpic.y > row_index1:
                        if self.task_lock[0] == 0:
                            self.get_logger().info("检测到黑色方块，开始执行任务")
                            time.sleep(2)
                            magnet.state = 0
                            self.magnet_controller_pub.publish(magnet)
                            self.led(1, 1)
                            time.sleep(1)
                            self.get_logger().info("任务执行完毕")
                            self.task_lock[0] = 1
                        else:
                            cmd.pos.x = 0.0
                            cmd.pos.y = dy
                            cmd.pos.z = 0.0
                            cmd.pos.rz = drz
                            self.target_pos_down_pub.publish(cmd)
                            time.sleep(timesleep)
                elif self.yolov8_data_down.state[2] != 0 and self.yolov8_data_down.targets[2].tpos_inpic.y > row_index1:
                    if self.task_lock[1] == 0:
                        self.get_logger().info("检测到绿色圆形，开始执行任务")
                        time.sleep(2)
                        a, b, c = self.cam2robot(2, 4, "down")
                        b = b - 0.4
                        self.movex(a)
                        self.movey(b)
                        self.movez(depth)
                        self.led(1,0)
                        self.movez(-depth)
                        self.led(0,0)
                        self.get_logger().info("任务执行完毕")
                        self.task_lock[1] = 1
                    else:
                        cmd.pos.x = 0.0
                        cmd.pos.y = dy
                        cmd.pos.z = 0.0
                        cmd.pos.rz = drz
                        self.target_pos_down_pub.publish(cmd)
                        time.sleep(timesleep)
                elif self.yolov8_data_down.state[0] != 0 and self.yolov8_data_down.targets[0].tpos_inpic.y > row_index1:
                    if self.task_lock[2] == 0:
                        self.get_logger().info("检测到黄色三角，开始执行任务")
                        time.sleep(2)
                        self.moverz(179.9)
                        time.sleep(1)
                        self.moverz(179.9)
                        time.sleep(1)
                        self.get_logger().info("任务执行完毕")
                        self.task_lock[2] = 1
                    else:
                        cmd.pos.x = 0.0
                        cmd.pos.y = dy
                        cmd.pos.z = 0.0
                        cmd.pos.rz = drz
                        self.target_pos_down_pub.publish(cmd)
                        time.sleep(timesleep)
                else:
                    cmd.pos.x = 0.0
                    cmd.pos.y = dy
                    cmd.pos.z = 0.0
                    cmd.pos.rz = drz
                    self.target_pos_down_pub.publish(cmd)
                    time.sleep(timesleep)
                    
            if all(x == 1 for x in self.task_lock):
                cmd.pos.x = 0.0
                cmd.pos.y = dy
                cmd.pos.z = 0.0
                cmd.pos.rz = drz
                self.target_pos_down_pub.publish(cmd)
                time.sleep(timesleep)
                cnt += 1
                if cnt > 190:
                    return
    

            # except Exception as e:
            #     self.get_logger().error(f"图像处理过程中出现错误 {e}")
            #     break
                # except Exception as e:
                #     self.get_logger().error(f"图像处理过程中出现错误 {e}")
                #     break
 


# 从命令行获取任务
