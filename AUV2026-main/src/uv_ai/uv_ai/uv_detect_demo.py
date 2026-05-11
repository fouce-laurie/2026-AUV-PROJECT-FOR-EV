import rclpy
from rclpy.node import Node
import subprocess
from multiprocessing import Process,Pipe
import threading

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
import cv2
from pathlib import Path
import numpy as np
import torch
import torch.backends.cudnn as cudnn
from numpy import random
import argparse
import torch
import torch.nn as nn
from ultralytics import YOLO

from sensor_msgs.msg import Image
from uv_msgs.srv import DetectRequest
from uv_msgs.msg import Yolov8, TargetParams
from typing import Dict, Tuple, Union, List

from uv_vision.stereocam import StereoCamera

# 养了一只猫猫，路过的人可以摸一摸
# 　／l、
# （ﾟ､ 。 ７
# 　l、 ~ヽ
# 　じしf_, )ノ


class AiNode(Node):
    def __init__(self, name, opt, pipe_1, pipe_2):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        self.opt = opt
        self.bridge = CvBridge()  # 图像格式转换器

        # 是否正在处理图像
        self.processing = False
        self.front_cam_Image_data = None
        self.down_cam_Image_data = None
        self.pipe_front   =  pipe_1
        self.pipe_down    =  pipe_2
        #话题发布
        #创建话题发布detectedimg，定义其消息类型为Image，用于发布检测结果图像
        self.detectedimg_down_pub = self.create_publisher(
            Image, "detectedimg_down", 10)
                #创建话题发布detectedimg，定义其消息类型为Image，用于发布检测结果图像
        self.detectedimg_front_pub = self.create_publisher(
            Image, "detectedimg_front", 10)
        
        #创建话题发布detectedimg，定义其消息类型为Image，用于发布检测结果图像
        self.detectserveimg_pub = self.create_publisher(
            Image, "detect_server_img", 10)
        
        # 创建话题发布 uv_detect，定义其消息类型为Yolov8，用于发布检测结果
        self.detect_result_pub_down = self.create_publisher(
            Yolov8, 'uv_detect_down', 10)
        
        self.detect_result_pub_front = self.create_publisher(
            Yolov8, 'uv_detect_front', 10) 

        #话题接收
        #创建话题接收down_cam/rectified,定义其消息类型为Image
        self.create_subscription(
            Image, 'down_cam/rectified', self.down_cam_callback, 10)
        
        self.create_subscription(
            Image, 'front_cam/rectified', self.front_cam_callback, 10   
        )
        #初始化服务端
        #服务类型为DetectRequest，服务 uv_detect_srv , 回调函数为self.detectrequest_callback
        self.detect_srv = self.create_service(
            DetectRequest, 'uv_detect_srv', self.detectrequest_callback)

        # # 初始化相机参数
        fsc = StereoCamera()
        dsc = StereoCamera()
        self.stereocams = {"front": fsc, "down": dsc}
        self.stereocams["front"].cal_parameters_init(opt.front_params)
        self.stereocams["down"].cal_parameters_init(opt.down_params)

        # 初始化
        #set_logging()
        self.device = torch.device("cuda:0") if torch.cuda.is_available() else torch.device("cpu")
        # 加载模型
        self.model = YOLO(opt.weights2)
        self.model.to(self.device)

        self.names = self.model.names if hasattr(self.model, 'names') else self.model.module.names


    # 下置摄像头回调函数
    def down_cam_callback(self, data):
        self.down_cam_Image_data = data
        self.process_image_down()

    def front_cam_callback(self, data):
        self.front_cam_Image_data = data
        self.process_image_front()
    
    def process_image_down(self):
        if self.down_cam_Image_data is not None and not self.processing:
            self.processing = True
            img0 = self.bridge.imgmsg_to_cv2(self.down_cam_Image_data, 'bgr8')
            self.pipe_down.send((img0,"down"))
            self.get_logger().info("下视图像已发送到检测进程")
            results_down, img_msg_down = self.pipe_down.recv()
            self.get_logger().info(f"接收到下视检测结果！")
            self.detectedimg_down_pub.publish(img_msg_down)
            self.detect_result_pub_down.publish(results_down)
            self.processing = False

    def process_image_front(self):
        if self.front_cam_Image_data is not None and not self.processing: 
            self.processing = True      
            img1 = self.bridge.imgmsg_to_cv2(self.front_cam_Image_data, 'bgr8')
            self.pipe_front.send((img1,"front"))
            self.get_logger().info("前视图像已发送到检测进程")
            results_front, img_msg_front = self.pipe_front.recv()
            self.get_logger().info(f"接收到前视检测结果！")
            self.detectedimg_front_pub.publish(img_msg_front)
            self.detect_result_pub_front.publish(results_front)
            self.processing = False
        
            
    #检测结果预处理
    def img_process(self, results):
        try:
            clas = results.boxes.cls.tolist()
            conf = results.boxes.conf.tolist()
            point = results.boxes.xyxy.tolist()
            combined = list(zip(clas, conf, point))
            # 过滤掉置信度低于0.1的检测结果，并将结果格式化为 (clas, conf, center)
            filtered = [(item[0], item[1], ((item[2][0] + item[2][2]) / 2, (item[2][1] + item[2][3]) / 2)) for item in combined if item[1] >= 0.1]

            return filtered
        except Exception as e:
            self.get_logger().error(f"图像处理错误: {e}")
            return []

    # 服务回调函数
    def detectrequest_callback(self, request, response):
        img0 = self.bridge.imgmsg_to_cv2(request.imagein, 'bgr8')
        askedtarget = request.target
        response.s = 0
        _, width, _ = img0.shape
        half_width = width // 2

        #把图像分为左右两部分
        img_l = img0[:, :half_width]
        img_r = img0[:, half_width:]

        t0 = time.time()
        #模型预测
        results_l = self.model.predict(source=img_l)[0]
        filtered_l = self.img_process(results_l)
        self.get_logger().info(f"server:左目完成图像预测,检测到 {len(filtered_l)} 个目标")

        results_r = self.model.predict(source=img_r)[0]
        filtered_r = self.img_process(results_r)
        self.get_logger().info(f"server:右目完成图像预测,检测到 {len(filtered_r)} 个目标")
        #合并检测结果
        targets_dict: Dict[Union[str, int], Tuple[float, Tuple[float], Tuple[float]]] = {}
        for clas, conf, center_l in filtered_l:
                targets_dict[clas] = (conf, center_l, None)
        for clas, conf, center_r in filtered_r:
            if clas in targets_dict:
                targets_dict[clas] = (targets_dict[clas][0] , targets_dict[clas][1], center_r)

        #遍历字典，计算每个目标物的三维坐标
        for clas in targets_dict:
            left_pt = None
            right_pt = None
            if self.names[int(clas)] == askedtarget:
                    left_pt = targets_dict[clas][1]
                    right_pt = targets_dict[clas][2]

            if left_pt is not None and right_pt is not None:
                # 使用左右两点纵坐标的平均值作为新的纵坐标
                avg_y = (left_pt[1] + right_pt[1]) / 2
                left_pt = np.array([left_pt[0], avg_y], dtype=np.float32).reshape(2, 1)
                right_pt = np.array([right_pt[0], avg_y], dtype=np.float32).reshape(2, 1)      
                
                pt = cv2.triangulatePoints(self.stereocams[request.stero].P1, self.stereocams[request.stero].P2, left_pt, right_pt)
                pt = pt / pt[3][0]  # 标准化为笛卡尔坐标
                x, y, z = -float(pt[0][0]), -float(pt[1][0]), float(pt[2][0])  # 转换为浮点数确保正确的数据类型

                response.s = 1
                response.x, response.y, response.z = x, y, z

        img_l = results_l.plot()
        img_r = results_r.plot()
        img0 = np.hstack((img_l, img_r))
        img0 = np.array(img0)
        data = self.bridge.cv2_to_imgmsg(img0, encoding="bgr8")
        self.detectserveimg_pub.publish(data)

        if response.s == 1:
            self.get_logger().info("已检测到目标，用时: %.3fs" % (time.time() - t0))
        else:
            self.get_logger().info("未检测到目标，用时: %.3fs" % (time.time() - t0))

        return response

        

def detect_process(pipe, opt):

    # 初始化相机参数
    fsc = StereoCamera()
    dsc = StereoCamera()
    stereocams = {"front": fsc, "down": dsc}
    stereocams["front"].cal_parameters_init(opt.front_params)
    stereocams["down"].cal_parameters_init(opt.down_params)
    
    model1 = YOLO(opt.weights)
    device1 = torch.device("cuda:0") if torch.cuda.is_available() else torch.device("cpu")
    model1.to(device1)

    while True:
        img0 ,cam_type = pipe.recv()
        tar = Yolov8()

        try:
            print("detect_process:开始处理图像")
            _, width, _ = img0.shape
            half_width = width // 2
            img_l = img0[:, :half_width]
            img_r = img0[:, half_width:]
            # 模型预测
            #results = model1.predict(source=img0)[0]

            print("detect_process:完成图像预测...")
            results_l = model1.predict(source=img_l)[0]
            clas_l = results_l.boxes.cls.tolist()
            conf_l = results_l.boxes.conf.tolist()
            point_l = results_l.boxes.xyxy.tolist()
            combined_l = list(zip(clas_l, conf_l, point_l))
            # 过滤掉置信度低于0.5的检测结果，并将结果格式化为 (clas, conf, center)
            filtered_l = [(item[0], item[1], ((item[2][0] + item[2][2]) / 2, (item[2][1] + item[2][3]) / 2)) for item in combined_l if item[1] >= 0.10]
            print(f"process:左目完成图像预测,检测到 {len(filtered_l)} 个目标")
            
            # 绘制左图过滤后的框
            for item in combined_l:
                if item[1] >= 0.85:
                    x1, y1, x2, y2 = item[2]
                    cv2.rectangle(img_l, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    label = f"{model1.names[int(item[0])]}: {item[1]:.2f}"
                    cv2.putText(img_l, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            results_r = model1.predict(source=img_r)[0]
            clas_r = results_r.boxes.cls.tolist()
            conf_r = results_r.boxes.conf.tolist()
            point_r = results_r.boxes.xyxy.tolist()
            combined_r = list(zip(clas_r, conf_r, point_r))
            # 过滤掉置信度低于0.5的检测结果，并将结果格式化为 (clas, conf, center)
            for item in combined_l:
                if item[1] >= 0.85:
                    x1, y1, x2, y2 = item[2]
                    cv2.rectangle(img_l, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    label = f"{model1.names[int(item[0])]}: {item[1]:.2f}"
                    cv2.putText(img_l, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            results_r = model1.predict(source=img_r)[0]
            clas_r = results_r.boxes.cls.tolist()
            conf_r = results_r.boxes.conf.tolist()
            point_r = results_r.boxes.xyxy.tolist()
            combined_r = list(zip(clas_r, conf_r, point_r))
            # 过滤掉置信度低于0.5的检测结果，并将结果格式化为 (clas, conf, center)
            filtered_r = [(item[0], item[1], ((item[2][0] + item[2][2]) / 2, (item[2][1] + item[2][3]) / 2)) for item in combined_r if item[1] >= 0.10]
            print(f"process:右目完成图像预测,检测到 {len(filtered_r)} 个目标")
            
            # 绘制右图过滤后的框
            for item in combined_r:
                if item[1] >= 0.85:
                    x1, y1, x2, y2 = item[2]
                    cv2.rectangle(img_r, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    label = f"{model1.names[int(item[0])]}: {item[1]:.2f}"
                    cv2.putText(img_r, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            targets_dict: Dict[Union[str, int], Tuple[float, Tuple[float, float], Tuple[float, float]]] = {}
            
            # 对每个类别分别进行左右图像的目标匹配
            classes_in_l = set(item[0] for item in filtered_l)
            classes_in_r = set(item[0] for item in filtered_r)
            common_classes = classes_in_l.intersection(classes_in_r)
            
            for cls_id in common_classes:
                # 提取当前类的所有目标
                targets_l = [item for item in filtered_l if item[0] == cls_id]
                targets_r = [item for item in filtered_r if item[0] == cls_id]
                
                best_pair = None
                best_score = float('inf')  # 评价标准：y的差异越小越好，视差 > 0 (x_l > x_r)
                
                for l_tar in targets_l:
                    for r_tar in targets_r:
                        y_diff = abs(l_tar[2][1] - r_tar[2][1])
                        disp = l_tar[2][0] - r_tar[2][0]
                        # 极线约束：y差异应较小，视差应为正（通常在正常范围内）
                        if y_diff < 50 and disp > 0:
                            # 我们可以选择 y_diff 最小的，或者综合置信度最高的
                            # 这里以 y_diff 最优为主，附加置信度考量
                            score = y_diff - (l_tar[1] + r_tar[1]) * 10
                            if score < best_score:
                                best_score = score
                                best_pair = (l_tar, r_tar)
                
                if best_pair is not None:
                    # 记录该类最优匹配的一对目标（置信度取两者平均或左侧即可）
                    targets_dict[cls_id] = (best_pair[0][1], best_pair[0][2], best_pair[1][2])
            
            # 动态根据模型类别数初始化 state 和 targets 列表
            num_classes = len(model1.names) if hasattr(model1, 'names') else 0
            
            # 使用不定长数组，初始化 state 和 targets
            tar.state = [0.0] * num_classes
            tar.targets = [TargetParams() for _ in range(num_classes)]

            for clas in targets_dict:
                idx = int(clas)
                # 越界保护：如果类别索引超出当前初始化数组长度则跳过
                if idx >= num_classes or idx < 0:
                    print(f"detect_process: 类别索引 {idx} 超出模型类别数 {num_classes}，跳过。")
                    continue

                if targets_dict[clas][1] is not None or targets_dict[clas][2] is not None:
                    tar.state[idx] += 1

                left_pt = targets_dict[clas][1]
                right_pt = targets_dict[clas][2]

                if left_pt is not None and right_pt is not None:
                    # left_pt = np.array(left_pt, dtype=np.float32).reshape(2, 1)
                    # right_pt = np.array(right_pt, dtype=np.float32).reshape(2, 1)

                    # 使用左右两点纵坐标的平均值作为新的纵坐标
                    avg_y = (left_pt[1] + right_pt[1]) / 2
                    left_pt = np.array([left_pt[0], avg_y], dtype=np.float32).reshape(2, 1)
                    right_pt = np.array([right_pt[0], avg_y], dtype=np.float32).reshape(2, 1)      

                    pt = cv2.triangulatePoints(stereocams[cam_type].P1, stereocams[cam_type].P2, left_pt, right_pt)
                    pt = pt / pt[3][0]  # 标准化为笛卡尔坐标
                    x, y, z = -float(pt[0][0]), -float(pt[1][0]), float(pt[2][0])  # 转换为浮点数确保正确的数据类型

                    # 再次保证索引安全（已在上面检查，但双重保险）
                    if idx < len(tar.targets):
                        tar.targets[idx].tpos_inpic.x = round(targets_dict[clas][1][0])
                        tar.targets[idx].tpos_inpic.y = round(targets_dict[clas][1][1])
                        tar.targets[idx].tpos_inworld.x = x
                        tar.targets[idx].tpos_inworld.y = y
                        tar.targets[idx].tpos_inworld.z = z
                    else:
                        print(f"detect_process: 目标索引 {idx} 无对应 tar.targets 元素，跳过写入。")
            
            img0 = np.hstack((img_l, img_r)) #图像二合一
            img0 = np.array(img0)    
            img_msg = CvBridge().cv2_to_imgmsg(img0, encoding="bgr8")

            pipe.send((tar, img_msg))

            print("detect_process:图像处理完成")
        except CvBridgeError as e:
            print(f"detect_process:图像转换错误: {e}")
        except Exception as e:
            print(f"detect_process:目标检测错误: {e}")



import rclpy
from rclpy.node import Node
import argparse
from multiprocessing import Pipe, Process
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def main(args=None):
    try:
        package_share_directory = get_package_share_directory('uv_ai')
        datas_dir = Path(package_share_directory) / 'datas'
    except Exception:
        datas_dir = Path('.') / 'datas'
        print(f"Warning: Could not find package share directory, using fallback: {datas_dir}")

    # 过滤ROS 2自动添加的参数（关键步骤）
    # 从命令行参数中移除ROS相关参数（如--ros-args），只保留自定义参数
    ros_args = rclpy.utilities.remove_ros_args(sys.argv)
    
    #  解析自定义参数（使用过滤后的参数列表）
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str,
                        default=str(datas_dir / 'SAUVC_sim52.pt'), help='model.pt path(s)')
    parser.add_argument('--weights2', type=str,
                        default=str(datas_dir / 'SAUVC_sim52.pt'), help='model.pt path(s)')
    parser.add_argument('--show-img', type=bool,
                        default=True, help='是否展示图像')
    parser.add_argument('--front-params', type=str,
                        default=str(datas_dir / 'front.npz'), help='前置摄像头参数存储路径')
    parser.add_argument('--down-params', type=str,
                        default=str(datas_dir / 'down.npz'), help='下置摄像头参数存储路径')

    # 注意：解析时跳过脚本名本身（ros_args[0]是脚本名）
    opt = parser.parse_args(ros_args[1:])

    # 初始化ROS 2节点
    rclpy.init(args=args)

    parent_conn_1, child_conn_1 = Pipe()
    parent_conn_2, child_conn_2 = Pipe()
    p_1 = Process(target=detect_process, args=(child_conn_1, opt))
    p_2 = Process(target=detect_process, args=(child_conn_2, opt))
    p_1.start()
    p_2.start()

    node = AiNode("uv_detect_demo", opt, parent_conn_1, parent_conn_2)
    rclpy.spin(node)
    rclpy.shutdown()

    # 进程清理（避免僵尸进程）
    p_1.join()
    p_2.join()