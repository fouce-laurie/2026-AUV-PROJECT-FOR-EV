import rclpy
from rclpy.node import Node
import subprocess

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

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized

from sensor_msgs.msg import Image
from uv_msgs.srv import DetectRequest

from uv_vision.stereocam import StereoCamera

# 养了一只猫猫，路过的人可以摸一摸
# 　／l、
# （ﾟ､ 。 ７
# 　l、 ~ヽ
# 　じしf_, )ノ


# 图形归一化
def letterbox(im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # Resize and pad image while meeting stride-multiple constraints
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better val mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - \
        new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / \
            shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right,
                            cv2.BORDER_CONSTANT, value=color)  # add border
    return im, ratio, (dw, dh)


class AiNode(Node):
    def __init__(self, name, opt):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        self.opt = opt

        self.bridge = CvBridge()  # 图像格式转换器

        if opt.show_img:
            self.detectedimg_pub = self.create_publisher(
                Image, "detectedimg", 10)

        # 初始化相机参数
        fsc = StereoCamera()
        dsc = StereoCamera()
        self.stereocams = {"front": fsc, "down": dsc}
        self.stereocams["front"].cal_parameters_init(opt.front_params[0])
        self.stereocams["down"].cal_parameters_init(opt.down_params[0])

        # 初始化
        set_logging()
        self.device = select_device(opt.device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # 加载模型
        self.model = attempt_load(
            opt.weights, map_location=self.device)  # load FP32 model
        self.stride = int(self.model.stride.max())  # model stride
        self.imgsz = check_img_size(
            opt.img_size, s=self.stride)  # check img_size
        if self.half:
            self.model.half()  # to FP16

        # 获取对象名称和颜色
        self.names = self.model.module.names if hasattr(
            self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255)
                        for _ in range(3)] for _ in self.names]

        # 初始化服务端，服务类型 DetectRequest ，服务 uv_detect_srv ，回调函数 detectrequest_callback
        self.srv = self.create_service(
            DetectRequest, 'uv_detect_srv', self.detectrequest_callback)  # CHANGE

    # 在此对图像进行处理

    def detectrequest_callback(self, request, response):
        img0 = self.bridge.imgmsg_to_cv2(request.imagein, "bgr8")
        askedtarget = request.target
        response.s = 0
        _, width, _ = img0.shape
        halfwidth = width//2

        targetlocation = [[0, 0], [0, 0]]
        size_l = size_r = 0

        # 预测
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device).type_as(
                next(self.model.parameters())))  # run once
        t0 = time.time()

        img = letterbox(img0, self.imgsz, stride=self.stride)[0]

        img = img.transpose((2, 0, 1))[::-1]
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()
        img /= 255.

        if len(img.shape) == 3:
            img = img[None]

        # 预测
        t1 = time_synchronized()
        pred = self.model(img, augment=self.opt.augment)[0]

        # Apply NMS
        pred = non_max_suppression(
            pred, 0.45, 0.5, classes=self.opt.classes, agnostic=self.opt.agnostic_nms)
        t2 = time_synchronized()

        for i, det in enumerate(pred):
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(
                    img.shape[2:], det[:, :4], img0.shape).round()
                for *xyxy, conf, cls in reversed(det):
                    if self.opt.show_img:
                        plot_one_box(xyxy, img0, label=f'{self.names[int(cls)]} {conf:.2f}', color=self.colors[int(
                            cls)], line_thickness=3)

                    label = f'{self.names[int(cls)]} {conf:.2f}'
                    if self.names[int(cls)] == askedtarget and conf >= self.opt.score:
                        x = 0.5*(xyxy[2] + xyxy[0])
                        y = 0.5*(xyxy[1] + xyxy[3])
                        size = abs((xyxy[2] - xyxy[0])*(xyxy[1] - xyxy[3]))

                        if x <= halfwidth:
                            if size > size_l:
                                size_l = size
                                targetlocation[0][0] = float(x)
                                targetlocation[0][1] = float(y)
                        else:
                            if size > size_r:
                                size_r = size
                                targetlocation[1][0] = float(x)
                                targetlocation[1][1] = float(y)

        if self.opt.show_img:
            self.img_transmit(img0)

        if size_l > 0 and size_r > 0:
            response.s = 1

            # 目标物在左右图像中的位置
            x1, y1 = targetlocation[0][0], targetlocation[0][1]
            x2, y2 = targetlocation[1][0] - halfwidth, targetlocation[1][1]

            # 计算对应点的像素坐标
            left_pt = np.array([[x1], [y1]])
            right_pt = np.array([[x2], [y2]])
            # left_pt_undistorted = cv2.undistortPoints(
            #     left_pt,  self.stereocams[request.stero].camera_matrix_left, self.stereocams[request.stero].dist_coeffs_left)
            # right_pt_undistorted = cv2.undistortPoints(
            #     right_pt, self.stereocams[request.stero].camera_matrix_right, self.stereocams[request.stero].dist_coeffs_right)

            # 计算对应点的相机坐标
            pt = cv2.triangulatePoints(
                self.stereocams[request.stero].P1, self.stereocams[request.stero].P2, left_pt, right_pt)

            # 计算对应点在世界坐标系中的位置
            pt_ = pt/pt[3][0]
            pt = pt_[:3, :]
            response.x, response.y, response.z = -pt[0][0], -pt[1][0], pt[2][0]

        if response.s == 1:
            self.get_logger().info(f'已检测到目标,用时({time.time() - t0:.3f}s)')
        else:
            self.get_logger().info(f'未检测到目标,用时({time.time() - t0:.3f}s)')

        return response

    # 传递图像
    def img_transmit(self, frame):
        frame = np.array(frame)
        data = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.detectedimg_pub.publish(data)


def main(args=None):
    # 加载参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str,
                        default=['/home/nvidia/Workspace/Cruise/datas/auv.pt'], help='model.pt path(s)')
    parser.add_argument('--img-size', type=int, default=640,
                        help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float,
                        default=0.45, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float,
                        default=0.5, help='IOU threshold for NMS')
    parser.add_argument('--device', default='0',
                        help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--classes', nargs='+', type=int,
                        help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true',
                        help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true',
                        help='augmented inference')

    parser.add_argument('--show-img', type=bool,
                        default=True, help='是否展示图像')
    parser.add_argument('--show-fps', type=bool, default=True, help='是否显示fps')
    parser.add_argument('--show-label', type=bool,
                        default=False, help='是否显示label')
    parser.add_argument('--show-thickness', type=int,
                        default=2, help='BoundingBox边框粗细')
    parser.add_argument('--score', type=float,
                        default=0.2, help='置信分数')

    parser.add_argument('--front-params', nargs='+', type=str,
                        default=['/home/nvidia/Workspace/Cruise/datas/front.npz'], help='前置摄像头参数存储路径')
    parser.add_argument('--down-params', nargs='+', type=str,
                        default=['/home/nvidia/Workspace/Cruise/datas/down.npz'], help='下置摄像头参数存储路径')

    opt = parser.parse_args()

    rclpy.init(args=args)  # 初始化rclpy
    node = AiNode("uv_detect", opt)  # 新建一个节点
    rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()  # 关闭rclpy
