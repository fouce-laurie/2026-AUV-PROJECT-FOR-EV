import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import torch
from ultralytics import YOLO
import cv2
import numpy as np
import argparse
import sys


class SegmentNode(Node):
    def __init__(self, name, opt):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)

        # 初始化CVBridge
        self.bridge = CvBridge()

        # 初始化模型
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model = YOLO(opt.weights)
        self.model.to(self.device)

        # 订阅话题
        self.create_subscription(
            Image, 'down_cam/rectified/left', self.img_callback, 10)

        # 创建话题发布器，用于发布分割结果
        self.segment_img_pub = self.create_publisher(
            Image, 'segment_img', 10)

        # 创建话题发布器，用于发布二值化后的分割结果
        self.binary_segment_img_pub = self.create_publisher(
            Image, 'binary_segment_img', 10)

    def img_callback(self, msg):
        self.get_logger().info("接收到下视左相机图像信息。")
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if cv_image is None:
                self.get_logger().error("bridge.imgmsg_to_cv2 returned None")
                return

            # 进行语义分割
            results = self.model(cv_image)[0]

            if results.masks is not None:
                # 创建红色掩码图像
                red_mask = np.zeros_like(cv_image, dtype=np.uint8)
                binary_mask = np.zeros((cv_image.shape[0], cv_image.shape[1]), dtype=np.uint8)

                # 叠加每个掩码
                for mask in results.masks.data.cpu().numpy():
                    mask = cv2.resize(mask, (cv_image.shape[1], cv_image.shape[0]))  # 确保掩码尺寸匹配
                    mask = mask.astype(np.uint8)
                    binary_mask = cv2.bitwise_or(binary_mask, mask)
                    red_overlay = np.zeros_like(cv_image, dtype=np.uint8)
                    red_overlay[:, :, 2] = mask * 255
                    red_mask = cv2.addWeighted(red_mask, 1, red_overlay, 0.5, 0)

                # 将原始图像和红色掩码图像叠加
                overlay_image = cv2.addWeighted(cv_image, 1, red_mask, 0.5, 0)

                # 将二值化后的分割结果转换为ROS图像消息
                binary_image_msg = self.bridge.cv2_to_imgmsg(binary_mask * 255, "mono8")

                # 将带透明红色掩码的图像转换为ROS图像消息
                overlay_image_msg = self.bridge.cv2_to_imgmsg(overlay_image, "bgr8")

                # 发布带透明红色掩码的分割结果
                self.segment_img_pub.publish(overlay_image_msg)

                # 发布二值化后的分割结果
                self.binary_segment_img_pub.publish(binary_image_msg)
                self.get_logger().info("发布分割结果")
            else:
                self.get_logger().info("未检测到对象或分割结果。")

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge错误: {e}")
        except Exception as e:
            self.get_logger().error(f"错误: {e}")

def main(args=None):

    ros_args = rclpy.utilities.remove_ros_args(sys.argv)
    
    # 解析自定义参数（使用过滤后的参数列表）
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str,
                        default='/home/nvidia/Workspace/Cruise/datas/redline-7-2.pt', help='model.pt路径')

    # 注意：解析时跳过脚本名本身（ros_args[0]是脚本名）
    opt = parser.parse_args(ros_args[1:])

    # 初始化ROS 2节点（保持原逻辑）
    rclpy.init(args=args)
    node = SegmentNode("uv_segment", opt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

