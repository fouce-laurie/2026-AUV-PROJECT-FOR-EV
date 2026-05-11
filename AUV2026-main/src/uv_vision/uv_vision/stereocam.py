import numpy as np
import cv2
import os


def stereoMatchSGBM(left_image, right_image, down_scale=False):
    # 视差计算

    # SGBM匹配参数设置
    if left_image.ndim == 2:
        img_channels = 1
    else:
        img_channels = 3
    blockSize = 3
    paraml = {'minDisparity': 0,  # Minimum possible disparity value
              'numDisparities': 128,  # Maximum disparity minus minimum disparity
              'blockSize': blockSize,  # Matched block size
              # The first parameter controlling the disparity smoothness
              'P1': 8 * img_channels * blockSize ** 2,
              # The second parameter controlling the disparity smoothness
              'P2': 32 * img_channels * blockSize ** 2,
              'disp12MaxDiff': 1,    # 左右视差检查中允许的最大差异(以整数像素为单位)
              'preFilterCap': 63,    # 预滤波图像像素的截断值
              'uniquenessRatio': 15,  # 通常,5-15范围内的值就足够了
              'speckleWindowSize': 0,  # 平滑视差区域的最大尺寸,以考虑其噪声斑点和无效。将其设置为0可禁用斑点过滤。否则,将其设置在50-200的范围内
              'speckleRange': 1,         # 每个连接组件内的最大视差变化。
              'mode': cv2.STEREO_SGBM_MODE_SGBM_3WAY  #
              }

    # 构建SGBM对象
    left_matcher = cv2.StereoSGBM_create(**paraml)
    paramr = paraml
    paramr['minDisparity'] = -paraml['numDisparities']
    right_matcher = cv2.StereoSGBM_create(**paramr)

    # 计算视差图
    size = (left_image.shape[1], left_image.shape[0])
    if down_scale == False:
        disparity_left = left_matcher.compute(left_image, right_image)
        disparity_right = right_matcher.compute(right_image, left_image)

    else:
        # 实现高斯金字塔中的下采样,抛弃偶数行和偶数列
        left_image_down = cv2.pyrDown(left_image)
        right_image_down = cv2.pyrDown(right_image)
        factor = left_image.shape[1] / left_image_down.shape[1]

        disparity_left_half = left_matcher.compute(
            left_image_down, right_image_down)
        disparity_right_half = right_matcher.compute(
            right_image_down, left_image_down)
        disparity_left = cv2.resize(
            disparity_left_half, size, interpolation=cv2.INTER_AREA)
        disparity_right = cv2.resize(
            disparity_right_half, size, interpolation=cv2.INTER_AREA)
        disparity_left = factor * disparity_left
        disparity_right = factor * disparity_right

    # 真实视差(因为SGBM算法得到的视差是x16的)
    trueDisp_left = disparity_left.astype(np.float32) / 16
    trueDisp_right = disparity_right.astype(np.float32) / 16

    return trueDisp_left, trueDisp_right


def getDepthMap(disparityMap: np.ndarray, config) -> np.array:
    """
    :param disparityMap:输入视差图
    :return:
    """

    # 得到视差图后,就可以计算像素深度
    # 利用opencv函数计算深度图

    # cv2.reprojectImageTo3D将像素坐标转换为三维坐标,该函数会返回一个3通道的矩阵,分别存储X、Y、Z坐标(左摄像机坐标下)
    fb = config.camera_matrix_left[0, 0] * (-config.T[0])
    doffs = config.doffs
    depthMap = np.divide(fb, disparityMap + doffs)
    reset_index = np.where(np.logical_or(depthMap < 0.0, depthMap > 50000.0))
    depthMap[reset_index] = 0
    reset_index2 = np.where(disparityMap < 0.0)
    depthMap[reset_index2] = 0
    return depthMap.astype(np.float32)


class StereoCamera(object):
    def __init__(self):

        self.cap = None

        self.camera_matrix_left = None  # 左相机内参
        self.camera_matrix_right = None  # 右相机内参

        self.dist_coeffs_left = None  # 左相机畸变系数
        self.dist_coeffs_right = None  # 右相机畸变系数

        self.R = None  # 左相机至右相机的旋转矩阵
        self.T = None  # 左相机至右相机的平移矩阵

        # 主点列坐标的差
        self.doffs = 0.0

        # 指示上述内外参是否为经过立体校正后的结果
        self.isRectified = False

        # 相机矩阵
        self.R1 = None
        self.R2 = None
        self.P1 = None
        self.P2 = None
        self.Q = None

        self.map1_left = None
        self.map2_left = None
        self.map1_right = None
        self.map2_right = None

        # 图像大小
        self.half_size = (1280, 960)
        self.size = (self.half_size[0]*2, self.half_size[1])

        # 主点列坐标的差
        self.doffs = 0.0

    def cal_parameters_init(self, path):
        calib_file = np.load(path)

        self.camera_matrix_left = calib_file["camera_matrix_left"]
        self.dist_coeffs_left = calib_file["dist_coeffs_left"]
        self.camera_matrix_right = calib_file["camera_matrix_right"]
        self.dist_coeffs_right = calib_file["dist_coeffs_right"]

        self.R1 = calib_file["R1"]
        self.R2 = calib_file["R2"]

        self.P1 = calib_file["P1"]
        self.P2 = calib_file["P2"]

        self.R = calib_file["R"]
        self.T = calib_file["T"]

        self.Q = calib_file["Q"]

    def device_parameter_config(self, device=0, stream_format=cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), size=(2560, 960), fps=30):
        """
        @param stream_format:视频流格式
        @param size: 设置图像大小
        @param fps: 设置帧率
        """
        self.cap = cv2.VideoCapture(device)

        self.size = size
        self.half_size = (size[0]//2, size[1])

        self.cap.set(6, stream_format)  # 视频流格式
        self.cap.set(5, fps)  # 帧率
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])  # 设置宽度
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])  # 设置高度

    def cam_init(self):
        """
        向摄像头发送捕获指令,使摄像头开启
        """

        for i in range(30):
            self.cap.read()

    def capture(self):
        """
        @return s:是否捕获成功
        @return f:捕获图像
        """

        for i in range(10):
            s, f = self.cap.read()
            if s:
                f = cv2.flip(f, -1)
                return s, f
        return s, f

    def get_halfsize(self, image):
        """
        @param image:输入图像
        @return size:图像大小
        """

        height, width, channels = image.shape
        size = (width//2, height)
        return size

    def get_fullimg(self, img1, img2):
        """
        合并图像
        @param image_left:输入左侧图像
        @param image_right:输入右侧图像
        """
        result = cv2.hconcat([img1, img2])
        return result

    def get_halfimg(self, image):
        """
        @param image:输入图像
        @return image_left:左侧图像
        @return image_right:右侧图像
        """

        height, width, channels = image.shape
        image_left = image[:, 0:width//2, :]
        image_right = image[:, width//2:width, :]
        return image_left, image_right

    def rectification_init(self):
        """
        计算摄像机转换映射
        """

        # self.R1, self.R2, self.P1, self.P2, self.Q, _, _ = cv2.stereoRectify(
        #     self.camera_matrix_left, self.dist_coeffs_left, self.camera_matrix_right, self.dist_coeffs_right, self.half_size, self.R, self.T, alpha=1)

        # 计算左右摄像机的转换映射
        self.map1_left, self.map2_left = cv2.initUndistortRectifyMap(
            self.camera_matrix_left, self.dist_coeffs_left, self.R1, self.P1, self.half_size, cv2.CV_32FC1)
        self.map1_right, self.map2_right = cv2.initUndistortRectifyMap(
            self.camera_matrix_right, self.dist_coeffs_right, self.R2, self.P2, self.half_size, cv2.CV_32FC1)

    def preprocess(self, image_left, image_right):
        """
        图像预处理,用于减小光照影响
        @param image_left:输入左侧图像
        @param image_right:输入右侧图像
        """
        if (image_left.ndim == 3):
            # 通过opencv加载的图像通道顺序是BGR
            image_left = cv2.cvtColor(image_left, cv2.COLOR_BGR2GRAY)
        if (image_right.ndim == 3):
            image_right = cv2.cvtColor(image_right, cv2.COLOR_BGR2GRAY)

        # 直方图均衡
        image_left = cv2.equalizeHist(image_left)
        image_right = cv2.equalizeHist(image_right)

        return image_left, image_right

    def rectifyImage(self, image_left, image_right):
        """
        校正图像
        @param image_left:输入左侧图像
        @param image_right:输入右侧图像
        """
        rectified_left = cv2.remap(
            image_left, self.map1_left, self.map2_left, cv2.INTER_AREA)
        rectified_right = cv2.remap(
            image_right, self.map1_right, self.map2_right, cv2.INTER_AREA)
        return rectified_left, rectified_right

    def getdepth(self, rectified_left, rectified_right):
        """
        获取深度
        @param image_left:输入左侧图像(已校正)
        @param image_right:输入右侧图像(已校正)
        @return depthMap:深度图(基于已校正的右侧图像)
        """

        # 获取两幅图像的视差值
        disp, _ = stereoMatchSGBM(rectified_left, rectified_right, False)

        # 计算深度图
        depthMap = getDepthMap(disp, self)

        return depthMap

    def depth2img(self, depthMap):
        """
        将深度信息可视化
        @param depthMap:深度图
        @return depth_image:深度图(可视化)
        """
        depthMap = -depthMap

        minDepth = np.min(depthMap)
        maxDepth = np.max(depthMap)

        depthMapVis = (255.0 * (depthMap - minDepth)) / (maxDepth - minDepth)
        depth_image = depthMapVis.astype(np.uint8)
        depth = -depthMap[self.half_size[1]//2,
                          self.half_size[0]//2] / 0.025 / 2

        return depth, depth_image
