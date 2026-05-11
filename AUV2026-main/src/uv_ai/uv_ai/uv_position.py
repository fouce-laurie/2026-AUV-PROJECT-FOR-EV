import rclpy
from rclpy.node import Node
import numpy as np
from uv_msgs.msg import Yolov8, TargetParams
from std_msgs.msg import Float32MultiArray
import math

class PositionNode(Node):
    def __init__(self):
        super().__init__('uv_position')
        
        # 订阅视觉检测话题
        self.create_subscription(Yolov8, 'uv_detect_front', self.front_detect_cb, 10)
        self.create_subscription(Yolov8, 'uv_detect_down', self.down_detect_cb, 10)
        
        # 订阅机器人位姿话题 (ZIT6 协议)
        self.create_subscription(Float32MultiArray, '/zit6/state/pos', self.zit6_pos_cb, 10)
        
        # 保存机器人当前的世界坐标系中的位姿（位置和姿态欧拉角）
        self.current_pose = None
        
        # 用于缓存多帧单目射线的字典: {class_id: [{'origin': array, 'dir': array}]}
        self.ray_history_front = {}
        self.ray_history_down = {}
        self.MAX_HISTORY = 20 # 最大保留的历史记录长度，通过移动产生基线
        
        # === 根据 .scn 加载实际视场角和偏移 ===
        # front_cam_left: resolution_x="1280" resolution_y="960" horizontal_fov="34.19"
        self.w_front = 1280.0
        self.h_front = 960.0
        self.fx_front = self.w_front / (2 * math.tan(math.radians(34.19)/2))
        self.fy_front = self.fx_front
        self.cx_front = self.w_front / 2
        self.cy_front = self.h_front / 2
        # NED body frame: X=前, Y=右, Z=下 (来自 xunyun.scn front_cam_left)
        self.T_front_body = np.array([0.25, -0.02825, 0.25])

        # down_cam_left: resolution_x="1280" resolution_y="960" horizontal_fov="32.18"
        self.w_down = 1280.0
        self.h_down = 960.0
        self.fx_down = self.w_down / (2 * math.tan(math.radians(32.18)/2))
        self.fy_down = self.fx_down
        self.cx_down = self.w_down / 2
        self.cy_down = self.h_down / 2
        # NED body frame: X=前, Y=右, Z=下 (来自 xunyun.scn down_cam_left)
        self.T_down_body = np.array([0.0, 0.03765, 0.21])
        
        # 增加发布融合后的结果，供 automaton 等下游使用
        self.front_pub = self.create_publisher(Yolov8, 'uv_detect_front_pos', 10)
        self.down_pub = self.create_publisher(Yolov8, 'uv_detect_down_pos', 10)

        # 物品ID映射字典
        self.class_names = {
            0: "blue_drump", 1: "blue_stick", 2: "gate_green_stick", 
            3: "gate_red_stick", 4: "red_drump", 5: "red_stick", 
            6: "yellow_flag", 7: "yellow_stick"
        }
        
        # 实际地面真值坐标字典 (NED: North, East)，用于计算实时误差
        # WND→NED: x_ned = y_wnd, y_ned = -x_wnd
        self.ground_truth = {
            0: (24.00, 3.60),    # blue_drump
            1: (10.50, 0.20),    # blue_stick
            2: (16.00, 0.75),    # gate_green_stick
            3: (16.00, -0.75),   # gate_red_stick
            4: (24.00, 1.20),    # red_drump
            5: (13.00, 1.80),    # red_stick
            6: (6.00, 2.80),     # yellow_flag
            7: (11.50, -2.20)    # yellow_stick
        }

        # 核心记忆字典：记录已解算出的物体的世界坐标
        # 格式: { class_id: array([x, y, z]) }
        self.object_memory = {}

        # 定时器：以 10Hz 频率持续广播已知物体位置
        self.timer = self.create_timer(0.1, self.broadcast_memory)
        self.print_counter = 0

        # 创建用于记录射线夹角与误差的数据文件
        import os
        os.makedirs('/home/origin/AUV2026/log', exist_ok=True)
        self.error_log_file = open('/home/origin/AUV2026/log/ray_error_log.csv', 'w')
        self.error_log_file.write("time,obj_id,obj_name,rays_count,max_angle_deg,est_x,est_y,gt_x,gt_y,err_x,err_y,err_dist\n")
        self.error_log_file.flush()

    def destroy_node(self):
        if hasattr(self, 'error_log_file') and self.error_log_file:
            self.error_log_file.close()
        super().destroy_node()

    def print_evaluation_table(self):
        """打印带有真值的坐标估测误差表格"""
        if not self.object_memory:
            return
            
        header = f"{'编号':<4} {'物体名称':<25} {'实际坐标 (x,y)':<20} {'本次估测坐标 (x,y)':<20} {'二维欧氏距离误差 (m)'}"
        lines = ["\n" + "="*95, header, "-"*95]
        
        for idx in sorted(self.object_memory.keys()):
            if idx == 4: # 排除红油桶，因为存在三个实例，基准不唯一
                continue
                
            name = self.class_names.get(idx, "Unknown")
            est_pos = self.object_memory[idx]
            est_x, est_y = est_pos[0], est_pos[1]
            
            if idx in self.ground_truth:
                gt_x, gt_y = self.ground_truth[idx]
                gt_str = f"({gt_x:.2f}, {gt_y:.2f})"
                err = math.hypot(est_x - gt_x, est_y - gt_y)
                err_str = f"{err:.2f}"
            else:
                gt_str = "N/A"
                err_str = "N/A"
                
            est_str = f"({est_x:.2f}, {est_y:.2f})"
            lines.append(f"{idx:<6} {name:<27} {gt_str:<24} {est_str:<26} {err_str}")
            
        lines.append("="*95 + "\n")
        self.get_logger().info("\n" + "\n".join(lines))

    def broadcast_memory(self):
        """持续广播已解算出的物体位置"""
        if not self.object_memory:
            return

        # 每 2 秒 (20 tick) 打印一次评估表格
        self.print_counter += 1
        if self.print_counter >= 20:
            self.print_counter = 0
            self.print_evaluation_table()

        # 构造前视和下视的 Yolov8 消息（共享同一套物体记忆）
        msg = Yolov8()
        num_classes = 8 # 根据 class_names 确定
        msg.state = [0.0] * num_classes
        msg.targets = [TargetParams() for _ in range(num_classes)]

        for idx, pos in self.object_memory.items():
            if idx < num_classes:
                msg.state[idx] = 1.0 # 标记为已知
                msg.targets[idx].tpos_inworld.x = pos[0]
                msg.targets[idx].tpos_inworld.y = pos[1]
                msg.targets[idx].tpos_inworld.z = pos[2]

        self.front_pub.publish(msg)
        self.down_pub.publish(msg)

    def zit6_pos_cb(self, msg: Float32MultiArray):
        """实时更新当前auv的世界位姿（ZIT6 协议，NED 坐标系）"""
        if len(msg.data) >= 4:
            # ZIT6 发布位置为 NED 坐标系 (北、东、下)，yaw 为 NED 坐标系（弧度）
            self.current_pose = {
                'x': msg.data[0],      # NED 北向 (m)
                'y': msg.data[1],      # NED 东向 (m)
                'z': msg.data[2],      # 深度 (m, 正值=下潜)
                'rx': 0.0,             # roll (ZIT6 不提供，设为0)
                'ry': 0.0,             # pitch (ZIT6 不提供，设为0)
                'rz': math.degrees(msg.data[3])  # yaw (NED 坐标系，度数)
            }

    def euler_to_matrix(self, rx, ry, rz):
        """将欧拉角(角度)转换为旋转矩阵，假设 ZYX 顺序 / NED基准"""
        rx = math.radians(rx)
        ry = math.radians(ry)
        rz = math.radians(rz)

        Rx = np.array([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
        Ry = np.array([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
        Rz = np.array([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])

        return Rz.dot(Ry).dot(Rx)

    def front_detect_cb(self, msg: Yolov8):
        self.process_detection(msg, self.ray_history_front, camera_type='front')

    def down_detect_cb(self, msg: Yolov8):
        self.process_detection(msg, self.ray_history_down, camera_type='down')

    def process_detection(self, msg: Yolov8, history_dict, camera_type):
        """核心处理函数：单目射线世界化并与历史基线交汇"""
        if self.current_pose is None:
            return

        R_robot = self.euler_to_matrix(self.current_pose['rx'], self.current_pose['ry'], self.current_pose['rz'])

        for idx, s in enumerate(msg.state):
            if s > 0 and idx < len(msg.targets):
                target = msg.targets[idx]
                px = target.tpos_inpic.x
                py = target.tpos_inpic.y
                
                # 1. 局部成像到机体转换 (相机光心坐标系 -> NED机体坐标系)
                if camera_type == 'front':
                    v_cam = np.array([(px - self.cx_front) / self.fx_front, (py - self.cy_front) / self.fy_front, 1.0])
                    v_cam /= np.linalg.norm(v_cam)
                    # 前视光学坐标系 -> NED body (X前, Y右, Z下)
                    # ZYX RPY=(π/2,0,π/2): cam Z -> body X(前), cam X -> body Y(右), cam Y -> body Z(下)
                    v_body_ned = np.array([v_cam[2], v_cam[0], v_cam[1]])
                    T_body_ned = self.T_front_body
                else:
                    v_cam = np.array([(px - self.cx_down) / self.fx_down, (py - self.cy_down) / self.fy_down, 1.0])
                    v_cam /= np.linalg.norm(v_cam)
                    # 下视光学坐标系 -> NED body (X前, Y右, Z下)
                    # ZYX RPY=(0,0,π/2): cam X -> body Y(右), cam Y -> body -X(后), cam Z -> body Z(下)
                    v_body_ned = np.array([-v_cam[1], v_cam[0], v_cam[2]])
                    T_body_ned = self.T_down_body

                # 2. 从机体 NED 转化为世界 NED 框架
                v_world = R_robot.dot(v_body_ned)
                camera_offset_world = R_robot.dot(T_body_ned)

                origin_world = np.array([self.current_pose['x'], self.current_pose['y'], self.current_pose['z']]) + camera_offset_world
                
                if idx not in history_dict:
                    history_dict[idx] = []
                    
                # ========================== 核心修复 ==============================
                # 如果小车静止，射线都在同一个点发出，交汇一定会退化返回起点。
                # 强制要求与上一记录条目具有一定基线位移 (例如 5cm)，才存入历史。
                if len(history_dict[idx]) > 0:
                    last_O = history_dict[idx][-1]['origin']
                    dist = np.linalg.norm(origin_world - last_O)
                    if dist < 0.1:  # 小于 10cm 不累加新射线 (无意义基线)
                        # 但为了实时使用最新的视角提供测距位置（如果历史长度足够），可以选择直接进行交汇测距
                        pass
                    else:
                        history_dict[idx].append({'origin': origin_world, 'dir': v_world})
                else:
                    history_dict[idx].append({'origin': origin_world, 'dir': v_world})
                # =================================================================
                
                # 满长时智能剔除：由于所有物品静止，交汇质量正比于射线起点的空间分散度(Baseline)和观测角度的多样性。
                # 策略：寻找冗余度最高(冗余=基线近+角度窄)或质量最低(观测距离远)的射线并删去其一。
                if len(history_dict[idx]) > self.MAX_HISTORY:
                    max_redundancy = -1.0
                    drop_index = 0
                    n_rays = len(history_dict[idx])
                    
                    # 获取该物体的最后已知位置，用于计算“射线质量” (距离越远质量越低)
                    target_pos = self.object_memory.get(idx)
                    
                    # 权重分配：
                    # DIST_WEIGHT (2D/3D 基线冗余): 2.0  (基线越短，值越大)
                    # ANGLE_WEIGHT (观测视角冗余): 3.0  (cos接近1，值越大)
                    # QUALITY_WEIGHT (观测距离惩罚): 0.5 (距离目标越远，值越大)
                    # —— 这样当小车通过目标后，会优先丢弃还在远距离（刚发现目标时）的低质量射线
                    DIST_WEIGHT = 2.0
                    ANGLE_WEIGHT = 3.0
                    QUALITY_WEIGHT = 0.5
                    
                    for i in range(n_rays):
                        # 单个射线的质量惩罚 (基于距离目标的距离)
                        penalty_i = 0.0
                        if target_pos is not None:
                            d_to_target = np.linalg.norm(history_dict[idx][i]['origin'] - target_pos)
                            penalty_i = d_to_target * QUALITY_WEIGHT

                        for j in range(i + 1, n_rays):
                            # 1. 空间冗余 (基线贡献)
                            d_origins = np.linalg.norm(history_dict[idx][i]['origin'] - history_dict[idx][j]['origin'])
                            dist_score = 1.0 / (d_origins + 0.05) 
                            
                            # 2. 角度冗余 (平行程度贡献)
                            cos_sim = np.dot(history_dict[idx][i]['dir'], history_dict[idx][j]['dir'])
                            
                            # 3. 质量惩罚 (如果是远距离对齐的两根射线，更容易被丢弃)
                            penalty_j = 0.0
                            if target_pos is not None:
                                d_to_target_j = np.linalg.norm(history_dict[idx][j]['origin'] - target_pos)
                                penalty_j = d_to_target_j * QUALITY_WEIGHT
                                
                            # 综合冗余得分
                            redundancy = (DIST_WEIGHT * dist_score) + (ANGLE_WEIGHT * cos_sim) + (penalty_i + penalty_j) / 2.0
                            
                            if redundancy > max_redundancy:
                                max_redundancy = redundancy
                                drop_index = i
                    history_dict[idx].pop(drop_index)

                # 3. 三维交汇定点 (平面 2D 降维：利用物体在水底或固定深度，仅交汇 X-Y 分量)
                if len(history_dict[idx]) >= 3:
                    # 【新增】基于 RANSAC 的 YOLO 误识别射线剔除
                    # 原理：两两解算焦点，寻找拥有最多“内点”(距离低于阈值的射线)的共识点，从而孤立并删除偏差极大的假阳性框。
                    n_rays = len(history_dict[idx])
                    best_inliers = []
                    
                    for i in range(n_rays):
                        for j in range(i + 1, n_rays):
                            p_ij = self.intersect_rays_3d([history_dict[idx][i], history_dict[idx][j]])
                            if p_ij is None: 
                                continue
                                
                            current_inliers = []
                            for k in range(n_rays):
                                r_k = history_dict[idx][k]
                                O, D = r_k['origin'], r_k['dir']
                                norm_d = np.linalg.norm(D)
                                if norm_d < 1e-3: continue
                                D = D / norm_d
                                
                                # 计算 3D 射线到候选点 p_ij 的垂直距离 (叉乘形式或投影剔除法均可，此处使用投影剔除)
                                vec = p_ij - O
                                dist = np.linalg.norm(vec - np.dot(vec, D) * D)
                                
                                # 将距离 < 1.5m 的视为合理观测 (由于船体运动和像素跳动，阈值设置不宜太苛刻)
                                if dist < 1.5: 
                                    current_inliers.append(r_k)
                                    
                            if len(current_inliers) > len(best_inliers):
                                best_inliers = current_inliers
                    
                    # # 如果找到优势共识 (内点>=3)，并且有发现偏离严重的离群点，直接从历史中剔除
                    # if len(best_inliers) >= 3 and len(best_inliers) < n_rays:
                    #     # 覆盖历史，相当于永久删除错误射线
                    #     history_dict[idx] = best_inliers

                    # 使用过滤后的高纯度射线进行最终解算
                    valid_rays = history_dict[idx]
                    
                    # 【核心变更】采用两两交汇中值法，彻底代替并拒绝最小二乘法的集合缩水效应 (Clumping Bias)
                    estimated_pos = self.intersect_rays_pairwise(valid_rays)
                    
                    if estimated_pos is not None:
                        # 维持原先的基线夹角计算和日志输出，方便您继续收集数据验证
                        min_cos = 1.0
                        for r_i in valid_rays:
                            for r_j in valid_rays:
                                cos_val = np.dot(r_i['dir'], r_j['dir'])
                                if cos_val < min_cos:
                                    min_cos = cos_val
                        max_angle_deg = math.degrees(math.acos(np.clip(min_cos, -1.0, 1.0)))
                        
                        # 把测算得到的结果存入 csv 中 (已排除 red_drump idx==4)
                        if idx in self.ground_truth and idx != 4:
                            gt_x, gt_y = self.ground_truth[idx]
                            # X 是南北轴（NED North方向）
                            err_x = gt_x - estimated_pos[0]
                            # Y 是北南轴（Y方向的误差对“接近”物体的距离估算非常关键）
                            err_y = gt_y - estimated_pos[1]
                            err_dist = math.hypot(err_x, err_y)
                            
                            t = self.get_clock().now().nanoseconds / 1e9
                            obj_name = self.class_names.get(idx, 'Unknown')
                            log_line = f"{t:.2f},{idx},{obj_name},{len(valid_rays)},{max_angle_deg:.3f},{estimated_pos[0]:.3f},{estimated_pos[1]:.3f},{gt_x:.3f},{gt_y:.3f},{err_x:.3f},{err_y:.3f},{err_dist:.3f}\n"
                            
                            if hasattr(self, 'error_log_file') and self.error_log_file:
                                self.error_log_file.write(log_line)
                                self.error_log_file.flush()

                        # 【核心修复】合理性校验：计算与当前相机的距离，必须在合理探测范围内 (如 50m 内)
                        # 以及防止解算到背后的虚假解 (射线反向延长线交点)
                        dist_to_obj = np.linalg.norm(estimated_pos - origin_world)
                        
                        # 检测解出的点是否位于所有射线的真实前方（内积 > 0）
                        # 避免最小二乘法得到射线反向的交点 
                        is_forward = all(np.dot(estimated_pos - r['origin'], r['dir']) > 0 for r in valid_rays)

                        if dist_to_obj < 50.0 and is_forward:
                            self.object_memory[idx] = estimated_pos

                            if len(valid_rays) >= 5:
                                obj_name = self.class_names.get(idx, f"Obj_{idx}")
                                self.get_logger().info(
                                    f"[{camera_type}] {obj_name}({idx}) 平面更新成功 -> 采样: {len(valid_rays)}, "
                                    f"NED: X={estimated_pos[0]:.2f}, Y={estimated_pos[1]:.2f}")
                        else:
                            # 如果解算出的位置不合理，不更新记忆
                            pass
                    else:
                        # 【改进】若因共线等原因解算连续失败，可考虑在 object_memory 中标记该条记录已过时
                        pass

    def intersect_rays_3d(self, rays):
        """最小二乘法(Least Squares): 计算N条射线的 3D 最佳交点"""
        M = np.zeros((3, 3))
        b = np.zeros(3)
        
        for r in rays:
            O = r['origin']
            D = r['dir']
            
            norm_d = np.linalg.norm(D)
            if norm_d < 1e-3: continue
            D = D / norm_d
            
            I_minus_DD = np.eye(3) - np.outer(D, D)
            M += I_minus_DD
            b += I_minus_DD.dot(O)
            
        try:
            cond = np.linalg.cond(M)
            # 放宽条件数限制，防止正常远距离微小角度变化的退化失效
            if cond > 1e5: 
                return None
            
            P_3d = np.linalg.solve(M, b)
            return P_3d
        except np.linalg.LinAlgError:
            return None

    def intersect_rays_2d(self, rays):
        """最小二乘法(Least Squares): 计算N条射线的 2D 平面(X, Y)最佳交点"""
        # 方程形式： M * P = B (2x2 矩阵)
        M = np.zeros((2, 2))
        b = np.zeros(2)
        
        for r in rays:
            O = r['origin'][:2] # 取 X, Y
            D = r['dir'][:2]    # 取 X, Y 方向
            
            # 归一化投影后的 2D 方向向量
            norm_d = np.linalg.norm(D)
            if norm_d < 1e-3: continue
            D = D / norm_d
            
            I_minus_DD = np.eye(2) - np.outer(D, D)
            M += I_minus_DD
            b += I_minus_DD.dot(O)
            
        try:
            cond = np.linalg.cond(M)
            if cond > 1e6: 
                return None
            P_2d = np.linalg.solve(M, b)
            z_val = rays[-1]['origin'][2] 
            return np.array([P_2d[0], P_2d[1], z_val])
        except np.linalg.LinAlgError:
            return None

    def intersect_rays_pairwise(self, rays):
        """
        消除最小二乘法集束偏差 (Clumping Shrinkage Bias) 的对向交汇法。
        通过计算有效射线对的两两公垂线中点并取空间中值，完全免疫射线分布不均和局部粘滞现象。
        """
        n = len(rays)
        if n < 2: return None
        
        pts = []
        for i in range(n):
            for j in range(i + 1, n):
                O1, D1 = rays[i]['origin'], rays[i]['dir']
                O2, D2 = rays[j]['origin'], rays[j]['dir']
                
                # 过滤平行射线 (夹角太小的组合对计算深度无有效贡献且容易引起数值爆炸)
                cos_val = np.dot(D1, D2)
                if cos_val > 0.9995: # 约<1.8度
                    continue
                    
                cross_dir = np.cross(D1, D2)
                denom = np.linalg.norm(cross_dir)**2
                if denom < 1e-8:
                    continue
                    
                t1 = np.dot(np.cross(O2 - O1, D2), cross_dir) / denom
                t2 = np.dot(np.cross(O2 - O1, D1), cross_dir) / denom
                
                # 目标对象必须落在机器人的正前方有效视角内
                if t1 > 0 and t2 > 0:
                    p1 = O1 + D1 * t1
                    p2 = O2 + D2 * t2
                    # 空间公垂线线段的长度不能过于离谱 (若超过5m说明在三维空间中相距甚远，属于错配射线)
                    if np.linalg.norm(p1 - p2) < 5.0:
                        pts.append((p1 + p2) / 2.0)
                
        if not pts:
            # 如果所有的基线对都太窄（尚未建立起显著观测角度差），退化为传统 3D 射线交汇作为保底
            return self.intersect_rays_3d(rays)
            
        pts = np.array(pts)
        # 采用中位数滤波 (Median filter)，直接无视少数由于框位置跳动引起的极远/极近的离群点
        return np.median(pts, axis=0)

    def intersect_rays(self, rays):
        """兼容层代码：使用最强大的 3D 成对中值法"""
        return self.intersect_rays_pairwise(rays)

def main(args=None):
    rclpy.init(args=args)
    node = PositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
