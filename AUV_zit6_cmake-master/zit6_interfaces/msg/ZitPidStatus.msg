# 全轴 PID 与 规划器状态汇总反馈 (1Hz)

# 位置环 P 参数与规划器限值 [X, Y, Z, Yaw]
float32[4] pos_kp
float32[4] pos_max_v
float32[4] pos_max_a
float32[4] pos_out_limit

# 速度环 PID 参数 [X, Y, Z, Yaw]
float32[4] vel_kp
float32[4] vel_ki
float32[4] vel_kd
float32[4] vel_i_limit
float32[4] vel_out_limit
