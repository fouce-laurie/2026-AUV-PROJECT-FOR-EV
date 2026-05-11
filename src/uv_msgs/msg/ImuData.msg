uint8 mode

# mode 当前导航模式
# uint8
# 0x00 待机
# 0x01 粗对准
# 0x02 精对准
# 0x04 SINS/DVL
# 0x05 MRU (无DVL数据时自动进入姿态模式)
# 0xFF 系统故障

uint8 dvl

# dvl 传感器标志位，指示传感器状态
# 0x00 DVL 未上传数据
# 0x01 DVL 数据更新但无效
# 0x02 DVL 数据更新且有效

RobotAxis pos
RobotAxis spd
