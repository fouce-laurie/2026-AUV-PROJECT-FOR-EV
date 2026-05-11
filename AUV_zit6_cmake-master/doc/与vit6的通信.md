通信串口uart6 115200

1.推力下发(fx fy fz 扭矩)
数据格式 FA AF 0x01  F<sub>x</sub> F<sub>y</sub> F<sub>z</sub> F<sub>yaw</sub> F<sub>pitch</sub> F<sub>roll</sub>  FB BF(bbbfffffffbb)(不需要fpitch froll，一直都是0)

2.配置推力曲线
数据格式 FA AF 0x07 0(0是读 1是写) i motor[i]关于推力曲线的8个值 FB BF(bbbbffffffffbb)
            memcpy(&(thrustcurve[buf[3]].pwm[0]), buf + 4, 4);
            memcpy(&(thrustcurve[buf[3]].pwm[1]), buf + 8, 4);
            memcpy(&(thrustcurve[buf[3]].pwm[2]), buf + 12, 4);
            memcpy(&(thrustcurve[buf[3]].pwm[3]), buf + 16, 4);

            memcpy(&(thrustcurve[buf[3]].thrust[0]), buf + 20, 4);
            memcpy(&(thrustcurve[buf[3]].thrust[1]), buf + 24, 4);
            memcpy(&(thrustcurve[buf[3]].thrust[2]), buf + 28, 4);
            memcpy(&(thrustcurve[buf[3]].thrust[3]), buf + 32, 4);

3.舵机命令
数据格式 FA AF 0x02 angle FB BF(bbbfbb)
4.外接显示灯
灯的是 0xfa 0xaf 0x03 state  0xfb 0xbf    state 位uint8_t表示当前灯的 状态红黄蓝