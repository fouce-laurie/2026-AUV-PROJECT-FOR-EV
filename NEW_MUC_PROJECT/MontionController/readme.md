# MontionController实现功能
## 1.正常工作模式
### 数据格式 FA AF 0x01  F<sub>x</sub> F<sub>y</sub> F<sub>z</sub> F<sub>yaw</sub> F<sub>pitch</sub> F<sub>roll</sub> Angle<sub>servo</sub> FB BF(bbbfffffffbb)
## 2.配置推力曲线
### 数据格式 FA AF 0x07 0(0是读 1是写) i motor[i]关于推力曲线的8个值 FB BF(bbbbffffffffbb)
            memcpy(&(thrustcurve[buf[3]].pwm[0]), buf + 4, 4);
            memcpy(&(thrustcurve[buf[3]].pwm[1]), buf + 8, 4);
            memcpy(&(thrustcurve[buf[3]].pwm[2]), buf + 12, 4);
            memcpy(&(thrustcurve[buf[3]].pwm[3]), buf + 16, 4);

            memcpy(&(thrustcurve[buf[3]].thrust[0]), buf + 20, 4);
            memcpy(&(thrustcurve[buf[3]].thrust[1]), buf + 24, 4);
            memcpy(&(thrustcurve[buf[3]].thrust[2]), buf + 28, 4);
            memcpy(&(thrustcurve[buf[3]].thrust[3]), buf + 32, 4);
## 3.配置推力矩阵 
### 数据格式 FA AF 0x10 0(0是读 1是写) A_1 A_2 B C _2b FB BF(bbfffffbb)
