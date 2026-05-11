#include "command.h"
#include "usart.h"
#include "motor.h"
#include "thrust.h"
#include <string.h>

extern UART_HandleTypeDef huart1;
extern ThrustParams_t thrust_params;


RecBuf uart1rec = {0};

void CommandInit(void)
{
    uart1rec.cnt = 0;
    HAL_UART_Receive_IT(&huart1, uart1rec.buf, 1);
}

void Command_Callback_NormalMode(NormalModeData_t *data) {
    float askedthrust[6];
    float motorthrust[6];
    
    // index 0~5 对应 x, y, z, rx(roll), ry(pitch), rz(yaw)
    askedthrust[0] = data->Fx;
    askedthrust[1] = data->Fy;
    askedthrust[2] = data->Fz;
    askedthrust[3] = data->Froll;
    askedthrust[4] = data->Fpitch;
    askedthrust[5] = data->Fyaw;

    // 分配推力
    ThrustAllocate(askedthrust, motorthrust);
    
    // 归一化/等比例缩放: 找到绝对值最大的一个电机输出
    float max_output = 0.0f;
    for (int i = 0; i < 6; i++) {
        float abs_val = motorthrust[i] > 0 ? motorthrust[i] : -motorthrust[i];
        if (abs_val > max_output) {
            max_output = abs_val;
        }
    }

    // 如果最大输出超过1.0，则将所有输出等比例缩小
    if (max_output > 1.0f) {
        for (int i = 0; i < 6; i++) {
            motorthrust[i] = motorthrust[i] / max_output;
        }
    }
     for (int i = 0; i < 6; i++) {
            motorthrust[i] *= 2000;
        }
    // 更新电机PWM输出
    MotorPwmRefresh(motorthrust);
    
    // 舵机控制(如果有对应API可以在此调用)
    float servo_angle = data->Angle_servo;
}

void Command_Callback_ThrustCurve(uint8_t rw_flag, uint8_t index, ThrustCurveData_t *data) {
    // 接收到推力曲线配置时更新 thrustcurve
    if  (rw_flag == 1) { // 1代表写
        if (index < 6) {
            memcpy(&(thrustcurve[index].pwm[0]), data->pwm, sizeof(float) * 4);
            memcpy(&(thrustcurve[index].thrust[0]), data->thrust, sizeof(float) * 4);
        }
    }
    else if (rw_flag == 0)
		{ // 0代表读
        if (index < 6) {
            memcpy(data->pwm, &(thrustcurve[index].pwm[0]), sizeof(float) * 4);
            memcpy(data->thrust, &(thrustcurve[index].thrust[0]), sizeof(float) * 4);            
            // 下发打包返回数据
            uint8_t tx_buf[39];
            tx_buf[0] = 0xFA;
            tx_buf[1] = 0xAF;
            tx_buf[2] = CMD_THRUST_CURVE;
            tx_buf[3] = 0x00; // rw_flag为0，表示读返回
            tx_buf[4] = index;
            memcpy(&tx_buf[5], data, sizeof(ThrustCurveData_t));
            tx_buf[37] = 0xFB;
            tx_buf[38] = 0xBF;
            
            HAL_UART_Transmit_IT(&huart1, tx_buf, 39);        }
    }
}

void Command_Callback_ThrustMatrix(uint8_t rw_flag, ThrustMatrixData_t *data) {
    if (rw_flag == 1) { // 1代表写
        thrust_params.A_1 = data->A_1;
        thrust_params.A_2 = data->A_2;
        thrust_params.B   = data->B;
        thrust_params.C   = data->C;
        thrust_params._2b = data->_2b;
    }
}

void Command_ProcessPacket(uint8_t *buf) {
    uint8_t cmd = buf[2];

    switch (cmd) {
        case CMD_NORMAL_MODE: {
            NormalModeData_t data;
            memcpy(&data, &buf[3], sizeof(NormalModeData_t));
            Command_Callback_NormalMode(&data);
            break;
        }
        case CMD_THRUST_CURVE: {
            ThrustCurveData_t data;
            uint8_t rw_flag_CURVE = buf[3];
            uint8_t index = buf[4];
            memcpy(&data, &buf[5], sizeof(ThrustCurveData_t));
            Command_Callback_ThrustCurve(rw_flag_CURVE, index, &data);
            break;
        }
        case CMD_THRUST_MATRIX: {
            ThrustMatrixData_t data;
            uint8_t rw_flag = buf[3];
            memcpy(&data, &buf[4], sizeof(ThrustMatrixData_t));
            Command_Callback_ThrustMatrix(rw_flag, &data);
            break;
        }
        default:
            break;
    }
}
 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        // 查找帧头 FA AF
        if (uart1rec.cnt > 0 && uart1rec.buf[uart1rec.cnt - 1] == 0xfa && uart1rec.buf[uart1rec.cnt] == 0xaf)
        {
            uart1rec.cnt    = 1;
            uart1rec.buf[0] = 0xfa;
            uart1rec.buf[1] = 0xaf;
        }
        
        // 查找帧尾 FB BF
        if (uart1rec.cnt > 0 && uart1rec.buf[uart1rec.cnt - 1] == 0xfb && uart1rec.buf[uart1rec.cnt] == 0xbf)
        {
            Command_ProcessPacket(uart1rec.buf);
            uart1rec.cnt = 201; // 使缓冲计数归零
        }
        
        if (uart1rec.cnt > 200) // 防止缓冲溢出
            uart1rec.cnt = 0;
        else
            uart1rec.cnt++;

        HAL_UART_Receive_IT(&huart1, uart1rec.buf + uart1rec.cnt, 1);
    }
}
