#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>

/* 指令码定义 */
#define CMD_NORMAL_MODE     0x01
#define CMD_THRUST_CURVE    0x07
#define CMD_THRUST_MATRIX   0x10

/* 正常工作模式 (33 bytes: FA AF 01 Fx Fy Fz Fyaw Fpitch Froll Angle FB BF) */
typedef struct __attribute__((packed)) {
    float Fx;
    float Fy;
    float Fz;
    float Fyaw;
    float Fpitch;
    float Froll;
    float Angle_servo;
} NormalModeData_t;

typedef struct {
    uint8_t buf[256];
    uint16_t cnt;
} RecBuf;

/* 推力曲线 (4+32+2 = 38 bytes) */
typedef struct __attribute__((packed)) {
    float pwm[4];
    float thrust[4];
} ThrustCurveData_t;

/* 推力矩阵 */
typedef struct __attribute__((packed)) {
    float A_1;
    float A_2;
    float B;
    float C;
    float _2b;
} ThrustMatrixData_t;
extern RecBuf uart1rec;
void Command_ParseChar(uint8_t ch);
void Command_ProcessPacket(uint8_t *buf);

/* 弱定义回调函数，用户可在其他文件实现这些逻辑 */
void Command_Callback_NormalMode(NormalModeData_t *data);
void Command_Callback_ThrustCurve(uint8_t rw_flag, uint8_t index, ThrustCurveData_t *data);
void Command_Callback_ThrustMatrix(uint8_t rw_flag, ThrustMatrixData_t *data);

void CommandInit(void);

#endif /* COMMAND_H */
