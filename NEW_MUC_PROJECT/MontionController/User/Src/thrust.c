#include "thrust.h"

/*
 * 函数名: ThrustAllocate
 * 描述  : 推力分配
 * 输入  : askedthrust 0~5 对应 x y z rx ry rz
 *         motorthrust 电机被分配的推力 0~5 对应电机 0~5
 * 输出  : /
 * 备注  : /
 */

extern ThrustParams_t thrust_params;
// 实例化并赋初值
ThrustParams_t thrust_params = {
    .A_1 = 0.290f,
    .A_2 = 0.290f,
    .B   = 0.949f,
    .C   = 0.436f,
    ._2b  = 0.264f
};
void ThrustAllocate(float *askedthrust, float *motorthrust)
{
    motorthrust[0] = -thrust_params.A_2 * askedthrust[0] / thrust_params.B + thrust_params.C * askedthrust[1] + askedthrust[5] / thrust_params.B;
    motorthrust[1] = thrust_params.A_2 * askedthrust[0] / thrust_params.B + thrust_params.C * askedthrust[1] - askedthrust[5] / thrust_params.B;
    motorthrust[2] = -0.5 * askedthrust[2] + askedthrust[4] / thrust_params._2b;
    motorthrust[3] = -0.5 * askedthrust[2] - askedthrust[4] / thrust_params._2b;
    motorthrust[4] = -thrust_params.A_1 * askedthrust[0] / thrust_params.B - thrust_params.C * askedthrust[1] - askedthrust[5] / thrust_params.B;
    motorthrust[5] = thrust_params.A_1 * askedthrust[0] / thrust_params.B - thrust_params.C * askedthrust[1] + askedthrust[5] / thrust_params.B;
}

