#include "thrust.h"

/*
 * 函数名: ThrustAllocate
 * 描述  : 推力分配
 * 输入  : askedthrust 0~5 对应 x y z rx ry rz
 *         motorthrust 电机被分配的推力 0~5 对应电机 0~5
 * 输出  : /
 * 备注  : /
 */
void ThrustAllocate(float *askedthrust, float *motorthrust)
{
    motorthrust[0] = -__A_2 * askedthrust[0] / __B + __C * askedthrust[1] + askedthrust[5] / __B;
    motorthrust[1] = __A_2 * askedthrust[0] / __B + __C * askedthrust[1] - askedthrust[5] / __B;
    motorthrust[2] = -0.5 * askedthrust[2] + askedthrust[4] / __2b;
    motorthrust[3] = -0.5 * askedthrust[2] - askedthrust[4] / __2b;
    motorthrust[4] = -__A_1 * askedthrust[0] / __B - __C * askedthrust[1] - askedthrust[5] / __B;
    motorthrust[5] = __A_1 * askedthrust[0] / __B - __C * askedthrust[1] + askedthrust[5] / __B;
}

