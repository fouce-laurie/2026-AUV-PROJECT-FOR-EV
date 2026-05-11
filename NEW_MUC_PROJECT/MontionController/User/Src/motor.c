#include "motor.h"
#include "tim.h"

ThrustCurve thrustcurve[6];


const int motornum[6] = {0, 1, 2, 3, 4, 5};

float line(float startx, float endx, float starty, float endy, float input)
{
    float k = (endy - starty) / (endx - startx);
    return starty + k * (input - startx);
}

/*
 * 函数名:Thrust2PWM
 * 描述  :推力曲线拟合
 * 输入  :int motornum 电机序号 float Thrust 推力大小
 * 输出  :int PWM数据
 * 备注  :利用C语言取整特性查表
 */
int Thrust2PWM(int num, float thrust)
{
    volatile int mnum;

    volatile float pwm = Motor_Pwm_Median_Duty;

    if (fabs(thrust) <= 1e-4) return Motor_Pwm_Median_Duty;

    mnum = motornum[num];

    if (thrust <= thrustcurve[mnum].thrust[0])
        pwm = Motor_Pwm_Median_Duty - Motor_Pwm_Half_Range_N;
    else if (thrust > thrustcurve[mnum].thrust[0] && thrust <= thrustcurve[mnum].thrust[1])
    {
        pwm = line(thrustcurve[mnum].thrust[0], thrustcurve[mnum].thrust[1], Motor_Pwm_Median_Duty - Motor_Pwm_Half_Range_N, thrustcurve[mnum].pwm[0],
                   thrust);
    }
    else if (thrust > thrustcurve[mnum].thrust[1] && thrust <= 0)
    {
        pwm = line(thrustcurve[mnum].thrust[1], 0, thrustcurve[mnum].pwm[0], thrustcurve[mnum].pwm[1], thrust);
    }
    else if (thrust > 0 && thrust <= thrustcurve[mnum].thrust[2])
    {
        pwm = line(0, thrustcurve[mnum].thrust[2], thrustcurve[mnum].pwm[2], thrustcurve[mnum].pwm[3], thrust);
    }
    else if (thrust > thrustcurve[mnum].thrust[2] && thrust < thrustcurve[mnum].thrust[3])
    {
        pwm = line(thrustcurve[mnum].thrust[2], thrustcurve[mnum].thrust[3], thrustcurve[mnum].pwm[3], Motor_Pwm_Median_Duty + Motor_Pwm_Half_Range_P,
                   thrust);
    }
    else if (thrust >= thrustcurve[mnum].thrust[3])
        pwm = Motor_Pwm_Median_Duty + Motor_Pwm_Half_Range_P;

    return (int)(pwm + 0.5);
}

void ThrustCurveInit(ThrustCurve *thrustcurve)
{
    thrustcurve->pwm[0] = 2582.15f;
    thrustcurve->pwm[1] = 2950.0f;
    thrustcurve->pwm[2] = 3050.0f;
    thrustcurve->pwm[3] = 3338.05f;

    thrustcurve->thrust[0] = -1065.44f;
    thrustcurve->thrust[1] = -364.32f;
    thrustcurve->thrust[2] = 273.06f;
    thrustcurve->thrust[3] = 990.12f;
}


/*
 * 函数名: MotorInit
 * 描述  : 开启PWM输出，初始化电调，做油门校准
 * 输入  : /
 * 输出  : /
 * 备注  : 频率50hz psc=39999 中值3000 min 2400 max 3600
 */
void MotorInit(void)
{
    // 开启PWM输出
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);//中左
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);//中右边
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);//左后
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);//右后
    // __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);
    // __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);
    // __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 0);
    // __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 0);
    // __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 0);
    // __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 0);



    // 电调校准
    // __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, Motor_Pwm_Median_Duty + Motor_Pwm_Half_Range);
    // __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, Motor_Pwm_Median_Duty + Motor_Pwm_Half_Range);
    // __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, Motor_Pwm_Median_Duty + Motor_Pwm_Half_Range);
    // __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, Motor_Pwm_Median_Duty + Motor_Pwm_Half_Range);
    // __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, Motor_Pwm_Median_Duty + Motor_Pwm_Half_Range);
    // __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, Motor_Pwm_Median_Duty + Motor_Pwm_Half_Range);
    // HAL_Delay(1000);
    // HAL_Delay(1000);
    // HAL_Delay(1000);
    // __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, Motor_Pwm_Median_Duty - Motor_Pwm_Half_Range);
    // __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, Motor_Pwm_Median_Duty - Motor_Pwm_Half_Range);
    // __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, Motor_Pwm_Median_Duty - Motor_Pwm_Half_Range);
    // __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, Motor_Pwm_Median_Duty - Motor_Pwm_Half_Range);
    // __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, Motor_Pwm_Median_Duty - Motor_Pwm_Half_Range);
    // __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, Motor_Pwm_Median_Duty - Motor_Pwm_Half_Range);
    // HAL_Delay(1000);
    // HAL_Delay(1000);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, Motor_Pwm_Median_Duty);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, Motor_Pwm_Median_Duty);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, Motor_Pwm_Median_Duty);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, Motor_Pwm_Median_Duty);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, Motor_Pwm_Median_Duty);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, Motor_Pwm_Median_Duty);
    HAL_Delay(30000);
    // HAL_Delay(1000);
    // HAL_Delay(1000);
}

/*
 * 函数名: MotorPwmRefresh
 * 描述  : PWM输出更新
 * 输入  : 电机被分配的推力 0~5 对应电机 0~5
 * 输出  : /
 * 备注  : 输入、输出同时反转时可使推力曲线反转
 */
void MotorPwmRefresh(float *motorthrust)
{
    // 变量初始化
    volatile static int pwmtarget[6] = {Motor_Pwm_Median_Duty, Motor_Pwm_Median_Duty, Motor_Pwm_Median_Duty,
                                        Motor_Pwm_Median_Duty, Motor_Pwm_Median_Duty, Motor_Pwm_Median_Duty};

    volatile static int pwmoutput[6] = {Motor_Pwm_Median_Duty, Motor_Pwm_Median_Duty, Motor_Pwm_Median_Duty,
                                        Motor_Pwm_Median_Duty, Motor_Pwm_Median_Duty, Motor_Pwm_Median_Duty};

                                    
    // motorthrust[0] = -motorthrust[0]; 使输入反转
    // motorthrust[0] = -motorthrust[0];
    // motorthrust[1] = -motorthrust[1];
    // motorthrust[2] = -motorthrust[2];
    // motorthrust[3] = -motorthrust[3];
    // motorthrust[4] = -motorthrust[4];
    // motorthrust[5] = -motorthrust[5];


    // 对输出信号做模糊控制
    for (int i = 0; i < 6; i++)
    {
        // 暂时跳过推力曲线，直接使用推力值作为PWM偏移量进行测试
				pwmoutput[i] = pwmtarget[i] = Thrust2PWM(i, motorthrust[i]);
//        pwmoutput[i] = pwmtarget[i] = Motor_Pwm_Median_Duty + (int)motorthrust[i];

        if (pwmoutput[i] < 2400) pwmoutput[i] = 2400;
        if (pwmoutput[i] > 3600) pwmoutput[i] = 3600;

        // if (abs(pwmoutput[i] - Motor_Pwm_Median_Duty) < 200)
        // {
        //     int delta = abs(pwmoutput[i] - pwmtarget[i]);
        //     float pwm_change_speed;

        //     // 使用一个比例因子来动态计算步进值，例如每差50增加5的步进
        //     pwm_change_speed = 3 + ((float)delta / 50.0) * 5;

        //     // 确保步进值不超过一个最大限定值，比如50
        //     if (pwm_change_speed > 50) 
        //     {
        //         pwm_change_speed = 50;
        //     }

        //     // 调整PWM输出以靠近目标值
        //     if (pwmoutput[i] < pwmtarget[i] - pwm_change_speed) {
        //         pwmoutput[i] += pwm_change_speed;
        //     } else if (pwmoutput[i] > pwmtarget[i] + pwm_change_speed) {
        //         pwmoutput[i] -= pwm_change_speed;
        //     }
        // }        
        // else
        // {
        //     if (pwmoutput[i] < pwmtarget[i] - Motor_Pwm_Change_Speed_HIGH) pwmoutput[i] += Motor_Pwm_Change_Speed_HIGH;
        //     if (pwmoutput[i] > pwmtarget[i] + Motor_Pwm_Change_Speed_HIGH) pwmoutput[i] -= Motor_Pwm_Change_Speed_HIGH;
        // }
    }

    // 信号输出
    // __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 2 * Motor_Pwm_Median_Duty - pwmoutput[0]);
    // e.g. 使输出反转

    int final_pwm[6];
    final_pwm[0] = 2 * Motor_Pwm_Median_Duty - pwmoutput[0];
    final_pwm[1] = pwmoutput[1];
    final_pwm[2] = 2 * Motor_Pwm_Median_Duty - pwmoutput[2];
    final_pwm[3] = pwmoutput[3];
    final_pwm[4] = pwmoutput[4];
    final_pwm[5] = pwmoutput[5];

    for (int i = 0; i < 6; i++) {
        if (final_pwm[i] < 2400) final_pwm[i] = 2400;
        if (final_pwm[i] > 3600) final_pwm[i] = 3600;
    }

    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, final_pwm[0]);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, final_pwm[1]);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, final_pwm[2]);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, final_pwm[3]);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, final_pwm[4]);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, final_pwm[5]);
}
