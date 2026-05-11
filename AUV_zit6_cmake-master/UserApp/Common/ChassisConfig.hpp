#ifndef __CHASSIS_CONFIG_HPP
#define __CHASSIS_CONFIG_HPP

#include <stdint.h>

/**
 * @struct ChassisConfig
 * @brief 底盘默认参数配置 (Auto-generated from config.json)
 */
struct ChassisProfile {
    float default_max_v;
    float default_max_a;
};

struct PIDConfig {
    float kp;
    float ki;
    float kd;
    float i_limit;
    float output_limit;
    float dt;
};

struct ChassisConfig {
    ChassisProfile profile;
    PIDConfig pos_pid;
    PIDConfig vel_pid;
};

static const ChassisConfig DEFAULT_CHASSIS_CONFIG = {
    .profile = { .default_max_v = 0.5, .default_max_a = 0.2 },
    .pos_pid = { .kp = 0.01, .ki = 0.0, .kd = 0.0, .i_limit = 1.0, .output_limit = 1.0, .dt = 0.01 },
    .vel_pid = { .kp = 0.01, .ki = 0.005, .kd = 0.01, .i_limit = 1.0, .output_limit = 1.0, .dt = 0.01 },
};

#endif
