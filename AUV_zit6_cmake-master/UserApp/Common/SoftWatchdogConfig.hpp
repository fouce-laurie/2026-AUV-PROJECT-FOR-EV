#ifndef __SOFT_WATCHDOG_CONFIG_HPP
#define __SOFT_WATCHDOG_CONFIG_HPP

#include <stdint.h>

/**
 * @struct SoftWatchdogConfig
 * @brief 软件看门狗配置结构体 (Auto-generated from config.json)
 */
struct SoftWatchdogConfig {
    uint32_t timeout_ms;
    bool check_microros;
    bool check_ins;
    bool check_depth;
};

static const SoftWatchdogConfig DEFAULT_WATCHDOG_CONFIG = {
    .timeout_ms = 3000,
    .check_microros = true,
    .check_ins = false,
    .check_depth = true
};

#endif
