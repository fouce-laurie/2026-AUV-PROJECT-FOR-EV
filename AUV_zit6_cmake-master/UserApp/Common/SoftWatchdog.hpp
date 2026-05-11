#ifndef __SOFT_WATCHDOG_HPP
#define __SOFT_WATCHDOG_HPP

#include "SoftWatchdogConfig.hpp"
#include <stdint.h>

namespace auv {
namespace device {

/**
 * @class SoftWatchdog
 * @brief 软件看门狗管理类
 */
class SoftWatchdog {
public:
    enum class Component {
        MICROROS = 0,
        INS = 1,
        DEPTH = 2,
        COUNT = 3
    };

    static SoftWatchdog& getInstance() {
        static SoftWatchdog instance;
        return instance;
    }

    /**
     * @brief 初始化看门狗
     * @param config 配置项
     */
    void init(const SoftWatchdogConfig& config = DEFAULT_WATCHDOG_CONFIG);

    /**
     * @brief 喂狗 (针对特定组件)
     * @param component 组件ID
     */
    void feed(Component component);

    /**
     * @brief 检查所有组件是否正常
     * @return true 如果所有启用的组件都在超时时间内喂过狗
     */
    bool check();

private:
    SoftWatchdog() = default;
    SoftWatchdogConfig config_;
    uint32_t last_feed_ms_[(int)Component::COUNT] = {0};
};

} // namespace device
} // namespace auv

#endif
