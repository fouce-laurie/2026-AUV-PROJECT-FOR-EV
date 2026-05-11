#include "GlobalContext.hpp"
#include "FreeRTOS.h"
#include "task.h"

// --- DMA 缓冲区定义 (位于 RAM_D2) ---
__attribute__((section(".dma_buffer"))) uint8_t ins_rx_buffer[512];
__attribute__((section(".dma_buffer"))) auv::device::MotionController_Driver::ThrustPacket motor_tx_packet;

// --- 驱动实例定义 ---
namespace auv {
namespace device {
    INS_Driver ins_driver(&huart7, &huart7, ins_rx_buffer, 512);
    MotionController_Driver motor_driver(&huart6, &motor_tx_packet);
    MS5837 depth_sensor(&hi2c1);
}

namespace control {
    ChassisManager chassis;
}
}

// --- 共享变量定义 ---
float target_p[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float last_output_forces[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float last_dt_ms = 0.0f;
uint32_t last_received_seq = 0;
float current_depth_z = 0.0f;

bool is_system_armed = false;
uint32_t arm_heartbeat_count = 0;
uint32_t last_arm_heartbeat_ms = 0;
uint32_t last_arm_heartbeat_data = 0;
uint32_t arm_start_ms = 0;
auv::NavState shared_nav_state{};

namespace auv {
namespace shared {

bool isNavigationValid(const auv::NavState &nav) {
    // 只要惯导进入 03 或 04 模式且数据新鲜，就认为 Ready
    return ((nav.imu_state == 3 || nav.imu_state == 4) && auv::device::ins_driver.isDataFresh());
}

auv::NavState snapshotNavState() {
    auv::NavState nav;
    taskENTER_CRITICAL();
    nav = shared_nav_state;
    taskEXIT_CRITICAL();
    return nav;
}

} // namespace shared
} // namespace auv
