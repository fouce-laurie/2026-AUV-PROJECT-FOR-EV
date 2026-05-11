#ifndef __GLOBAL_CONTEXT_HPP
#define __GLOBAL_CONTEXT_HPP

#include "CommonConfig.hpp"
#include "INS_Driver.hpp"
#include "MotionController_Driver.hpp"
#include "ChassisManager.hpp"
#include "MS5837_Class.h"
#include "stm32h7xx_hal.h"

// --- 硬件句柄 extern ---
extern "C" {
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern IWDG_HandleTypeDef hiwdg1;
extern I2C_HandleTypeDef hi2c1;
}

// --- DMA 缓冲区 extern ---
extern uint8_t ins_rx_buffer[512];
extern auv::device::MotionController_Driver::ThrustPacket motor_tx_packet;

// --- 全局驱动实例 ---
namespace auv {
    namespace device {
        extern INS_Driver ins_driver;
        extern MotionController_Driver motor_driver;
        extern MS5837 depth_sensor;
    }
    
    namespace control {
        extern ChassisManager chassis;
    }
}

// --- 共享状态变量 ---
extern float target_p[4];
extern float last_output_forces[4];
extern float last_dt_ms;
extern uint32_t last_received_seq;
extern float current_depth_z;

// --- 安全 ARM 状态机变量 ---
extern bool is_system_armed;
extern uint32_t arm_heartbeat_count;
extern uint32_t last_arm_heartbeat_ms;
extern uint32_t last_arm_heartbeat_data;
extern uint32_t arm_start_ms;
extern auv::NavState shared_nav_state;

// --- 公共辅助函数 ---
namespace auv {
namespace shared {
    bool isNavigationValid(const NavState &nav);
    NavState snapshotNavState();
}
}

#endif
