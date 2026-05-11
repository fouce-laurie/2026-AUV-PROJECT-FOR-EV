#ifndef MOTION_CONTROLLER_DRIVER_HPP
#define MOTION_CONTROLLER_DRIVER_HPP

#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include <string.h>

namespace auv {
namespace device {

/**
 * @class MotionController_Driver
 * @brief 动力控制板驱动类
 *
 * 职责：
 * 1. 封装与下位机单片机的通信协议（0xFA 0xAF ... 0xFB 0xBF）。
 * 2. 支持推力下发、推力曲线配置、舵机控制及灯光控制。
 */
class MotionController_Driver {
public:
  // 0x01: 推力下发数据包
  struct __attribute__((packed)) ThrustPacket {
    uint8_t head[2]; // FA AF
    uint8_t id;      // 0x01
    float Fx, Fy, Fz;
    float Fyaw, Fpitch, Froll;
    uint8_t tail[2]; // FB BF
  };

  // 0x07: 推力曲线配置包
  struct __attribute__((packed)) CurvePacket {
    uint8_t head[2]; // FA AF
    uint8_t id;      // 0x07
    uint8_t mode;    // 0: Read, 1: Write
    uint8_t index;   // Motor Index
    float pwm[4];    // 4 PWM points
    float thrust[4]; // 4 Thrust points
    uint8_t tail[2]; // FB BF
  };

  // 0x02: 舵机控制包
  struct __attribute__((packed)) ServoPacket {
    uint8_t head[2]; // FA AF
    uint8_t id;      // 0x02
    float angle;
    uint8_t tail[2]; // FB BF
  };

  // 0x03: 灯光控制包
  struct __attribute__((packed)) LightPacket {
    uint8_t head[2]; // FA AF
    uint8_t id;      // 0x03
    uint8_t state;   // R/Y/B state
    uint8_t tail[2]; // FB BF
  };

  MotionController_Driver(UART_HandleTypeDef *huart, ThrustPacket *ext_pkt)
      : huart_(huart), thrust_pkt_ptr_(ext_pkt) {
    initPacket(thrust_pkt_ptr_, 0x01);
  }

  MotionController_Driver(UART_HandleTypeDef *huart)
      : huart_(huart), thrust_pkt_ptr_(&thrust_pkt_internal_) {
    initPacket(thrust_pkt_ptr_, 0x01);
  }

  /**
   * @brief 下发推力指令 (ID: 0x01)
   * 使用 DMA 发送，不阻塞控制流
   */
  void publishThrust(float fx, float fy, float fz, float fyaw, float fp = 0,
                     float fr = 0) {
    // 更新推力值（线程安全）
    taskENTER_CRITICAL();
    thrust_pkt_ptr_->Fx = fx;
    thrust_pkt_ptr_->Fy = fy;
    thrust_pkt_ptr_->Fz = fz;
    thrust_pkt_ptr_->Fyaw = fyaw;
    thrust_pkt_ptr_->Fpitch = fp;
    thrust_pkt_ptr_->Froll = fr;
    taskEXIT_CRITICAL();
    
    // 通过 DMA 发送推力数据包副本
    sendThrustPacketDMA();
  }

  /**
   * @brief 配置推力曲线 (ID: 0x07)
   */
  void setThrustCurve(uint8_t mode, uint8_t index, const float pwm[4],
                      const float thrust[4]) {
    static CurvePacket pkt;
    initPacket(&pkt, 0x07);
    pkt.mode = mode;
    pkt.index = index;
    memcpy(pkt.pwm, pwm, 16);
    memcpy(pkt.thrust, thrust, 16);
    transmitDMA((uint8_t *)&pkt, sizeof(CurvePacket));
  }

  /**
   * @brief 舵机角度控制 (ID: 0x02)
   */
  void setServoAngle(float angle) {
    static ServoPacket pkt;
    initPacket(&pkt, 0x02);
    pkt.angle = angle;
    transmitDMA((uint8_t *)&pkt, sizeof(ServoPacket));
  }

  /**
   * @brief 灯光状态控制 (ID: 0x03)
   */
  void setLightState(uint8_t state) {
    static LightPacket pkt;
    initPacket(&pkt, 0x03);
    pkt.state = state;
    transmitDMA((uint8_t *)&pkt, sizeof(LightPacket));
  }

private:
  template <typename T> void initPacket(T *pkt, uint8_t id) {
    pkt->head[0] = 0xFA;
    pkt->head[1] = 0xAF;
    pkt->id = id;
    pkt->tail[0] = 0xFB;
    pkt->tail[1] = 0xBF;
  }

  /**
   * @brief 创建推力数据包副本并通过 DMA 发送
   */
  void sendThrustPacketDMA() {
    // 创建用于 DMA 发送的本地副本，避免原数据被修改
    static ThrustPacket dma_pkt __attribute__((aligned(32)));
    
    taskENTER_CRITICAL();
    memcpy(&dma_pkt, thrust_pkt_ptr_, sizeof(ThrustPacket));
    taskEXIT_CRITICAL();
    
    transmitDMA((uint8_t *)&dma_pkt, sizeof(ThrustPacket));
  }

  void transmitDMA(uint8_t *data, uint16_t size) {
    // 使用 FreeRTOS 关键段保护，防止多线程竞争 UART 句柄
    taskENTER_CRITICAL();
    if (huart_->gState == HAL_UART_STATE_READY) {
      HAL_UART_Transmit_DMA(huart_, data, size);
    }
    taskEXIT_CRITICAL();
  }

  UART_HandleTypeDef *huart_;
  ThrustPacket *thrust_pkt_ptr_;
  ThrustPacket thrust_pkt_internal_ __attribute__((aligned(32)));
};

} // namespace device
} // namespace auv

#endif // MOTION_CONTROLLER_DRIVER_HPP
