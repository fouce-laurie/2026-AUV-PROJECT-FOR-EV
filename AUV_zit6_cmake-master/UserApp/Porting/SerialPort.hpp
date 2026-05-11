#ifndef __SERIAL_PORT_HPP
#define __SERIAL_PORT_HPP

#include "usart.h"
#include <stdint.h>
#include <string.h>

namespace auv {

class SerialPort {
public:
    enum class Mode {
        RECEIVE_ONLY,
        TRANSMIT_ONLY,
        DUPLEX
    };

    static constexpr uint16_t kMaxRxBufferSize = 512;

    SerialPort(UART_HandleTypeDef* huart, uint8_t* ext_rx_buf, uint16_t rx_buf_size)
        : huart_(huart), rx_buffer_ptr_(ext_rx_buf), rx_buf_size_(rx_buf_size) {}

    SerialPort(UART_HandleTypeDef* huart, uint16_t rx_buf_size = kMaxRxBufferSize)
        : huart_(huart), rx_buffer_ptr_(rx_buffer_internal_), rx_buf_size_(rx_buf_size > kMaxRxBufferSize ? kMaxRxBufferSize : rx_buf_size) {}

    ~SerialPort() = default;

    /**
     * @brief 启动 DMA 循环接收
     */
    bool startReceive() {
        // 强行清除之前由于上电时序差可能产生的 ORE (Overrun) 等错误标志
        __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
        return HAL_UART_Receive_DMA(huart_, rx_buffer_ptr_, rx_buf_size_) == HAL_OK;
    }

    /**
     * @brief 发送数据 (DMA 方式)
     */
    bool transmit(const uint8_t* data, uint16_t len) {
        return HAL_UART_Transmit_DMA(huart_, const_cast<uint8_t*>(data), len) == HAL_OK;
    }

    /**
     * @brief 从循环缓冲区读取新数据
     * @return 读取到的字节数
     */
    uint16_t read(uint8_t* out_buf, uint16_t max_len) {
        uint16_t current_pos = rx_buf_size_ - __HAL_DMA_GET_COUNTER(huart_->hdmarx);
        uint16_t bytes_to_read = 0;

        if (current_pos != last_rx_pos_) {
            if (current_pos > last_rx_pos_) {
                bytes_to_read = current_pos - last_rx_pos_;
                if (bytes_to_read > max_len) bytes_to_read = max_len;
                memcpy(out_buf, rx_buffer_ptr_ + last_rx_pos_, bytes_to_read);
            } else {
                // 发生回绕
                uint16_t first_part = rx_buf_size_ - last_rx_pos_;
                bytes_to_read = first_part + current_pos;
                if (bytes_to_read > max_len) bytes_to_read = max_len;

                if (bytes_to_read <= first_part) {
                    memcpy(out_buf, rx_buffer_ptr_ + last_rx_pos_, bytes_to_read);
                } else {
                    memcpy(out_buf, rx_buffer_ptr_ + last_rx_pos_, first_part);
                    memcpy(out_buf + first_part, rx_buffer_ptr_, bytes_to_read - first_part);
                }
            }
            last_rx_pos_ = current_pos;
        }
        return bytes_to_read;
    }

private:
    UART_HandleTypeDef* huart_;
    uint8_t* rx_buffer_ptr_;
    alignas(32) uint8_t rx_buffer_internal_[kMaxRxBufferSize] = {0};
    uint16_t rx_buf_size_;
    uint16_t last_rx_pos_ = 0;
};

} // namespace auv

#endif
