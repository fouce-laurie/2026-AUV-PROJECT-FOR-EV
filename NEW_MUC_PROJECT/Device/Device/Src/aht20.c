#include "aht20.h"

void AHT20_Init(AHT20_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c, uint8_t slave_address) {
    dev->hi2c = hi2c;
    dev->slave_address = slave_address;
}

void AHT20_Begin(AHT20_HandleTypeDef *dev) {
    uint8_t readBuffer;
    HAL_Delay(40);
    HAL_I2C_Master_Receive(dev->hi2c, dev->slave_address, &readBuffer, 1, HAL_MAX_DELAY);
    
    if ((readBuffer & 0x08) == 0x00) {
        uint8_t sendBuffer[3] = {0xBE, 0x08, 0x00};
        HAL_I2C_Master_Transmit(dev->hi2c, dev->slave_address, sendBuffer, sizeof(sendBuffer), HAL_MAX_DELAY);
    }
}

void AHT20_Read(AHT20_HandleTypeDef *dev, float *Temperature, float *Humidity) {
    uint8_t sendBuffer[3] = {0xAC, 0x33, 0x00};
    uint8_t readBuffer[6] = {0};

    HAL_I2C_Master_Transmit(dev->hi2c, dev->slave_address, sendBuffer, sizeof(sendBuffer), HAL_MAX_DELAY);
    HAL_Delay(75);
    HAL_I2C_Master_Receive(dev->hi2c, dev->slave_address, readBuffer, sizeof(readBuffer), HAL_MAX_DELAY);

    if ((readBuffer[0] & 0x80) == 0x00) {
        uint32_t data = 0;
        
        data = ((uint32_t)readBuffer[3] >> 4) + ((uint32_t)readBuffer[2] << 4) + ((uint32_t)readBuffer[1] << 12);
        if (Humidity != NULL) {
            *Humidity = data * 100.0f / (1 << 20);
        }

        data = (((uint32_t)readBuffer[3] & 0x0F) << 16) + ((uint32_t)readBuffer[4] << 8) + (uint32_t)readBuffer[5];
        if (Temperature != NULL) {
            *Temperature = data * 200.0f / (1 << 20) - 50.0f;
        }
    }
}