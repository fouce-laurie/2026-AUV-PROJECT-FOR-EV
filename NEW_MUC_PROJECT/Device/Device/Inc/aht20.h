#ifndef __AHT20_H__
#define __AHT20_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"

#define AHT20_ADDRESS 0x70

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t slave_address;
	  float temperature;
		float humidity;
} AHT20_HandleTypeDef;

/**
 * @brief  Initialize the AHT20 struct
 * @param  dev: Pointer to the AHT20 handle
 * @param  hi2c: Pointer to the I2C handle
 * @param  slave_address: I2C slave address of the AHT20 (default AHT20_ADDRESS)
 */
void AHT20_Init(AHT20_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c, uint8_t slave_address);

/**
 * @brief  Start and initialize the AHT20 device via I2C
 * @param  dev: Pointer to the AHT20 handle
 */
void AHT20_Begin(AHT20_HandleTypeDef *dev);

/**
 * @brief  Read temperature and humidity from the AHT20
 * @param  dev: Pointer to the AHT20 handle
 * @param  Temperature: Pointer to variable to store temperature
 * @param  Humidity: Pointer to variable to store humidity
 */
void AHT20_Read(AHT20_HandleTypeDef *dev, float *Temperature, float *Humidity);

#ifdef __cplusplus
}
#endif

#endif /* __AHT20_H__ */