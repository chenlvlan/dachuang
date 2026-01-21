/*
 * mpu6500_port.h
 *
 *  Created on: Jan 21, 2026
 *      Author: yufei
 */

#ifndef USER_MPU6500_MPU6500_PORT_H_
#define USER_MPU6500_MPU6500_PORT_H_

#include "stm32f4xx_hal.h"
#include <string.h>

void mpu_spi_write(uint8_t reg, const uint8_t *data, uint16_t len);
void mpu_spi_read(uint8_t reg, uint8_t *data, uint16_t len);

extern SPI_HandleTypeDef hspi1;




#endif /* USER_MPU6500_MPU6500_PORT_H_ */
