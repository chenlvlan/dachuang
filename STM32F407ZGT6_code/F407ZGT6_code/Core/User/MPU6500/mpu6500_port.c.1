/*
 * mpu6500_port.c
 *
 *  Created on: Jan 21, 2026
 *      Author: yufei
 */

#include "mpu6500_port.h"

#define MPU_CS_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define MPU_CS_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

void mpu_spi_write(uint8_t reg, const uint8_t *data, uint16_t len) {
	MPU_CS_LOW();

	// 写寄存器地址（MSB = 0）
	uint8_t reg_addr = reg & 0x7F;
	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);

	// 连续写数据
	if (len > 0) {
		HAL_SPI_Transmit(&hspi1, (uint8_t*) data, len, HAL_MAX_DELAY);
	}

	MPU_CS_HIGH();
}

void mpu_spi_read(uint8_t reg, uint8_t *data, uint16_t len) {
	uint8_t reg_addr = reg | 0x80;   // 读：MSB = 1
	uint8_t dummy = 0xFF;

	MPU_CS_LOW();

	// 发送寄存器地址
	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);

	// 关键：通过“发送 dummy”来产生 SPI clock，同时读数据
	for (uint16_t i = 0; i < len; i++) {
		HAL_SPI_TransmitReceive(&hspi1, &dummy, &data[i], 1, HAL_MAX_DELAY);
	}

	MPU_CS_HIGH();
}
