/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_mpu6500_interface_template.c
 * @brief     driver mpu6500 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2024-07-30
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2024/07/30  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_mpu6500_interface.h"
#include "stm32f4xx_hal.h"   // 根据你的芯片族，F1/F4/H7
#include <stdio.h>
#include <stdarg.h>

#define MPU6500_CS_GPIO_Port   GPIOA
#define MPU6500_CS_Pin         GPIO_PIN_4
#define MPU6500_CS_LOW()   HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_RESET)
#define MPU6500_CS_HIGH()  HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET)

extern SPI_HandleTypeDef hspi1;

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t mpu6500_interface_iic_init(void) {
	return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t mpu6500_interface_iic_deinit(void) {
	return 0;
}

/**
 * @brief      interface iic bus read
 * @param[in]  addr iic device write address
 * @param[in]  reg iic register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t mpu6500_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf,
		uint16_t len) {
	return 0;
}

/**
 * @brief     interface iic bus write
 * @param[in] addr iic device write address
 * @param[in] reg iic register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t mpu6500_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf,
		uint16_t len) {
	return 0;
}

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t mpu6500_interface_spi_init(void) {
	//已经初始化了，这里不用重复了
	MPU6500_CS_HIGH();   // 确保空闲时 CS 为高
	return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t mpu6500_interface_spi_deinit(void) {
	return 0;
}

/**
 * @brief      interface spi bus read
 * @param[in]  reg register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t mpu6500_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len) {
	/*
	 uint8_t addr = reg | 0x80;

	 MPU6500_CS_LOW();

	 if (HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY) != HAL_OK) {
	 MPU6500_CS_HIGH();
	 return 1;
	 }

	 if (HAL_SPI_Receive(&hspi1, buf, len, HAL_MAX_DELAY) != HAL_OK) {
	 MPU6500_CS_HIGH();
	 return 1;
	 }

	 MPU6500_CS_HIGH();
	 return 0;
	 */

	uint8_t tx[len + 1];
	uint8_t rx[len + 1];

	tx[0] = reg | 0x80;       // 读命令
	memset(tx + 1, 0xFF, len); // dummy bytes

	MPU6500_CS_LOW();
	if (HAL_SPI_TransmitReceive(&hspi1, tx, rx, len + 1, HAL_MAX_DELAY)
			!= HAL_OK) {
		MPU6500_CS_HIGH();
		printf("SPI read failed at reg 0x%02X\n", reg);
		return 1;
	}
	MPU6500_CS_HIGH();

	memcpy(buf, rx + 1, len);
	return 0;

}

/**
 * @brief     interface spi bus write
 * @param[in] reg register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t mpu6500_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len) {
	/*
	 uint8_t addr = reg & 0x7F;

	 MPU6500_CS_LOW();

	 if (HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY) != HAL_OK) {
	 MPU6500_CS_HIGH();
	 return 1;
	 }

	 if (HAL_SPI_Transmit(&hspi1, buf, len, HAL_MAX_DELAY) != HAL_OK) {
	 MPU6500_CS_HIGH();
	 return 1;
	 }

	 MPU6500_CS_HIGH();
	 return 0;
	 */
	uint8_t tx[len + 1];
	tx[0] = reg & 0x7F;
	memcpy(tx + 1, buf, len);

	MPU6500_CS_LOW();
	if (HAL_SPI_Transmit(&hspi1, tx, len + 1, HAL_MAX_DELAY) != HAL_OK) {
		MPU6500_CS_HIGH();
		printf("SPI write failed at reg 0x%02X\n", reg);
		return 1;
	}
	MPU6500_CS_HIGH();

	return 0;

}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void mpu6500_interface_delay_ms(uint32_t ms) {
	HAL_Delay(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void mpu6500_interface_debug_print(const char *const fmt, ...) {
	char buf[128];
	va_list args;

	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	printf("%s", buf);
}

/**
 * @brief     interface receive callback
 * @param[in] type irq type
 * @note      none
 */
void mpu6500_interface_receive_callback(uint8_t type) {
	switch (type) {
	case MPU6500_INTERRUPT_MOTION: {
		mpu6500_interface_debug_print("mpu6500: irq motion.\n");

		break;
	}
	case MPU6500_INTERRUPT_FIFO_OVERFLOW: {
		mpu6500_interface_debug_print("mpu6500: irq fifo overflow.\n");

		break;
	}
	case MPU6500_INTERRUPT_FSYNC_INT: {
		mpu6500_interface_debug_print("mpu6500: irq fsync int.\n");

		break;
	}
	case MPU6500_INTERRUPT_DMP: {
		mpu6500_interface_debug_print("mpu6500: irq dmp\n");

		break;
	}
	case MPU6500_INTERRUPT_DATA_READY: {
		mpu6500_interface_debug_print("mpu6500: irq data ready\n");

		break;
	}
	default: {
		mpu6500_interface_debug_print("mpu6500: irq unknown code.\n");

		break;
	}
	}
}

/**
 * @brief     interface dmp tap callback
 * @param[in] count tap count
 * @param[in] direction tap direction
 * @note      none
 */
void mpu6500_interface_dmp_tap_callback(uint8_t count, uint8_t direction) {
	switch (direction) {
	case MPU6500_DMP_TAP_X_UP: {
		mpu6500_interface_debug_print("mpu6500: tap irq x up with %d.\n",
				count);

		break;
	}
	case MPU6500_DMP_TAP_X_DOWN: {
		mpu6500_interface_debug_print("mpu6500: tap irq x down with %d.\n",
				count);

		break;
	}
	case MPU6500_DMP_TAP_Y_UP: {
		mpu6500_interface_debug_print("mpu6500: tap irq y up with %d.\n",
				count);

		break;
	}
	case MPU6500_DMP_TAP_Y_DOWN: {
		mpu6500_interface_debug_print("mpu6500: tap irq y down with %d.\n",
				count);

		break;
	}
	case MPU6500_DMP_TAP_Z_UP: {
		mpu6500_interface_debug_print("mpu6500: tap irq z up with %d.\n",
				count);

		break;
	}
	case MPU6500_DMP_TAP_Z_DOWN: {
		mpu6500_interface_debug_print("mpu6500: tap irq z down with %d.\n",
				count);

		break;
	}
	default: {
		mpu6500_interface_debug_print("mpu6500: tap irq unknown code.\n");

		break;
	}
	}
}

/**
 * @brief     interface dmp orient callback
 * @param[in] orientation dmp orientation
 * @note      none
 */
void mpu6500_interface_dmp_orient_callback(uint8_t orientation) {
	switch (orientation) {
	case MPU6500_DMP_ORIENT_PORTRAIT: {
		mpu6500_interface_debug_print("mpu6500: orient irq portrait.\n");

		break;
	}
	case MPU6500_DMP_ORIENT_LANDSCAPE: {
		mpu6500_interface_debug_print("mpu6500: orient irq landscape.\n");

		break;
	}
	case MPU6500_DMP_ORIENT_REVERSE_PORTRAIT: {
		mpu6500_interface_debug_print(
				"mpu6500: orient irq reverse portrait.\n");

		break;
	}
	case MPU6500_DMP_ORIENT_REVERSE_LANDSCAPE: {
		mpu6500_interface_debug_print(
				"mpu6500: orient irq reverse landscape.\n");

		break;
	}
	default: {
		mpu6500_interface_debug_print("mpu6500: orient irq unknown code.\n");

		break;
	}
	}
}
