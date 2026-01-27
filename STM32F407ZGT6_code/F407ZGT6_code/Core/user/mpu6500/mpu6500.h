/*
 * mpu6500.h
 *
 *  Created on: Jan 27, 2026
 *      Author: yufei
 */

#ifndef USER_MPU6500_MPU6500_H_
#define USER_MPU6500_MPU6500_H_

#include "../mpu6500/inv_mpu.h"
#include "../mpu6500/inv_mpu_dmp_motion_driver.h"

#define MPU6500_INT_PIN GPIO_PIN_3

int MPU6500_SPIInit();
void dmp_print_once();
void mpu_data_ready(void);



#endif /* USER_MPU6500_MPU6500_H_ */
