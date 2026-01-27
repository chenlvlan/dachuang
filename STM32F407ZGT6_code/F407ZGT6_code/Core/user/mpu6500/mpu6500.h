/*
 * mpu6500.h
 *
 *  Created on: Jan 27, 2026
 *      Author: yufei
 */

#ifndef USER_MPU6500_MPU6500_H_
#define USER_MPU6500_MPU6500_H_

#include <stdio.h>
#include <math.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#define MPU6500_INT_PIN GPIO_PIN_3

int mpu6500_SPIInit();
void mpu6500_DMPGet(float* quat_nom);
void mpu_data_ready();
int mpu6500_isReady();

#endif /* USER_MPU6500_MPU6500_H_ */
