/*
 * app.h
 *
 *  Created on: Dec 11, 2025
 *      Author: yufei
 */

#ifndef USER_SYS_APP_H_
#define USER_SYS_APP_H_

#include "main.h"
#include "midware.h"
#include "misc.h"
#include "../MPU6500/driver_mpu6500.h"
#include "../MPU6500/driver_mpu6500_interface.h"
#include "../MPU6500/driver_mpu6500_dmp.h"
#include <stdbool.h>

#define MPU6500_INT_PIN GPIO_PIN_3

extern bool doMotionCtrlCycle;
extern UART_HandleTypeDef huart4;
extern motorDataRead_t JMDataRead[4];
extern CanRxMessage_t CanRxMessage;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;

typedef struct {
	float ax, ay, az;   // 加速度原始值，单位 g
	float gx, gy, gz;   // 陀螺仪原始值，单位 deg/s
	float dt;           // 采样周期，单位秒
} imu_raw_t;

typedef struct {
	//float q0, q1, q2, q3; // 四元数
	float roll, pitch, yaw; // 欧拉角，单位度
} imu_attitude_t;

//speed in Rad/s, torque in N.m, timeout in ms
void returnToOrigin(float speed, float torque, uint32_t timeout);
void appLoop();
void appSetup();
void motionCtrlCycle();

void MPU6500_SPIInit();
void dmp_print_once();
//void mpu_receive_callback(uint8_t type);
//void mpu_tap_callback(uint8_t count, uint8_t dir);
//void mpu_orient_callback(uint8_t orientation);
void cli_init(void);
void cli_handle_command(char *cmd);
void refreshAll(uint8_t id);
void simple_attitude_fusion_rpy(const imu_raw_t *raw, imu_attitude_t *att, float alpha);

#endif /* USER_SYS_APP_H_ */
