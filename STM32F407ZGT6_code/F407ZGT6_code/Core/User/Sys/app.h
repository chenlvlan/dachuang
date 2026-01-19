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
#include <stdbool.h>

extern bool doMotionCtrlCycle;
extern UART_HandleTypeDef huart4;
extern motorDataRead_t JMDataRead[4];
extern CanRxMessage_t CanRxMessage;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim3;

//speed in Rad/s, torque in N.m, timeout in ms
void returnToOrigin(float speed, float torque, uint32_t timeout);
void appLoop();
void appSetup();
void motionCtrlCycle();

void refreshAll(uint8_t id);


#endif /* USER_SYS_APP_H_ */
