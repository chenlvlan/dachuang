/*
 * app.h
 *
 *  Created on: Dec 11, 2025
 *      Author: yufei
 */

#ifndef USER_SYS_APP_H_
#define USER_SYS_APP_H_

#include "main.h"
#include <stdbool.h>

#include "../mpu6500/mpu6500.h"
#include "cli.h"
#include "compute.h"
#include "../joint_motor/joint_motor.h"
#include "../wheel_motor/wheel_motor.h"
//#include "midware.h.txt"

extern bool doMotionCtrlCycle;
//extern UART_HandleTypeDef huart4;
extern motorDataRead_t JMDataRead[4];
extern CanRxMessage_t CanRxMessage;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;

//speed in Rad/s, torque in N.m, timeout in ms
void returnToOrigin(float speed, float torque, uint32_t timeout);
void appLoop();
void appSetup();
void motionCtrlCycle();
void HVHP(bool isEN);


void cli_init(void);
void cli_handle_command(char *cmd);
void refreshAll(uint8_t id);

void control_init(void);
void control_loop(float pitch_raw);
void quat2euler(float w, float x, float y, float z, float *roll, float *pitch,
		float *yaw);

#endif /* USER_SYS_APP_H_ */
