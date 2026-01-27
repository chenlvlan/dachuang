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

#include "cli.h"
#include "compute.h"
#include "../mpu6500/mpu6500.h"
#include "../joint_motor/joint_motor.h"
#include "../wheel_motor/wheel_motor.h"
//#include "midware.h.txt"

//extern bool doMotionCtrlCycle;
//extern UART_HandleTypeDef huart4;
extern motorDataRead_t JMDataRead[4];
//extern CanRxMessage_t CanRxMessage;
//extern CAN_HandleTypeDef hcan1;
//extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;

void appLoop();
void appSetup();
void motionCtrlCycle();
void HVHP(bool isEN);

//void cli_init(void);
//void cli_handle_command(char *cmd);

void control_init(void);
void control_loop(float pitch_raw);

#endif /* USER_SYS_APP_H_ */
