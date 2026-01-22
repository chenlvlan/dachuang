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
#include "../MPU6500/inv_mpu.h"
#include "../MPU6500/inv_mpu_dmp_motion_driver.h"
#include <stdbool.h>
#include "cli.h"

#define MPU6500_INT_PIN GPIO_PIN_3

extern bool doMotionCtrlCycle;
extern UART_HandleTypeDef huart4;
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

int MPU6500_SPIInit();
void dmp_print_once();
void mpu_data_ready(void);

void cli_init(void);
void cli_handle_command(char *cmd);
void refreshAll(uint8_t id);

#endif /* USER_SYS_APP_H_ */
