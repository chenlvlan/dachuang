/*
 * wheel_motor.h
 *
 *  Created on: Jan 27, 2026
 *      Author: yufei
 */

#ifndef USER_WHEEL_MOTOR_WHEEL_MOTOR_H_
#define USER_WHEEL_MOTOR_WHEEL_MOTOR_H_

#include "main.h"
#include "string.h"

extern UART_HandleTypeDef huart4;

typedef struct {
	uint8_t mode;
	float m0target;
	float m1target;
} motorCommand;

void WM_Send(motorCommand mot_cmd);
void WM_Restart();


#endif /* USER_WHEEL_MOTOR_WHEEL_MOTOR_H_ */
