/*
 * wheel_motor.h
 *
 *  Created on: Jan 27, 2026
 *      Author: yufei
 */

#ifndef USER_WHEEL_MOTOR_WHEEL_MOTOR_H_
#define USER_WHEEL_MOTOR_WHEEL_MOTOR_H_

#include "main.h"
#include "dma.h"
#include "usart.h"
#include "string.h"

//extern UART_HandleTypeDef huart4;

typedef struct {
	uint8_t mode;
	float m0target;
	float m1target;
	float m0velocity;
	float m0torque;
	float m1velocity;
	float m1torque;
} wheelMotorData_t;

enum wheelMotorMode {
	WM_Disable = 0, WM_Velocity, WM_Torque, WM_Restart = 10
};

void WM_CommInit();
void WM_Send(wheelMotorData_t mot_cmd);
void WM_SendDisable();
void WM_SendVelocity(float m0, float m1);
void WM_SendTorque(float m0, float m1);
void WM_SendRestart();
void WM_Receive(float *m0velocity, float *m0torque, float *m1velocity,
		float *m1torque);
void uart4DMA(UART_HandleTypeDef *huart);

#endif /* USER_WHEEL_MOTOR_WHEEL_MOTOR_H_ */
