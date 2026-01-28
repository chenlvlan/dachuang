/*
 * wheel_motor.c
 *
 *  Created on: Jan 27, 2026
 *      Author: yufei
 */

#include "wheel_motor.h"

void WM_Send(motorCommand mot_cmd) {
	uint8_t buf[10] = { 0 };
	buf[0] = mot_cmd.mode;
	memcpy(&buf[1], &mot_cmd.m0target, 4);
	memcpy(&buf[5], &mot_cmd.m1target, 4);
	buf[9] = 0xff;
	HAL_UART_Transmit(&huart4, &buf[0], sizeof(buf), HAL_MAX_DELAY);
}

void WM_Disable() {
	motorCommand a;
	a.mode = 0;
	a.m0target = 0;
	a.m1target = 0;
	WM_Send(a);
}

WM_SendVelocity(float m0, float m1) {
	motorCommand a;
	a.mode = 1;
	a.m0target = m0;
	a.m1target = m1;
	WM_Send(a);
}

void WM_SendTorque(float m0, float m1) {
	motorCommand a;
	a.mode = 2;
	a.m0target = m0;
	a.m1target = m1;
	WM_Send(a);
}

void WM_Restart() {
	motorCommand a;
	a.mode = 10;
	a.m0target = 0;
	a.m1target = 0;
	WM_Send(a);
}
