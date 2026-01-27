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
	buf[9] = '\n';
	HAL_UART_Transmit(&huart4, &buf[0], sizeof(buf), HAL_MAX_DELAY);
}

void WM_Restart() {
	motorCommand a;
	a.mode = 255;
	a.m0target = 0;
	a.m1target = 0;
	WM_Send(a);
}
