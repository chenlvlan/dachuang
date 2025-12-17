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

extern bool doMotionCtrlCycle;

//speed in Rad/s, torque in N.m, timeout in ms
void returnToOrigin(float speed, float torque, uint32_t timeout);
void app();
void motionCtrlCycle();

#endif /* USER_SYS_APP_H_ */
