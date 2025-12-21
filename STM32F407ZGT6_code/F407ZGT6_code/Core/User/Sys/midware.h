/*
 * midware.h
 *
 *  Created on: Dec 17, 2025
 *      Author: yufei
 *
 * 中间件
 */

#ifndef USER_SYS_MIDWARE_H_
#define USER_SYS_MIDWARE_H_

#include "../Comm/comm_can.h"
#include "misc.h"
#include <stdbool.h>
#include <main.h>
#include <stdio.h>
#include <string.h>

#define isDebug 1
//正常运行模式，不打印调试信息为0，打印调试信息为1

extern motorDataRead_t JMDataRead[4];
extern CanRxMessage_t CanRxMessage;

void JM_Disable(uint8_t id);
void JM_SetPosVelModeMaxTorque(uint8_t id, float torque);
void JM_GetSoftwareInfo(uint8_t id);
void JM_GetRealTimeStatusInfo(uint8_t id);
void JM_GetMotorInfo(uint8_t id, bool forced);//因为力矩常数是很重要的参数，所以可以用强制模式
void JM_VelMode(uint8_t id, float speed);
void JM_PosRelaMode(uint8_t id, float position);
void JM_PosAbsMode(uint8_t id, float position);
void JM_GetTorque(uint8_t id);
void JM_GetPos(uint8_t id);
void JM_Restart(uint8_t id);
void JM_ReturnToOrigin(uint8_t id);

void HVHP(bool isEN);

void solveMotorCanRx(motorDataRead_t *motorDataRead);

#endif /* USER_SYS_MIDWARE_H_ */
