/*
 * misc.h
 *
 *  Created on: Dec 4, 2025
 *      Author: yufei
 */

#ifndef USER_COMM_STRUCT_H_
#define USER_COMM_STRUCT_H_

#include "main.h"
#include <stdbool.h>

typedef struct {
	//软件信息
	uint16_t bootVer; //Boot软件版本
	uint16_t appVer; //应用软件版本
	uint16_t hardwareVer; //硬件版本
	uint8_t canSelfDefVer; //CAN自定义版本

	//运动控制参数
	float posKp; //位置控制闭环Kp
	float posKi; //位置控制闭环Ki
	float velKp; //速度控制闭环Kp
	float velKi; //速度控制闭环Ki
	float offsetAngle;	//Rad
	float posModeMaxVel;	//Rad/s
	float posVelModeMaxIq;	//A
	float iqSlope;	//A/s
	float velModeAcc;	//Rad/s^2
	bool motorBreak;

	//实时参数
	float iq;	//A
	float mechVel;	//Rad
	float singleAbsAngle;	//Rad
	float multiAbsAngle;	//Rad
	float busVolt;	//V
	float busCurr;	//A
	uint8_t temperature;	//℃
	uint8_t runStatus;
	uint8_t motorStatue;
	uint8_t errorCode;

	//电机信息
	uint8_t polarPair;
	float torqueConst;	//(N*m)/A
	uint8_t reduceRatio;

} motorDataRead_t

enum idJM {
	LF = 11, LR = 12, RF = 14, RR = 13,
};

enum indexJM {
	LF = 0, LR = 1, RF = 3, RR = 2,
};

void HVHP(bool isEN);

#endif /* USER_COMM_STRUCT_H_ */
