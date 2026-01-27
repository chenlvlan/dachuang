/*
 * joint_motor.h
 *
 *  Created on: Jan 27, 2026
 *      Author: yufei
 */

#ifndef USER_JOINT_MOTOR_JOINT_MOTOR_H_
#define USER_JOINT_MOTOR_JOINT_MOTOR_H_

#include "can.h"
#include <stdbool.h>
#include <main.h>
#include <stdio.h>
#include <string.h>

#include "comm_can.h"

#define isDebug 0
//正常运行模式，不打印调试信息为0，打印调试信息为1

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
	float torque;
	float mechVel;	//Rad/s
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

} motorDataRead_t;

extern motorDataRead_t JMDataRead[4];
extern CanRxMessage_t CanRxMessage;

enum idJM {
	idLF = 11, idLR = 12, idRF = 14, idRR = 13,
};

enum indexJM {
	indexLF = 0, indexLR = 1, indexRF = 3, indexRR = 2,
};

enum status {
	off = 0,
	volatgeControl = 1,
	qAxisCurrentControl = 2,
	velocityControl = 3,
	positionControl = 4,
	motorDisable = 0,
};

void JM_Disable(uint8_t id);
void JM_SetPosVelModeMaxTorque(uint8_t id, float torque);
void JM_GetSoftwareInfo(uint8_t id);
void JM_GetRealTimeStatusInfo(uint8_t id);
void JM_GetMotorInfo(uint8_t id, bool forced); //因为力矩常数是很重要的参数，所以可以用强制模式
void JM_VelMode(uint8_t id, float speed);
void JM_PosRelaMode(uint8_t id, float position);
void JM_PosAbsMode(uint8_t id, float position);
void JM_GetTorque(uint8_t id);
void JM_GetPos(uint8_t id);
void JM_Restart(uint8_t id);
void JM_ReturnToOrigin(uint8_t id);
void solveMotorCanRx(motorDataRead_t *motorDataRead);



static inline float degToRad(float deg) {
	return deg * M_PI / 180.0;
}

static inline float radToDeg(float rad) {
	return rad * 180.0 / M_PI;
}

static inline float rpmToRadps(float rpm) {
	return rpm * M_PI / 30.0;
}

static inline float radpsToRpm(float radps) {
	return radps * 30.0 / M_PI;
}

//id为id的电机应该采用哪个can发送？
static inline CAN_HandleTypeDef* idToHandle(uint8_t id) {
	CAN_HandleTypeDef *HCAN;
	if (id == idLF || id == idLR) {
		HCAN = &hcan1;
	} else if (id == idRF || id == idRR) {
		HCAN = &hcan2;
	}
	return HCAN;
}

//电机id映射到数组index
static inline uint8_t idToIndex(uint8_t id) {
	uint8_t index = 0;
	switch (id) {
	case idLF:
		index = indexLF;
		break;
	case idLR:
		index = indexLR;
		break;
	case idRF:
		index = indexRF;
		break;
	case idRR:
		index = indexRR;
		break;
	}
	return index;
}

//数组index映射到电机id
static inline uint8_t indexToID(uint8_t index) {
	uint8_t id = 0;
	switch (index) {
	case indexLF:
		id = idLF;
		break;
	case indexLR:
		id = idLR;
		break;
	case indexRF:
		id = idRF;
		break;
	case indexRR:
		id = idRR;
		break;
	}
	return id;
}



#endif /* USER_JOINT_MOTOR_JOINT_MOTOR_H_ */
