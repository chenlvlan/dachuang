/*
 * compute.h
 *
 *  Created on: Jan 22, 2026
 *      Author: yufei
 */

#ifndef USER_SYS_COMPUTE_H_
#define USER_SYS_COMPUTE_H_

#include <math.h>
#include <stdbool.h>
#include "arm_math.h"

#define EPS 1e-6f

/*
typedef struct {
	float L1;   // 前大腿长度
	float L2;   // 后大腿长度
	float L3;   // 前小腿长度
	float L4;   // 后小腿长度
	float d;    // 前后电机水平间距
} FiveBarGeom_t;

typedef struct {
	float theta_f_min;   // 前腿最小角（rad）
	float theta_f_max;   // 前腿最大角
	float theta_r_min;   // 后腿最小角
	float theta_r_max;   // 后腿最大角
} FiveBarLimit_t;
*/

typedef struct {
	float L1;   // 前大腿长度
	float L2;   // 后大腿长度
	float L3;   // 前小腿长度
	float L4;   // 后小腿长度
	float d;    // 前后电机水平间距
	float theta_f_min;   // 前腿最小角（rad）
	float theta_f_max;   // 前腿最大角
	float theta_r_min;   // 后腿最小角
	float theta_r_max;   // 后腿最大角
	float x;   //目标x位置
	float y;   //目标y位置
	float theta_f;   //前腿电机角
	float theta_r;   //后腿电机角
	uint8_t status;//状态
} legData;

typedef enum {
	IK_OK = 0, IK_OUT_OF_REACH,     // 足端不可达
	IK_NUMERIC_ERROR,   // acos / sqrt 数值错误
	IK_JOINT_LIMIT      // 关节超限
} IKStatus_t;

void fivebar_inverse_kinematics(legData *leg_data);

float clampf(float x, float min, float max);

void quat2euler(float w, float x, float y, float z, float *roll, float *pitch,
		float *yaw);

#endif /* USER_SYS_COMPUTE_H_ */
