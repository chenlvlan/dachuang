/*
 * compute.h
 *
 *  Created on: Jan 22, 2026
 *      Author: yufei
 */

#ifndef USER_SYS_COMPUTE_H_
#define USER_SYS_COMPUTE_H_

#include "main.h"
#include <math.h>
#include "dma.h"
#include "usart.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "arm_math.h"

extern UART_HandleTypeDef huart1;

#define EPS 1e-6f
#define CTRL_DT   0.02f   // 20ms, 50Hz

/* 姿态 PID（力矩环，快） */
#define PITCH_KP   0.02f
#define PITCH_KI   0.0000000005f/CTRL_DT
#define PITCH_KD   0.005f/CTRL_DT

#define SPEED_KP 0.01f
#define SPEED_KI 0.0f/CTRL_DT
#define SPEED_KD 0.0f/CTRL_DT

/* 限幅 */
#define TORQUE_LIMIT   0.11f
#define XREF_LIMIT     20.0f   // ±20mm

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
	uint8_t status;   //状态
} legData_t;

typedef struct {
	//输入
	float roll;
	float pitch;
	float yaw;
	float m0speed;
	float m1speed;
	float m0torque;   //兼做输出
	float m1torque;
	//输出
	float xRefLeft;
	float xRefRight;
	float yRefLeft;
	float yRefRight;
} controlData_t;

typedef enum {
	IK_OK = 0, IK_OUT_OF_REACH,     // 足端不可达
	IK_NUMERIC_ERROR,   // acos / sqrt 数值错误
	IK_JOINT_LIMIT      // 关节超限
} IKStatus_t;

void fivebar_inverse_kinematics(legData_t *leg_data);

float clampf(float x, float min, float max);

void quat2euler(float w, float x, float y, float z, float *roll, float *pitch,
		float *yaw);

void control_init();

void control_comm_init();
void uart1DMA(UART_HandleTypeDef *huart);
void control_loop(controlData_t *ctrlData);
void control_loop_simulinkLoopTest(controlData_t *ctrlData);

void control_loop_simulinkLoopTestRx(controlData_t *ctrlData);
#endif /* USER_SYS_COMPUTE_H_ */
