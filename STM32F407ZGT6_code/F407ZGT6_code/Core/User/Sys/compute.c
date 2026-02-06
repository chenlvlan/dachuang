/*
 * compute.c
 *
 *  Created on: Jan 22, 2026
 *      Author: yufei
 */

#include "compute.h"

#define rxBufSize 32
#define rxDataSize 32

//#define DSP
/* PID 实例（DSP库要求） */
arm_pid_instance_f32 pid_pitch;     // 姿态 PID（输出力矩）
arm_pid_instance_f32 pid_speed;
//arm_pid_instance_f32 pid_x;

static uint8_t rxBuf[rxBufSize];
static uint8_t rxData[rxDataSize];
static volatile uint16_t frameToDealLen = 0;     // 当前待处理帧长度
static volatile uint8_t frameReady = 0;   // 帧就绪标志

float clampf(float x, float min, float max) {
	if (x < min)
		return min;
	if (x > max)
		return max;
	return x;
}

//前大腿，后大腿，前小腿，后小腿，电机间距
//const static float legData[5] = { 90.0, 90.0, 130.0, 130.0, 65.5 };

void fivebar_inverse_kinematics(legData_t *leg_data) {
	/* ================= 前腿 ================= */
	float x_offset = (-32.75f);
	float y_offset = (0.0f);
	float x = leg_data->x + x_offset;
	float y = leg_data->y + y_offset;
	float rf2 = x * x + y * y;
	if (rf2 < EPS)
		leg_data->status = IK_NUMERIC_ERROR;

	float rf = sqrtf(rf2);

	if (rf > (leg_data->L1 + leg_data->L3)
			|| rf < fabsf(leg_data->L1 - leg_data->L3))
		leg_data->status = IK_OUT_OF_REACH;

	float cos_af = (leg_data->L1 * leg_data->L1 + rf * rf
			- leg_data->L3 * leg_data->L3) / (2.0f * leg_data->L1 * rf);
	cos_af = clampf(cos_af, -1.0f, 1.0f);

	float alpha_f = acosf(cos_af);

	/* 数学角：+Y 为 0，逆时针为正 */
	float phi_f = atan2f(x, y);

	/* 选择膝盖朝前（凸结构） */
	float th_f = phi_f - alpha_f;

	/* ================= 后腿 ================= */
	/* 后电机在 (-d, 0) */
	float xr = x + leg_data->d;
	float yr = y;

	float rr2 = xr * xr + yr * yr;
	if (rr2 < EPS)
		leg_data->status = IK_NUMERIC_ERROR;

	float rr = sqrtf(rr2);

	if (rr > (leg_data->L2 + leg_data->L4)
			|| rr < fabsf(leg_data->L2 - leg_data->L4))
		leg_data->status = IK_OUT_OF_REACH;

	float cos_ar = (leg_data->L2 * leg_data->L2 + rr * rr
			- leg_data->L4 * leg_data->L4) / (2.0f * leg_data->L2 * rr);
	cos_ar = clampf(cos_ar, -1.0f, 1.0f);

	float alpha_r = acosf(cos_ar);

	float phi_r = atan2f(xr, yr);

	/* 选择膝盖朝后（凸结构） */
	float th_r = phi_r + alpha_r;

	th_f = 0.0f - (th_f + M_PI);
	th_r = th_r - M_PI; //关节角取反

	/* ---------- 关节限位 ---------- */

	if (th_f < leg_data->theta_f_min || th_f > leg_data->theta_f_max
			|| th_r < leg_data->theta_r_min || th_r > leg_data->theta_r_max)
		leg_data->status = IK_JOINT_LIMIT;

	leg_data->theta_f = th_f;
	leg_data->theta_r = th_r;

	leg_data->status = IK_OK;
}

void quat2euler(float w, float x, float y, float z, float *roll, float *pitch,
		float *yaw) {
	/* -------- Roll (X axis) -------- */
	float sinr_cosp = 2.0f * (w * x + y * z);
	float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
	*roll = atan2f(sinr_cosp, cosr_cosp);

	/* -------- Pitch (Y axis) -------- */
	float sinp = 2.0f * (w * y - z * x);
	if (sinp >= 1.0f)
		*pitch = M_PI / 2.0f;
	else if (sinp <= -1.0f)
		*pitch = -M_PI / 2.0f;
	else
		*pitch = asinf(sinp);

	/* -------- Yaw (Z axis) -------- */
	float siny_cosp = 2.0f * (w * z + x * y);
	float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
	*yaw = atan2f(siny_cosp, cosy_cosp);

	*roll *= 57.29578f;
	*yaw *= 57.29578f;
	*pitch *= 57.29578f;
}

void control_init() {
	/* pitch PID */
	pid_pitch.Kp = PITCH_KP;
	pid_pitch.Ki = PITCH_KI;
	pid_pitch.Kd = PITCH_KD;
	arm_pid_init_f32(&pid_pitch, 1);

	pid_speed.Kp = SPEED_KP;
	pid_speed.Ki = SPEED_KI;
	pid_speed.Kd = SPEED_KD;
	arm_pid_init_f32(&pid_speed, 1);

	//pitch_avg = 0.0f;
	//x_ref = 0.0f;
	//wheel_torque_cmd = 0.0f;
}

void control_loop(controlData_t *ctrlData) {
	/* ========== 1. 姿态控制（快） ========== */
	float pitch_err = 0.0f - ctrlData->pitch;
	float torque_balance = arm_pid_f32(&pid_pitch, pitch_err);

	/* ========== 3. 力矩合成 ========== */
	float torque = torque_balance/* + torque_damp*/;
	ctrlData->m0torque = clampf(torque, -TORQUE_LIMIT, TORQUE_LIMIT);
	ctrlData->m1torque = ctrlData->m0torque;

	/* ========== 4. pitch 慢平均（给腿用） ========== */
	//pitch_avg += pitch_lpf_alpha * (pitch - pitch_avg);//低通滤波
	float Kx = 2.0f;  // m / rad / s（非常小）
	/* pitch_avg ≠ 0 说明结构不平衡 */
	//x_ref += Kx * pitch_avg;
	ctrlData->xRefLeft = /*Kx * pitch+*/0.5f
			* (((ctrlData->m0speed + ctrlData->m1speed) / 2.0f) - 0.0f);
	ctrlData->xRefLeft = clampf(ctrlData->xRefLeft, -XREF_LIMIT, XREF_LIMIT);
	ctrlData->xRefRight = ctrlData->xRefLeft;
}

void control_loop_simulinkLoopTest(controlData_t *ctrlData) {
	/* ========== 1. 姿态控制（快） ========== */
	float pitch_err = 0.0f - ctrlData->pitch;
	float torque_balance = arm_pid_f32(&pid_pitch, pitch_err);
	/* ========== 3. 力矩合成 ========== */
	float torque = torque_balance/* + torque_damp*/;
	torque = clampf(torque, -TORQUE_LIMIT, TORQUE_LIMIT);
	/* ========== 4. pitch 慢平均（给腿用） ========== */
	float xRef = /*Kx * pitch+*/0.5f
			* (((ctrlData->m0speed + ctrlData->m1speed) / 2.0f) - 0.0f);
	xRef = clampf(xRef, -XREF_LIMIT, XREF_LIMIT);

	uint8_t tmp[12];
	memcpy(&tmp[0], &torque, 4);
	memcpy(&tmp[4], &xRef, 4);
	//memcpy(&tmp[8], &torque, 4);
	tmp[8] = 0x00;
	tmp[9] = 0x00;
	tmp[10] = 0x80;
	tmp[11] = 0x7f;
	HAL_UART_Transmit(&huart1, &tmp[0], 12, 0xffff);
}

void control_loop_simulinkLoopTestRx(controlData_t *ctrlData) {
	//仅当有新帧的时候更新传入的指针
	if (frameReady == 1) {
		frameReady = 0;

		//printf("get Rx\r\n");
		uint32_t torque = (uint32_t) rxData[3] << 24
				| (uint32_t) rxData[2] << 16 | (uint32_t) rxData[1] << 8
				| (uint32_t) rxData[0];
		uint32_t xRef = (uint32_t) rxData[7] << 24 | (uint32_t) rxData[6] << 16
				| (uint32_t) rxData[5] << 8 | (uint32_t) rxData[4];
		uint32_t yRef = (uint32_t) rxData[11] << 24
				| (uint32_t) rxData[10] << 16 | (uint32_t) rxData[9] << 8
				| (uint32_t) rxData[8];
		//printf("%02X%02X%02X%02X\r\n", rxData[3], rxData[2], rxData[1],
		//		rxData[0]);
		//printf("%l\r\n",m0v);
		float torque_f, xRef_f, yRef_f;
		memcpy(&torque_f, &torque, 4);
		memcpy(&xRef_f, &xRef, 4);
		memcpy(&yRef_f, &yRef, 4);
		if (yRef_f >= -50) {
			yRef_f = -200;
		}
		ctrlData->m0torque = torque_f;
		ctrlData->m1torque = torque_f;
		ctrlData->xRefLeft = xRef_f;
		ctrlData->xRefRight = xRef_f;
		ctrlData->yRefLeft = yRef_f;
		ctrlData->yRefRight = yRef_f;
	}
}

void uart1DMA(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		HAL_UART_DMAStop(&huart1);
		//printf("cool we are going to deal the DMA\r\n");
		uint16_t frame_len = rxBufSize - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
		//printf("NDTR=%d\r\n", __HAL_DMA_GET_COUNTER(huart1.hdmarx));
		//HAL_UART_DMAStop(&huart1);
		//printf("frame_len=%d\r\n",frame_len);
		if (frame_len == 16) {
			frameToDealLen = frame_len;
			frameReady = 1;  // 标记帧就绪
			memcpy(&rxData[0], &rxBuf[0], frame_len);
		} else {
			frameToDealLen = 0;
			frameReady = 0;
		}

		HAL_UART_Receive_DMA(&huart1, &rxBuf[0], rxBufSize);

	}
}

void control_comm_init() {
	HAL_UART_Receive_DMA(&huart1, &rxBuf[0], rxBufSize);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}
