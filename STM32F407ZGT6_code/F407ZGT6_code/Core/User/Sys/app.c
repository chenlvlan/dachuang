/*
 * app.c
 *
 *  Created on: Dec 11, 2025
 *      Author: yufei
 */

#include "app.h"
#include "arm_math.h"

#define MAX_WHEEL_TORQUE 0.11f

bool doMotionCtrlCycle = 0;
motorDataRead_t JMDataRead[4] = { 0 };
motorCommand motorCmd;

float quat_nom[4]={0};
float roll, yaw, pitch;

float pitch_filt;
float pitch_ref = 0.0f;

float u;             // 轮子控制量

#define CTRL_DT   0.02f   // 20ms, 50Hz

arm_pid_instance_f32 pid_pitch;

/* 二阶低通滤波（IMU pitch） */
arm_biquad_cascade_df2T_instance_f32 lp_pitch;
float lp_coeffs[5] = { 0.0675f, 0.1349f, 0.0675f,   // b0 b1 b2
		-1.1430f, 0.4128f             // a1 a2
		};
float lp_state[6] = { 0 };

void HVHP(bool isEN) {
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, (GPIO_PinState) isEN);
}

void appSetup() {
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);   // 例：INT 接在 PA3
	HVHP(1); //母线上电
	WM_Restart();
	HAL_Delay(1000); //这个延时必须加，不然在上电（冷启动，不是按reset那种）后MPU6500会初始化失败
	mpu6500_SPIInit();
	cli_init();

	printf("CLI ready, type 'help'\r\n");
	//HAL_Delay(150); //等待供电稳定
	CommCan_Init(&hcan1); //关节电机can1通信初始化
	CommCan_Init(&hcan2); //关节电机can2通信初始化
	HAL_Delay(100);

	//上电后的基本信息读取
	refreshAll(idLF);
	refreshAll(idLR);
	refreshAll(idRF);
	refreshAll(idRR);

	HAL_TIM_Base_Start_IT(&htim3); //运动控制环开始定时

	//警告：在限位块未安装的时候，严禁执行回原点程序，否则会导致撞机
	returnToOrigin(0.3, 0.2, 3000); //回原点

	//开启6500的中断捕获
	HAL_NVIC_ClearPendingIRQ(EXTI3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	control_init();
	HAL_Delay(3000);
}

void appLoop() {
	if (doMotionCtrlCycle == 1) {
		doMotionCtrlCycle = 0;
		motionCtrlCycle();
	}
	if (mpu6500_isReady()) {
		mpu6500_DMPGet(&quat_nom[0]);
		quat2euler(quat_nom[0], quat_nom[1], quat_nom[2], quat_nom[3], &roll,
				&pitch, &yaw);
		printf("%.5f, %.5f, %.5f, ", roll, pitch, yaw);
		control_loop(pitch);
	}
	cli_poll();
	//HAL_Delay(1000);
}

void motionCtrlCycle() {
//运动控制环

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		// ---- 这里执行你的 20ms 控制环 ----
		doMotionCtrlCycle = 1;
	}
}

void control_init(void) {
	/* -------- PID -------- */
	pid_pitch.Kp = 0.025f;
	pid_pitch.Ki = 0.0f / CTRL_DT;
	pid_pitch.Kd = 0.0f / CTRL_DT;
	arm_pid_init_f32(&pid_pitch, 1);

	/* -------- Low-pass filter -------- */
	//arm_biquad_cascade_df2T_init_f32(&lp_pitch, 1, lp_coeffs, lp_state);
}

void control_loop(float pitch_raw) {
	/* 1. 读取 IMU */
	//pitch_raw = imu_get_pitch_rad();
	/* 2. 低通滤波 */
	//arm_biquad_cascade_df2T_f32(&lp_pitch, &pitch_raw, &pitch_filt, 1);
	float alpha = 0.9f; // 滤波系数，0~1
	pitch_filt = (1 - alpha) * pitch_filt + alpha * pitch_raw;
	//pitch_filt = pitch_raw;
	printf("%.5f, ", pitch_filt);

	/* 3. PID */
	float err = pitch_ref - pitch_filt;
	u = arm_pid_f32(&pid_pitch, err);

	/* 4. 限幅（非常重要） */
	u = clampf(u, -MAX_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
	/* 5. 给轮子 */
	//wheel_set_torque(u, u);
	printf("%.5f\r\n", u);
	motorCmd.mode = 2;
	motorCmd.m0target = u;
	motorCmd.m1target = u;
	WM_Send(motorCmd);
}
