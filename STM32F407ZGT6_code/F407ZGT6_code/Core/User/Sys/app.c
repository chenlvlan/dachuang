/*
 * app.c
 *
 *  Created on: Dec 11, 2025
 *      Author: yufei
 */

#include "app.h"
#include "arm_math.h"

bool doMotionCtrlCycle = 0;
legData_t legData = { .L1 = 90.0f, .L2 = 90.0f, .L3 = 130.0f, .L4 = 130.0f, .d =
		65.5f, .theta_f_max = 1.448623f, .theta_f_min = 0.0f, .theta_r_max =
		1.448623f, .theta_r_min = 0.0f, .x = 0.0f, .y = -150.0f };
wheelMotorData_t wheelMotorData = { .mode = WM_Torque };

float quat_nom[4] = { 0 };
float roll, yaw, pitch;

#define CTRL_DT   0.02f   // 20ms, 50Hz

/* PID 实例（DSP库要求） */
arm_pid_instance_f32 pid_pitch;     // 姿态 PID（输出力矩）
arm_pid_instance_f32 pid_pos;       // 轮子前后位置 PID

/* 状态量（全局共享） */
float pitch;        // 传感器输入
float pitch_rate;   // 可选，用于D项

float x_wheel;      // 当前轮子前后位置（编码器）
float x_ref;        // 轮子前后目标位置（输出）

float wheel_torque_cmd;  // 最终输出力矩

/* 姿态 PID（力矩环，快） */
#define PITCH_KP   0.01f
#define PITCH_KI   0.0000000005f/CTRL_DT
#define PITCH_KD   0.0005f/CTRL_DT

/* 位置 PID（慢，调姿态零点） */
#define POS_KP     0.6f
#define POS_KI     0.0f/CTRL_DT
#define POS_KD     0.1f/CTRL_DT

/* 限幅 */
#define TORQUE_LIMIT   0.11f
#define XREF_LIMIT     10.0f   // ±10cm

void HVHP(bool isEN) {
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, (GPIO_PinState) isEN);
}

void appSetup() {
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);   // 例：INT 接在 PA3
	HVHP(1); //母线上电
	WM_SendRestart(); //复位驱动器
	HAL_Delay(1000); //这个延时必须加，不然在上电（冷启动，不是按reset那种）后MPU6500会初始化失败
	mpu6500_SPIInit();
	cli_init();

	printf("CLI ready, type 'help'\r\n");
	//HAL_Delay(150); //等待供电稳定
	JM_CommInit();
	HAL_Delay(100);

	//上电后的基本信息读取
	JM_RefreshAll(idLF);
	JM_RefreshAll(idLR);
	JM_RefreshAll(idRF);
	JM_RefreshAll(idRR);

	HAL_TIM_Base_Start_IT(&htim3); //运动控制环开始定时

	//警告：在限位块未安装的时候，严禁执行回原点程序，否则会导致撞机
	JM_FindOrigin(0.4, 0.3, 3000); //回原点

	//开启6500的中断捕获
	HAL_NVIC_ClearPendingIRQ(EXTI3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	control_init();
	HAL_Delay(2000);
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
		//printf("%.5f, %.5f, %.5f, ", roll, pitch, yaw);
		control_loop(pitch);
		//legData.x = -x_ref;
		//fivebar_inverse_kinematics(&legData);
		//printf("%.5f, %.5f, %.5f, %.5f, %.5f\r\n", roll, pitch, yaw, legData.x,
		//		wheel_torque_cmd);
		/*
		 JM_PosAbsMode(idLF, legData.theta_f);
		 JM_PosAbsMode(idRF, legData.theta_f);
		 JM_PosAbsMode(idLR, legData.theta_r);
		 JM_PosAbsMode(idRR, legData.theta_r);
		 */
		wheelMotorData.m0target = wheel_torque_cmd;
		wheelMotorData.m1target = wheel_torque_cmd;
		//WM_SendTorque(wheel_torque_cmd, wheel_torque_cmd);
		WM_Send(&wheelMotorData);
		WM_Receive(&wheelMotorData.m0velocity, &wheelMotorData.m0torque,
				&wheelMotorData.m1velocity, &wheelMotorData.m1torque);
		//WM_SendTorque(0.00114, 0.00114);
		//WM_Disable();
		printf("%.5f, %.5f, %.5f, %.5f\r\n", wheelMotorData.m0velocity,
				wheelMotorData.m0torque, wheelMotorData.m1velocity,
				wheelMotorData.m1torque);
	}
	cli_poll();
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
	/* pitch PID */
	pid_pitch.Kp = PITCH_KP;
	pid_pitch.Ki = PITCH_KI;
	pid_pitch.Kd = PITCH_KD;
	//pid_pitch.
	arm_pid_init_f32(&pid_pitch, 1);

	/* position PID */
	pid_pos.Kp = POS_KP;
	pid_pos.Ki = POS_KI;
	pid_pos.Kd = POS_KD;
	arm_pid_init_f32(&pid_pos, 1);

	x_ref = 0.0f;
	wheel_torque_cmd = 0.0f;
}

void control_loop(float pitch_meas) {

	/*
	 float alpha = 0.9f; // 滤波系数，0~1
	 pitch_filt = (1 - alpha) * pitch_filt + alpha * pitch_raw;
	 //pitch_filt = pitch_raw;
	 printf("%.5f, ", pitch_filt);
	 */

	float pitch_err;
	float pos_err;

	//wheel_torque_cmd=PITCH_KP*(0-pitch_meas);
	//pitch = pitch_meas;

	/* ---------- 1️⃣ 姿态环：算力矩 ---------- */
	pitch_err = 0.5f - pitch_meas;   // 目标 pitch = 0
	wheel_torque_cmd = arm_pid_f32(&pid_pitch, pitch_err);
	wheel_torque_cmd = clampf(wheel_torque_cmd, -TORQUE_LIMIT, TORQUE_LIMIT);

	/* ---------- 2️⃣ 位置环：动态调 x_ref ---------- */
	/* 核心思想：
	 pitch ≠ 0 → 轮子应该前/后挪来“接住身体”
	 */

	//pos_err = pitch;   // pitch 越大，位置偏移越多
	//x_ref += arm_pid_f32(&pid_pos, pitch_meas);
	//x_ref = clampf(x_ref, -XREF_LIMIT, XREF_LIMIT);
}
