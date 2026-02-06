/*
 * app.c
 *
 *  Created on: Dec 11, 2025
 *      Author: yufei
 */

#include "app.h"
//#include "arm_math.h"

bool doMotionCtrlCycle = 0;
legData_t legData = { .L1 = 90.0f, .L2 = 90.0f, .L3 = 130.0f, .L4 = 130.0f, .d =
		65.5f, .theta_f_max = 1.448623f, .theta_f_min = 0.0f, .theta_r_max =
		1.448623f, .theta_r_min = 0.0f, .x = 0.0f, .y = -190.0f };
wheelMotorData_t wheelMotorData = { .mode = WM_Torque };

float quat_nom[4] = { 0 };
float roll, yaw, pitch;

controlData_t ctrlData;

/* 状态量（全局共享） */
//float pitch_rate;   // 可选，用于D项
//float pitch_avg = 0.0f;
//float x_ref;        // 轮子前后目标位置（输出）
//float pitch_lpf_alpha = 0.9f;  // 时间常数 ~1s
//float wheel_torque_cmd;  // 最终输出力矩
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
		ctrlData.roll = roll;
		ctrlData.pitch = pitch;
		ctrlData.yaw = yaw;

		control_loop(&ctrlData);
		control_loop_simulink(&ctrlData);

		legData.x = ctrlData.xRefLeft; //以左边的为基准
		fivebar_inverse_kinematics(&legData);
		//printf("%.5f, %.5f, %.5f, %.5f, %.5f\r\n", roll, pitch, yaw, legData.x,
		//		wheel_torque_cmd);

		/*
		uint8_t tmp[16];
		memcpy(&tmp[0], &roll, 4);
		memcpy(&tmp[4], &pitch, 4);
		memcpy(&tmp[8], &yaw, 4);
		tmp[12] = 0x00;
		tmp[13] = 0x00;
		tmp[14] = 0x80;
		tmp[15] = 0x7f;
		HAL_UART_Transmit(&huart1, &tmp[0], 16, 0xffff);
		*/

		JM_PosAbsMode(idLF, legData.theta_f);
		JM_PosAbsMode(idRF, legData.theta_f);
		JM_PosAbsMode(idLR, legData.theta_r);
		JM_PosAbsMode(idRR, legData.theta_r);

		wheelMotorData.m0target = ctrlData.m0torque;
		wheelMotorData.m1target = ctrlData.m1torque;
		//WM_SendTorque(wheel_torque_cmd, wheel_torque_cmd);
		WM_Send(&wheelMotorData);
		WM_Receive(&wheelMotorData.m0velocity, &wheelMotorData.m0torque,
				&wheelMotorData.m1velocity, &wheelMotorData.m1torque);
		ctrlData.m0speed = wheelMotorData.m0velocity;
		ctrlData.m1speed = wheelMotorData.m1velocity;
		ctrlData.m0torque = wheelMotorData.m0torque;
		ctrlData.m1torque = wheelMotorData.m1torque;
		//WM_SendTorque(0.00114, 0.00114);
		//WM_Disable();
		//printf("%.5f, %.5f, %.5f, %.5f\r\n", wheelMotorData.m0velocity,
		//		wheelMotorData.m0torque, wheelMotorData.m1velocity,
		//		wheelMotorData.m1torque);
	}
	//cli_poll();
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
