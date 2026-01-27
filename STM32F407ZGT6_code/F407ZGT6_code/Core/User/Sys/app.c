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
//----------------------------------------
int mpu_dmp_int = 0;
int int_count = 0;
short gyro[3], accel[3];
long quat[4];
float quat_nom[4];
unsigned long timestamp;
short sensors;
unsigned char more;
struct int_param_s mpu_int_param;
float roll, yaw, pitch;
//------------------------------------------
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
	motorCmd.mode = 10;
	motorCmd.m0target = 0;
	motorCmd.m1target = 0;
	WM_Send(motorCmd);
	HAL_Delay(1000); //这个延时必须加，不然在上电（冷启动，不是按reset那种）后MPU6500会初始化失败
	MPU6500_SPIInit();
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
	if (mpu_dmp_int) {
		mpu_dmp_int = 0;
		dmp_print_once();
		quat2euler(quat_nom[0], quat_nom[1], quat_nom[2], quat_nom[3], &roll,
				&pitch, &yaw);
		printf("%.5f, %.5f, %.5f, ", roll, pitch, yaw);
		control_loop(pitch);
	}
	cli_poll();

	//HAL_Delay(1000);
}

//警告：这个是阻塞函数，实时状态下禁止使用
void refreshAll(uint8_t id) {
	JM_GetMotorInfo(id, 0);
	HAL_Delay(5);
	JM_GetRealTimeStatusInfo(id);
	HAL_Delay(5);
	JM_GetSoftwareInfo(id);
	HAL_Delay(5);
}

void returnToOrigin(float speed, float torque, uint32_t timeout) {

//.......
//在执行回原点前，先安全地保存现场，完成准备工作

	JM_SetPosVelModeMaxTorque(idLF, torque);	//限制最大力矩
	JM_SetPosVelModeMaxTorque(idLR, torque);
	JM_SetPosVelModeMaxTorque(idRF, torque);
	JM_SetPosVelModeMaxTorque(idRR, torque);

	HAL_Delay(100);
	speed = 0 - fabsf(speed);	//速度为负数才是往原点转
	JM_VelMode(idLF, speed);	//进行归零动作
	JM_VelMode(idLR, speed);
	JM_VelMode(idRF, speed);
	JM_VelMode(idRR, speed);

	//float deltaPos[4][25];
	//这里使用简化的回原点程序，就是在限定力矩的情况下，给充足的时间，并假定所有关节电机都转到
	//了原点，时间一到直接设置当前为原点

	HAL_Delay(timeout);
	JM_PosRelaMode(idRF, 0.2);	//右侧两个电机转至右边的腿分开
	JM_PosRelaMode(idRR, 0.2);
	HAL_Delay(500);
	JM_Restart(idLF);	//重启，读取新的位置
	JM_Restart(idLR);
	JM_Restart(idRF);
	JM_Restart(idRR);
	HAL_Delay(200);	//必须加延时，不然RR电机无法回零
	JM_ReturnToOrigin(idLF);	//转到原点，以展示回原点是否正确
	HAL_Delay(200);
	JM_ReturnToOrigin(idLR);
	HAL_Delay(200);
	JM_ReturnToOrigin(idRF);
	HAL_Delay(200);
	JM_ReturnToOrigin(idRR);
	HAL_Delay(200);
//回原点完毕，安全地恢复现场
//........
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

void WM_Send(motorCommand mot_cmd) {
	uint8_t buf[10] = { 0 };
	buf[0] = mot_cmd.mode;
	memcpy(&buf[1], &mot_cmd.m0target, 4);
	memcpy(&buf[5], &mot_cmd.m1target, 4);
	buf[9] = '\n';
	HAL_UART_Transmit(&huart4, &buf[0], sizeof(buf), HAL_MAX_DELAY);
}
