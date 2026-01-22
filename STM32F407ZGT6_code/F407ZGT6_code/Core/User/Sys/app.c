/*
 * app.c
 *
 *  Created on: Dec 11, 2025
 *      Author: yufei
 */

#include "app.h"

#define CLI_BUF_LEN 64

static char cli_buf[CLI_BUF_LEN];
static uint8_t cli_idx = 0;
static uint8_t cli_rx_char;

bool doMotionCtrlCycle = 0;
motorDataRead_t JMDataRead[4] = { 0 };

//----------------------------------------
int mpu_dmp_int = 0;
int int_count = 0;
short gyro[3], accel[3];
long quat[4];
unsigned long timestamp;
short sensors;
unsigned char more;
struct int_param_s mpu_int_param;
//------------------------------------------

void appSetup() {
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);   // 例：INT 接在 PA3
	HVHP(1); //母线上电
	HAL_Delay(1000); //这个延时必须加，不然在上电（冷启动，不是按reset那种）后MPU6500会初始化失败
	MPU6500_SPIInit();
	cli_init();

	printf("CLI ready, type 'help'\r\n");
	HAL_Delay(150); //等待供电稳定
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
}

void appLoop() {
	//printf("loop begin  ");
	//这个是主程序
	if (doMotionCtrlCycle == 1) {
		doMotionCtrlCycle = 0;
		motionCtrlCycle();
	}
	if (mpu_dmp_int) {
		//printf("\r\n1\r\n");
		dmp_print_once();
		mpu_dmp_int = 0;
	}
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

int MPU6500_SPIInit() {
	int result;

	mpu_int_param.cb = mpu_data_ready;
	mpu_int_param.pin = MPU6500_INT_PIN;
	mpu_int_param.lp_exit = 1;
	mpu_int_param.active_low = 1;
	reg_int_cb(&mpu_int_param); // 注册回调
	// MPU 初始化（motion_driver_6.12 mpu_init 有参数）
	result = mpu_init(&mpu_int_param);  // 注意：必须传 int_param_s*
	printf("mpu_init(&mpu_int_param) = %d\r\n", result);
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

	//mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);

	// 加载 DMP 固件
	result = dmp_load_motion_driver_firmware();
	if (result) {
		printf("dmp_load_motion_driver_firmware failed: %d\n", result);
		return -1;
	}
	// 配置 DMP 输出
	//result = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP| DMP_FEATURE_ANDROID_ORIENT);
	result = dmp_enable_feature( DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL);
	printf("dmp_enable_feature = %d\r\n", result);
	result = dmp_set_fifo_rate(50); // 50 Hz
	result = mpu_set_dmp_state(1); // 打开 DMP
	printf("mpu_set_dmp_state = %d\r\n", result);
	result = mpu_set_int_latched(0); // 使能 MPU 中断
	printf("mpu_set_int_latched = %d\r\n", result);

	return 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == MPU6500_INT_PIN) {
		int_count++;
		if (int_count >= 4) {
			int_count=0;
			if (mpu_int_param.cb) {
				mpu_int_param.cb();  // 调用你写的 mpu_data_ready()
			}
		}
	}
}

void mpu_data_ready(void) {
	mpu_dmp_int = 1;
}

/* 四元数打印函数 */
void dmp_print_once() {
	do {
		dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
	} while (more);

	if (sensors & INV_WXYZ_QUAT) {
		// 这里再 printf / 处理
		const float scale = 1.0f / (1L << 30);
		float q_out[4];

		// 转 float
		float q0 = quat[0] * scale;
		float q1 = quat[1] * scale;
		float q2 = quat[2] * scale;
		float q3 = quat[3] * scale;

		// 计算模长
		float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

		// 防止除 0
		if (norm < 1e-6f) {
			q_out[0] = 1.0f;
			q_out[1] = 0.0f;
			q_out[2] = 0.0f;
			q_out[3] = 0.0f;
			return;
		}

		float inv = 1.0f / norm;

		q_out[0] = q0 * inv;
		q_out[1] = q1 * inv;
		q_out[2] = q2 * inv;
		q_out[3] = q3 * inv;
		printf("%.5f, %.5f, %.5f, %.5f\r\n", q_out[0], q_out[1], q_out[2],
				q_out[3]);
	}

}

void cli_init(void) {
	HAL_UART_Receive_IT(&huart1, &cli_rx_char, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart != &huart1)
		return;

	//printf("aaa\r\n");
	if (cli_rx_char == '\r' || cli_rx_char == '\n') {
		cli_buf[cli_idx] = '\0';
		cli_idx = 0;

		cli_handle_command(cli_buf);
	} else {
		if (cli_idx < CLI_BUF_LEN - 1) {
			cli_buf[cli_idx++] = cli_rx_char;
		}
	}

	HAL_UART_Receive_IT(&huart1, &cli_rx_char, 1);
}

void cli_handle_command(char *cmd) {
	if (strlen(cmd) == 0)
		return;

	if (strcmp(cmd, "help") == 0) {
		printf("Commands:\r\n");
		printf("  delay           show delay(ms)\r\n");
		printf("  delay <num>     set delay(ms)\r\n");
	} else if (strncmp(cmd, "delay", 5) == 0) {
		uint32_t val;

		if (sscanf(cmd, "delay %lu", &val) == 1) {
			//g_loop_delay_ms = val;
			//printf("delay set to %lu ms\r\n", g_loop_delay_ms);
		} else {
			//printf("delay = %lu ms\r\n", g_loop_delay_ms);
		}
	} else {
		printf("unknown cmd: %s\r\n", cmd);
	}
}
