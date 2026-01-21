/*
 * app.c
 *
 *  Created on: Dec 11, 2025
 *      Author: yufei
 */

#include "app.h"

//#define MPU6500_INTERFACE_SPI
#define CLI_BUF_LEN 64

static char cli_buf[CLI_BUF_LEN];
static uint8_t cli_idx = 0;
static uint8_t cli_rx_char;

bool doMotionCtrlCycle = 0;
motorDataRead_t JMDataRead[4] = { 0 };

//----------------------------------------
int mpu_dmp_int = 0;
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

	//printf("going to turn on the irq\r\n");
	//开启6500的中断捕获
	//HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_ClearPendingIRQ(EXTI3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	//printf("irq is on\r\n");

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

	/*
	 uint32_t startTime = HAL_GetTick(); //开始时间
	 while (HAL_GetTick() - startTime <= timeout) { //在超时时间以内的话
	 //这里面也跑着运动控制环
	 if (doMotionCtrlCycle == 1) {
	 doMotionCtrlCycle == 0;

	 }
	 }
	 */
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == MPU6500_INT_PIN) {

		//uint8_t status;
		//mpu6500_get_reg(&g_mpu6500, 0x3A, &status, 1);  // 关键！

		//printf("INT_STATUS = 0x%02X\r\n", status);
		mpu_dmp_int = 1;
	}
}

void MPU6500_SPIInit() {

	//------------以下为新加的------------
	//uint8_t tmp;

	// 清中断
	//mpu6500_get_reg(&g_mpu6500, 0x3A, &tmp, 1);

	// DMP / FIFO 中断
	//tmp = 0x02;  // FIFO_OFLOW_EN / DMP_INT_EN 由 DMP 内部控制
	//mpu6500_set_reg(&g_mpu6500, 0x38, &tmp, 1);

	// INT 引脚：非锁存，读状态清中断
	//tmp = 0x00;
	//mpu6500_set_reg(&g_mpu6500, 0x37, &tmp, 1);
	//------------以上为新加的------------

}

/* 四元数打印函数 */
void dmp_print_once() {
	//printf("%.4f, %.4f, %.4f, %.4f, ", q0, q1, q2, q3);
	//printf("RPY: R=%6.2f P=%6.2f Y=%6.2f\r\n", roll, pitch, yaw);
	//printf("%.5f, %.5f, %.5f", roll, pitch, yaw);
	//printf("\n");
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
