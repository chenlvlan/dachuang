/*
 * app.c
 *
 *  Created on: Dec 11, 2025
 *      Author: yufei
 */

#include "app.h"

bool doMotionCtrlCycle = 0;
motorDataRead_t JMDataRead[4] = { 0 };

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
	HAL_Delay(250);
	JM_ReturnToOrigin(idLF);	//转到原点，以展示回原点是否正确
	JM_ReturnToOrigin(idLR);
	JM_ReturnToOrigin(idRF);
	JM_ReturnToOrigin(idRR);

	//回原点完毕，安全地恢复现场
	//........
}

void motionCtrlCycle() {
	//运动控制环

}

void appLoop() {
	//这个是主程序
	if (doMotionCtrlCycle == 1) {
		doMotionCtrlCycle = 0;
		motionCtrlCycle();
	}
	HAL_Delay(500);

	unsigned char msg[] = "1,10.0,1,-10.0\n";
	HAL_UART_Transmit(&huart4, msg, sizeof(msg), 0xffff);
	//printf("111\n");
}

void appSetup() {
	HVHP(1); //母线上电
	HAL_Delay(250); //等待供电稳定
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		// ---- 这里执行你的 20ms 控制环 ----
		doMotionCtrlCycle = 1;
	}
}

