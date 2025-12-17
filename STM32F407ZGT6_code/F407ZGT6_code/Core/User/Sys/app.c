/*
 * app.c
 *
 *  Created on: Dec 11, 2025
 *      Author: yufei
 */

#include "app.h"

bool doMotionCtrlCycle=0;

void returnToOrigin(float speed, float torque, uint32_t timeout) {
	uint32_t startTime = HAL_GetTick(); //开始时间
	while(HAL_GetTick()-startTime<=timeout){//在超时时间以内的话

	}
}

void motionCtrlCycle(){
	//运动控制环

}

void app(){
	//这个是主程序
	if(doMotionCtrlCycle==1){
		doMotionCtrlCycle=0;
		motionCtrlCycle();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		// ---- 这里执行你的 20ms 控制环 ----
		doMotionCtrlCycle = 1;
	}
}

