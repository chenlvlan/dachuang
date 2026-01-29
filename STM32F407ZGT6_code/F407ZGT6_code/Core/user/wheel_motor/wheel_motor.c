/*
 * wheel_motor.c
 *
 *  Created on: Jan 27, 2026
 *      Author: yufei
 */

#include "wheel_motor.h"

#define rxBufSize 32
#define rxDataSize 32

uint8_t txBuf[9];
uint8_t rxBuf[rxBufSize];
uint8_t rxData[rxDataSize];
volatile uint8_t rxDataLen = 0;
volatile uint16_t rxReadPtr = 0;  // 读指针（用户维护）
//volatile uint8_t rxCompleteFlag = 0;
volatile uint16_t frameToDealLen = 0;     // 当前待处理帧长度
volatile uint8_t frameReady = 0;   // 帧就绪标志

void WM_CommInit() {
	// 方法 A: 定长接收（Normal 模式）- 不推荐用于通用接收
	HAL_UART_Receive_DMA(&huart4, &rxBuf[0], rxBufSize);

	// 方法 B: 循环接收（Circular 模式）- 推荐
	//HAL_UART_Receive_DMA(&huart4, rx_buffer, RX_BUF_SIZE);

	// 关键：使能空闲中断（IDLE）- 用于不定长数据
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
}

void WM_Send(wheelMotorData_t mot_cmd) {
	txBuf[0] = mot_cmd.mode;
	memcpy(&txBuf[1], &mot_cmd.m0target, 4);
	memcpy(&txBuf[5], &mot_cmd.m1target, 4);
	HAL_UART_Transmit_DMA(&huart4, &txBuf[0], sizeof(txBuf));
}

void WM_SendDisable() {
	wheelMotorData_t a;
	a.mode = 0;
	a.m0target = 0;
	a.m1target = 0;
	WM_Send(a);
}

void WM_SendVelocity(float m0, float m1) {
	wheelMotorData_t a;
	a.mode = 1;
	a.m0target = m0;
	a.m1target = m1;
	WM_Send(a);
}

void WM_SendTorque(float m0, float m1) {
	wheelMotorData_t a;
	a.mode = 2;
	a.m0target = m0;
	a.m1target = m1;
	WM_Send(a);
}

void WM_SendRestart() {
	wheelMotorData_t a;
	a.mode = 10;
	a.m0target = 0;
	a.m1target = 0;
	WM_Send(a);
}

void WM_Receive(float *m0velocity, float *m0torque, float *m1velocity,
		float *m1torque) {
	//仅当有新帧的时候更新传入的指针
	if(frameReady==1){
		frameReady=0;
		uint32_t m0v=(uint32_t)rxData[3] << 24 | (uint32_t)rxData[2] << 16 | (uint32_t)rxData[1] << 8 | (uint32_t)buf[0];
		uint32_t m0t;
		uint32_t m1v;
		uint32_t m1t;
	}
}

void uart4DMA(UART_HandleTypeDef *huart) {
	if (huart->Instance == UART4) {
		HAL_UART_DMAStop(&huart4);
		// 计算实际接收长度（关键！）
		// 原理：DMA 的 CNDTR 是剩余未传输的字节数

		uint16_t write_ptr = rxBufSize - __HAL_DMA_GET_COUNTER(huart4.hdmarx);
		//rxCompleteFlag = 1;
		uint16_t frame_len;
		if (write_ptr >= rxReadPtr) {
			frame_len = write_ptr - rxReadPtr;  // 正常情况，没有回环
		} else {
			frame_len = (rxBufSize - rxReadPtr) + write_ptr;  // 回环了
		}
		if (frame_len > 0) {
			// 情况 1: 数据没有回环（读指针 -> 写指针 是连续的）
			if (write_ptr > rxReadPtr) {
				memcpy(rxData, &rxBuf[rxReadPtr], frame_len);
			}
			// 情况 2: 数据回环了（分两段拷贝）
			else {
				uint16_t first_part = rxBufSize - rxReadPtr;  // 到缓冲区末尾的长度
				uint16_t second_part = frame_len - first_part;    // 从缓冲区开头开始的长度

				// 先拷贝后半段（缓冲区尾部）
				memcpy(rxData, &rxBuf[rxReadPtr], first_part);
				// 再拷贝前半段（缓冲区头部）
				memcpy(&rxData[first_part], &rxBuf[0], second_part);
			}

			frameToDealLen = frame_len;
			frameReady = 1;  // 标记帧就绪

			// 更新读指针（"清空"已处理数据）
			rxReadPtr = write_ptr;
		}

		HAL_UART_Receive_DMA(&huart4, &rxBuf[1], rxBufSize);
		// Circular 模式无需重启，但需清空已处理数据（可选）
		// 或者调整缓冲区指针（高级用法）
	}
}
