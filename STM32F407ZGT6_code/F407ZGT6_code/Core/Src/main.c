/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "test.h"
#include "../User/Sys/misc.h"
#include "../User/Sys/app.h"
#include "../User/Comm/comm_can.h"
#include <stdio.h>
//#include "../User/Comm/not_use/comm_rs485.h.1"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int _write(int file, char *ptr, int len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	/**
	 * 需要做的事：
	 * 1、测试主控是否能接收到关节电机can通信返回的数据（已实现！）
	 * 2、写一个中间层，介于应用层的代码与关节电机控制函数之间，可以被应用层调用，转换后传给控制函数
	 * 3、加一个定时器，周期20ms（50Hz），作为运动控制环，控制关节电机和轮毂电机（已实现！）
	 * 4、完善FOC驱动器的代码，增加与主控的通信
	 * 5、增加上电后的无传感器回零功能，其中，LF和RF关节电机下摆直至撞到碳板（此时与原点成
	 * 确定的角度），LR和RR关节电机增加限位块。基本步骤：
	 * （1）设置速度模式的力矩为一个较小但要够用的值（保证可以撞到限位但不会造成损坏）
	 * （2）设置为速度模式，给一个不用很大的速度，缓慢移动，撞到限位
	 * （3）在第二步的同时，不停地读取力矩，当力矩达到设定值（允许有一定的偏差）时，认为撞到了限位
	 * （4）相对位置控制，旋转一定角度（需要提前测算好），使得大腿竖直下垂
	 * ！！！需询问伺泰威的工程师，相对位置控制以及设置当前位置为原点，是按照当前的机械角度（实际角度）
	 * 还是目标角度？重要！！！（已询问待回复）
	 * （5）设置当前位置（大腿竖直下垂）为原点
	 * 6、尽早实现逆运动学
	 * 7、尽早实现轮足机器人动力学
	 * 8、尽早实现读取IMU
	 */
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART3_UART_Init();
	MX_USART6_UART_Init();
	MX_CAN1_Init();
	MX_CAN2_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_SPI2_Init();
	MX_I2C2_Init();
	MX_UART4_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_UART5_Init();
	MX_ADC2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	appSetup();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		HAL_Delay(1000);
		CommCan_SetAbsPosition_Count(&hcan1, idLF, 0);
		CommCan_SetAbsPosition_Count(&hcan1, idLR, 0);
		CommCan_SetAbsPosition_Count(&hcan2, idRF, 0);
		CommCan_SetAbsPosition_Count(&hcan2, idRR, 0);
		HAL_Delay(1000);
		CommCan_SetAbsPosition_Count(&hcan1, idLF, 1024);
		CommCan_SetAbsPosition_Count(&hcan1, idLR, 1024);
		CommCan_SetAbsPosition_Count(&hcan2, idRF, 1024);
		CommCan_SetAbsPosition_Count(&hcan2, idRR, 1024);
		//printf("a cycle\n");

		//appLoop();
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len, HAL_MAX_DELAY);
	return len;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
