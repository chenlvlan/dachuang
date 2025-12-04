/*
 * test.c
 *
 *  Created on: Dec 3, 2025
 *      Author: yufei
 */

#include "test.h"

void test_1(){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
}
