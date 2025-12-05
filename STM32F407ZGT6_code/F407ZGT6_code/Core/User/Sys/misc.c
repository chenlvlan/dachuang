/*
 * misc.c
 *
 *  Created on: Dec 5, 2025
 *      Author: yufei
 */

#include "misc.h"

void HVHP(bool isEN) {
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, (GPIO_PinState)isEN);
}

