/*
 * sensorDriver.c
 *
 *  Created on: Mar 25, 2025
 *      Author: chris
 */


#include <stdbool.h>
#include "stm32f4xx_hal.h"

#include "IR_driver.h"
#include "main.h"


int wallLeft() {
	return !HAL_GPIO_ReadPin(LEFT_IR_GPIO_Port, LEFT_IR_Pin);
}

int wallRight() {
	return !HAL_GPIO_ReadPin(RIGHT_IR_GPIO_Port, RIGHT_IR_Pin);
}

int wallFront() {
	return !HAL_GPIO_ReadPin(FRONT_IR_GPIO_Port, FRONT_IR_Pin);
}

