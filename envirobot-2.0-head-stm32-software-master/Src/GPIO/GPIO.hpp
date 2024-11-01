/*
 * GPIO.hpp
 *
 *  Created on: Dec 5, 2022
 *      Author: bignet
 */

#pragma once

#include <stm32h750xx.h>
#include <stm32h7xx_hal.h>

struct GPIO {
	GPIO_TypeDef* port;
	uint16_t pin;
};
