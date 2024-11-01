/*
 * HardwareDelay.hpp
 *
 *  Created on: Dec 19, 2022
 *      Author: bignet
 */

#pragma once

#define DELAY_NUMBER_OF_CHANNELS 3

#include <vector>
#include <stm32h750xx.h>
#include <stm32h7xx_hal.h>
#include "cmsis_os.h"

// Delay struct
struct Delay {
	uint8_t ID;							// delay ID
	uint16_t delay; 					// delay in 1/4 ms (translated to 1ms when calling AddDelayMS)
	void* context;						// context to forward to the callback after delay
	void (*callback)(void*, uint8_t);	// callback function to summon after the delay
};

// HardwareDelay class used to trigger timer interruptions to process callbacks in interruptions
class HardwareDelay {
public:
	HardwareDelay(TIM_HandleTypeDef* timer);
	void Init(void);

	bool AssignedTimer(TIM_HandleTypeDef* timer);
	bool AddDelayMS(uint8_t ID, uint16_t delay_ms, void* context = NULL, void (*callback)(void*, uint8_t) = NULL);
	bool RemoveDelayMS(uint8_t ID);
	bool IsDelayRegistered(uint8_t ID);
	void TriggeredDelay(void);
private:
	void StartDelay(void);
	void StartNextDelay(void);

	uint8_t FindDelayIndex(uint8_t ID, bool* success);

	// OS
	osSemaphoreId_t DelaySemaphore;

	TIM_HandleTypeDef* timer;
	I2C_HandleTypeDef* interface;

	uint8_t active_delay;
	std::vector<Delay> pending_delays;
};
