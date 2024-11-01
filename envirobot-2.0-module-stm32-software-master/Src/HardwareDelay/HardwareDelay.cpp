/*
 * HardwareDelay.cpp
 *
 *  Created on: Dec 19, 2022
 *      Author: bignet
 */

#include <HardwareDelay/HardwareDelay.hpp>

/**
 * @brief Class constructor
 *
 * @param input timer_: the timer pointer used to generate interrupts
 */
HardwareDelay::HardwareDelay(TIM_HandleTypeDef* timer_) {
	timer = timer_;

	pending_delays.reserve(DELAY_NUMBER_OF_CHANNELS);
}

/**
 * @brief Initialize the class
 */
void HardwareDelay::Init(void) {
	DelaySemaphore = osSemaphoreNew(1, 1, NULL);
	osSemaphoreRelease(DelaySemaphore);
}

/**
 * @brief Check that the input timer is registered in the class.
 *
 * @param input timer_: the timer pointer to check
 * @return true if input timer is used by the class
 */
bool HardwareDelay::AssignedTimer(TIM_HandleTypeDef* timer_) {
	osSemaphoreAcquire(DelaySemaphore, osWaitForever);
	bool result = timer == timer_;
	osSemaphoreRelease(DelaySemaphore);
	return result;
}

/**
 * @brief Add a delay in ms. Once the time has elapsed, the callback is summoned with the given context
 *
 * @param input ID: the delay ID
 * @param input delay_ms: the delay value
 * @param input delay_ms: the context pointer to forward to the callback when summoned
 * @param input callback: the callback function to call after delay. Take the delay ID as input.
 * @return whether successful
 */
bool HardwareDelay::AddDelayMS(uint8_t ID, uint16_t delay_ms, void* context, void (*callback)(void*, uint8_t)) {
	osSemaphoreAcquire(DelaySemaphore, osWaitForever);
	// Check there is still some space
	if (pending_delays.size() >= DELAY_NUMBER_OF_CHANNELS) {
		osSemaphoreRelease(DelaySemaphore);
		return false;
	}

	// Check the delay ID is not already used
	bool success;
	FindDelayIndex(ID, &success);
	if (success) {
		osSemaphoreRelease(DelaySemaphore);
		return false;
	}

	HAL_TIM_Base_Stop_IT(timer);

	// Setup the delay
	Delay hardware_delay;
	hardware_delay.ID = ID;
	hardware_delay.delay = delay_ms << 2; // * 4 to get delay in 1/4 ms
	hardware_delay.context = context;
	hardware_delay.callback = callback;

	if (pending_delays.size() > 0) {
		// If delays already pending, check when the new one will trigger in the chain
		uint16_t remaining_time = timer->Instance->ARR - timer->Instance->CNT;
		if (hardware_delay.delay < remaining_time) {
			// If the next delay to trigger is the new one, then update every registered delay, select the new one as active
			for (uint8_t index = 0; index < pending_delays.size(); index++) {
				if (index == active_delay)
					pending_delays[index].delay -= timer->Instance->CNT + hardware_delay.delay;
				else
					pending_delays[index].delay += remaining_time - hardware_delay.delay;
			}
			active_delay = pending_delays.size();
		} else {
			// If the next delay to trigger is not the new one, simply register the new delay
			hardware_delay.delay -= remaining_time;
			HAL_TIM_Base_Start_IT(timer);
			pending_delays.push_back(hardware_delay);
			osSemaphoreRelease(DelaySemaphore);
			return true;
		}
	} else {
		active_delay = 0;
	}

	// Push the new delay
	pending_delays.push_back(hardware_delay);

	// Start the delay
	StartDelay();
	osSemaphoreRelease(DelaySemaphore);
	return true;
}

/**
 * @brief Remove a delay from the registered delays
 *
 * @param input ID: the delay ID to remove
 * @return whether successful
 */
bool HardwareDelay::RemoveDelayMS(uint8_t ID) {
	osSemaphoreAcquire(DelaySemaphore, osWaitForever);
	bool success;
	// Find the delay based on the ID
	uint8_t index = FindDelayIndex(ID, &success);
	if (!success) {
		osSemaphoreRelease(DelaySemaphore);
		return false;
	}

	// Stop timer
	HAL_TIM_Base_Stop_IT(timer);
	pending_delays.erase(pending_delays.begin() + index);

	// Update other registered delays
	if (active_delay == index) {
		uint16_t remaining_time = timer->Instance->ARR - timer->Instance->CNT;
		for (uint8_t index = 0; index < pending_delays.size(); index++) {
			pending_delays[index].delay += remaining_time;
		}
		// Start a new delay if the removed one was the active delay
		StartNextDelay();
	} else {
		// Resume if the active delay is not the removed one
		if (index < active_delay) active_delay--;
		HAL_TIM_Base_Start_IT(timer);
	}
	osSemaphoreRelease(DelaySemaphore);
	return true;
}

/**
 * @brief Check if the input delay ID is registered
 *
 * @param input ID: the delay ID to check
 * @return true if the ID is found
 */
bool HardwareDelay::IsDelayRegistered(uint8_t ID) {
	bool success;
	osSemaphoreAcquire(DelaySemaphore, osWaitForever);
	FindDelayIndex(ID, &success);
	osSemaphoreRelease(DelaySemaphore);
	return success;
}

/**
 * @brief Setup and start the timer based on the active delay
 */
void HardwareDelay::StartDelay(void) {
	timer->Instance->CNT = 0;
	timer->Instance->ARR = pending_delays[active_delay].delay;
	timer->Instance->SR = ~TIM_IT_UPDATE;
	HAL_TIM_Base_Start_IT(timer);
}

/**
 * @brief Active and start the next delay
 */
void HardwareDelay::StartNextDelay(void) {
	if (pending_delays.size() == 0) return;

	// Find the smallest delay in memory
	uint8_t min_delay = 0;
	for (uint8_t index = 1; index < pending_delays.size(); index++) {
		if (pending_delays[index].delay < pending_delays[min_delay].delay)
			min_delay = index;
	}

	// Update other delays based on the active one
	active_delay = min_delay;
	for (uint8_t index = 0; index < pending_delays.size(); index++) {
		if (index != active_delay) {
			pending_delays[index].delay -= pending_delays[active_delay].delay;
		}
	}

	// Start the delay
	StartDelay();
}

/**
 * @brief ISR called by the timer. Summons the callback then start the next delay
 */
void HardwareDelay::TriggeredDelay(void) {
	HAL_TIM_Base_Stop_IT(timer);

	osSemaphoreAcquire(DelaySemaphore, osWaitForever);
	timer->Instance->SR = ~TIM_IT_UPDATE;

	// Summon callback
	if (pending_delays[active_delay].callback != NULL) {
		pending_delays[active_delay].callback(pending_delays[active_delay].context, pending_delays[active_delay].ID);
	}

	// Remove delay
	pending_delays.erase(pending_delays.begin() + active_delay);

	// Stat next delay
	StartNextDelay();
	osSemaphoreRelease(DelaySemaphore);
}

/**
 * @brief Find the vector index based on the delay ID
 *
 * @param input ID: the delay ID to be found
 * @param output success: true if the delay is found
 * @return the delay index in the delay vector
 */
uint8_t HardwareDelay::FindDelayIndex(uint8_t ID, bool* success) {
	uint8_t index = 0;
	for (Delay delay : pending_delays) {
		if (delay.ID == ID) {
			*success = true;
			return index;
		}
		index++;
	}
	*success = false;
	return 0;
}
