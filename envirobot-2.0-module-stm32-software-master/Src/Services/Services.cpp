/*
 * Services.cpp
 *
 *  Created on: 9 déc. 2022
 *      Author: bignet
 */

#include <Services/Services.hpp>

/**
 * @brief Class constructor
 */
Services::Services() {
	ResetQueue();
}

/**
 * @brief Initialize the class with references to other classes. Set default values.
 *
 * @param input registers_: the Registers instance
 * @param input communication_: the Communication instance
 * @param input leds_: the LEDS instance
 */
void Services::Init(Registers* registers_, Communication* communication_, LEDS* leds_) {
	registers = registers_;
	communication = communication_;
	leds = leds_;

	ServiceSemaphore = osSemaphoreNew(1,1,NULL);
	osSemaphoreRelease(ServiceSemaphore);
}

/**
 * @brief Reset the pending services
 */
void Services::ResetQueue(void) {
	queue_size = 0;
	osSemaphoreAcquire(ServiceSemaphore, osWaitForever);
	for (uint8_t index = 0; index < NUMBER_OF_SERVICES; index++) {
		services[index].active = false;
		services[index].reception_flag = false;
		services[index].error = false;
	}
	osSemaphoreRelease(ServiceSemaphore);
}

/**
 * @brief Find the pending service index with the correct register and interface pattern
 *
 * @param input start_index: start index in the service list to start the research
 * @param input remote_register: the register information used to find a service with the same data
 * @param input interface: the interface from which the data comes from, used to find the service
 * @param input access: whether WRITE or READ
 * @param output success: whether successful
 * @return service index
 */
uint8_t Services::FindQueueIndex(uint8_t start_index, Register remote_register, ServiceInterface interface, bool access, bool* success) {
	osSemaphoreAcquire(ServiceSemaphore, osWaitForever);
	for (uint8_t index = start_index; index < NUMBER_OF_SERVICES; index++) {
		if (services[index].active) {
			Register registered_register = services[index].configuration.register_;
			ServiceInterface registered_interface =services[index].configuration.interface;

			// Check register address
			if (remote_register.address == registered_register.address) {
				// Check interface
				if (interface.interface == registered_interface.interface) {
					// Check device address that ack the service
					if (interface.address == registered_interface.address || registered_interface.address == ALL || interface.address == ALL) {
						if (access == services[index].configuration.access) { // check access type
							osSemaphoreRelease(ServiceSemaphore);
							*success = true;
							return index;
						}
					}
				}
			}
		}
	}
	osSemaphoreRelease(ServiceSemaphore);
	*success = false;
	return 0;
}

/**
 * @brief Test if the service queue can welcome a new pending service. It can have different configuration for the same register ID as soon as it has a different interface, different module address (or ALL) or different access type
 *
 * @param input remote_register: the register information to remotely access
 * @param input interface: the interface from which the remote access will be processed
 * @param input access: whether WRITE or READ
 * @return whether successful
 */
bool Services::QueueAvailable(Register remote_register, ServiceInterface interface, bool access) {
	if (queue_size >= NUMBER_OF_SERVICES) return false;

	osSemaphoreAcquire(ServiceSemaphore, osWaitForever);
	for (uint8_t index = 0; index < NUMBER_OF_SERVICES; index++) {
		if (services[index].active) {
			Register registered_register = services[index].configuration.register_;
			ServiceInterface registered_interface =services[index].configuration.interface;

			// Check register address
			if (remote_register.address == registered_register.address) {
				// Check interface
				if (interface.interface == registered_interface.interface) {
					// Check device address that ack the service
					if (interface.address == registered_interface.address || registered_interface.address == ALL || remote_register.address == ALL) {
						// Check there is no other service with the same access mode
						if (access == services[index].configuration.access) {
							// Check there is no other register with the same ID and interface
							if (remote_register.type != registered_register.type || remote_register.isArray != registered_register.isArray || (remote_register.isArray && (remote_register.length != registered_register.length))) {
								osSemaphoreRelease(ServiceSemaphore);
								return false;
							}
						}
					}
				}
			}
		}
	}
	osSemaphoreRelease(ServiceSemaphore);
	return true;
}

/**
 * @brief Find a free index in the service queue
 *
 * @param output success: whether successful
 * @return a free queue index
 */
uint8_t Services::FindAvailableIndex(bool* success) {

	for (uint8_t index = 0; index < NUMBER_OF_SERVICES; index++) {
		if (not services[index].active) {
			*success = true;
			return index;
		}
	}

	*success = false;
	return 0;
}

/**
 * @brief Add a service in queue with an associated register and interface
 *
 * @param input remote_register: the register information related to the service
 * @param input interface: the interface used by the service
 * @param input access: access mode of the service
 * @param input timeout: time left to terminate the service procedure.
 * @param output success: whether successful
 * @return the service index
 */
uint8_t Services::AddToQueue(Register remote_register, ServiceInterface interface, bool access, uint16_t timeout, bool* success) {
	osSemaphoreAcquire(ServiceSemaphore, osWaitForever);

	// Find a free index
	uint8_t index = FindAvailableIndex(success);
	if (not *success) {
		osSemaphoreRelease(ServiceSemaphore);
		*success = false;
		return 0;
	}

	// Setup the service
	queue_size++;
	services[index].reception_flag = false;
	services[index].active = true;
	services[index].configuration.register_ = remote_register;
	services[index].configuration.interface = interface;
	services[index].configuration.access = access;
	services[index].timeout = timeout;
	services[index].timeout_timestamp = HAL_GetTick() + timeout;
	services[index].error = false;

	osSemaphoreRelease(ServiceSemaphore);
	*success = true;
	return index;
}

/**
 * @brief Remove a service from queue
 *
 * @param input index: sevice index in queue
 * @return whether successful
 */
bool Services::RemoveFromQueue(uint8_t index) {
	if (index >= NUMBER_OF_SERVICES) return false;

	osSemaphoreAcquire(ServiceSemaphore, osWaitForever);
	queue_size--;
	services[index].active = false;
	services[index].reception_flag = false;
	osSemaphoreRelease(ServiceSemaphore);
	return true;
}

/**
 * @brief Receive a write service access acknowledgment
 *
 * @param input information: service information associated to the write acknowledgment
 * @param input status: value returned by the remote module
 */
void Services::ReceiveWrite(ServiceConfiguration information, uint8_t status) {
	bool success = true;
	uint8_t start_index = 0;
	while (success) {
		uint8_t index = FindQueueIndex(start_index, information.register_, information.interface, information.access, &success);
		if (not success) return;

		osSemaphoreAcquire(ServiceSemaphore, osWaitForever);
		buffer_uint8[index][0] = status;
		services[index].reception_flag = true;
		osSemaphoreRelease(ServiceSemaphore);

		start_index = index + 1;
	}
}

/**
 * @brief Receive an service access error
 *
 * @param input information: service information associated to the acknowledgment
 * @param input error: error flag returned by the remote module
 */
void Services::ReceiveError(ServiceConfiguration information, uint8_t error) {
	bool success = true;
	uint8_t start_index = 0;
	while (success) {
		uint8_t index = FindQueueIndex(start_index, information.register_, information.interface, information.access, &success);
		if (not success) return;

		osSemaphoreAcquire(ServiceSemaphore, osWaitForever);
		buffer_uint8[index][0] = error;
		services[index].error = true;
		services[index].reception_flag = true;
		osSemaphoreRelease(ServiceSemaphore);

		start_index = index + 1;
	}
}

/**
 * @brief Find a register configuration from pending services
 *
 * @param input remote_register: provides only the register address to look at
 * @param input interface: interface associated to the register to find
 * @param input access: access mode of the register to find
 * @param output success: whether successful
 * @return the Register complete configuration (type, isArray, length)
 */
Register Services::FindRemoteRegister(Register remote_register, ServiceInterface interface, bool access, bool* success) {
	uint8_t index;
	index = FindQueueIndex(0, remote_register, interface, access, success);
	if (*success) {
		osSemaphoreAcquire(ServiceSemaphore, osWaitForever);
		Register found_register = services[index].configuration.register_;
		osSemaphoreRelease(ServiceSemaphore);

		return found_register;
	}

	*success = false;
	return Register();
}
