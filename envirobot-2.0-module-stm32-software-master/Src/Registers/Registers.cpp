/*
 * Registers.cpp
 *
 *  Created on: 29 nov. 2022
 *      Author: bignet
 */

#include <Registers/Registers.hpp>

/**
 * @brief Class constructor
 */
Registers::Registers() {
	callbacks[UINT8_TYPE] = NULL;
	callbacks[UINT16_TYPE] = NULL;
	callbacks[UINT32_TYPE] = NULL;
	callbacks[UINT64_TYPE] = NULL;
	callbacks[INT8_TYPE] = NULL;
	callbacks[INT16_TYPE] = NULL;
	callbacks[INT32_TYPE] = NULL;
	callbacks[INT64_TYPE] = NULL;
	callbacks[FLOAT_TYPE] = NULL;
	callbacks[DOUBLE_TYPE] = NULL;

#ifdef USE_UINT8_REGISTER
	callbacks[UINT8_TYPE] = &uint8_t_callbacks;
#endif
#ifdef USE_UINT16_REGISTER
	callbacks[UINT16_TYPE] = &uint16_t_callbacks;
#endif
#ifdef USE_UINT32_REGISTER
	callbacks[UINT32_TYPE] = &uint32_t_callbacks;
#endif
#ifdef USE_UINT64_REGISTER
	callbacks[UINT64_TYPE] = &uint64_t_callbacks;
#endif
#ifdef USE_INT8_REGISTER
	callbacks[INT8_TYPE] = &int8_t_callbacks;
#endif
#ifdef USE_INT16_REGISTER
	callbacks[INT16_TYPE] = &int16_t_callbacks;
#endif
#ifdef USE_INT32_REGISTER
	callbacks[INT32_TYPE] = &int32_t_callbacks;
#endif
#ifdef USE_INT64_REGISTER
	callbacks[INT64_TYPE] = &int64_t_callbacks;
#endif
#ifdef USE_FLOAT_REGISTER
	callbacks[FLOAT_TYPE] = &float_callbacks;
#endif
#ifdef USE_DOUBLE_REGISTER
	callbacks[DOUBLE_TYPE] = &double_callbacks;
#endif
}

/**
 * @brief Initialize the class
 */
void Registers::Init(void) {
	RegistersSemaphore = osSemaphoreNew(1,1,NULL);
	osSemaphoreRelease(RegistersSemaphore);
}

/**
 * @brief Add class related registers
 */
void Registers::AddBaseRegisters(void) {
	// Register to get the timestamp in ms
	AddRegister<uint32_t>(REG_TIMEBASE);
	SetRegisterAsSingle(REG_TIMEBASE);
	AddReadCallback<uint32_t>(REG_TIMEBASE, NULL,
		[](void* context, uint16_t register_id, uint32_t** output, uint16_t* length) -> bool {
			*length = 1;
			**output = HAL_GetTick();
			return true;
		}
	);
}

/**
 * @brief Find a register from an ID/address
 *
 * @param input ID: register address
 * @param output success: whether successful
 * @return the a copy of the register information
 */
Register Registers::FindRegister(uint16_t ID, bool* success) {
	RegisterConfiguration* register_configuration = FindRegisterConfiguration(ID, success);
	return register_configuration->register_info;
}

/**
 * @brief Find a register configuration from an ID/address
 *
 * @param input ID: register address
 * @param output success: whether successful
 * @return the pointer to the register configuration
 */
RegisterConfiguration* Registers::FindRegisterConfiguration(uint16_t ID, bool* success) {
	for (RegisterConfiguration &register_configuration : registers) {
		if (register_configuration.register_info.address == ID) {
			*success = true;
			return &register_configuration;
		}
	}
	*success = false;
	return NULL;
}

/**
 * @brief Link a register to a semaphore. At each register access, the semaphore will be taken then released
 *
 * @param input ID: register address
 * @param input register_semaphore: pointer to the semaphore to link
 * @return whether successful
 */
bool Registers::AddRegisterSemaphore(uint16_t ID, osSemaphoreId_t* register_semaphore) {
	bool success;
	osSemaphoreAcquire(RegistersSemaphore, osWaitForever);
	RegisterConfiguration* target_register = FindRegisterConfiguration(ID, &success);
	if (!success) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	target_register->register_semaphore = register_semaphore;
	osSemaphoreRelease(RegistersSemaphore);
	return true;
}

/**
 * @brief Group the callbacks of a base register to another register
 *
 * @param input ID: register address
 * @param input base_ID: base register address whose callbacks will be also assigned to the new register
 * @return whether successful
 */
bool Registers::GroupRegisterCallbacks(uint16_t ID, uint16_t base_ID) {
	bool success;
	osSemaphoreAcquire(RegistersSemaphore, osWaitForever);
	RegisterConfiguration* target_register = FindRegisterConfiguration(ID, &success);
	RegisterConfiguration* base_register = FindRegisterConfiguration(base_ID, &success);
	if (!success) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	if (not base_register->mode.callbacks) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	target_register->mode.callbacks = true; // = CALLBACK_MODE;
	target_register->callbacks_ID = base_register->callbacks_ID;
	target_register->access = base_register->access;
	osSemaphoreRelease(RegistersSemaphore);
	return true;
}

/**
 * @brief Set the register as a single value (not an array nor a vector)
 *
 * @param input ID: register address
 * @return whether successful
 */
bool Registers::SetRegisterAsSingle(uint16_t ID) {
	bool success;
	osSemaphoreAcquire(RegistersSemaphore, osWaitForever);
	RegisterConfiguration* target_register = FindRegisterConfiguration(ID, &success);
	if (!success) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	target_register->register_info.isArray = false;
	target_register->register_info.length = 1;
	osSemaphoreRelease(RegistersSemaphore);
	return true;
}

/**
 * @brief Set the register as an array
 *
 * @param input ID: register address
 * @param input length_: length of the array
 * @return whether successful
 */
bool Registers::SetRegisterAsArray(uint16_t ID, uint16_t length_) {
	bool success;
	osSemaphoreAcquire(RegistersSemaphore, osWaitForever);
	RegisterConfiguration* target_register = FindRegisterConfiguration(ID, &success);
	if (!success) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	target_register->register_info.isArray = true;
	target_register->register_info.length = length_;
	osSemaphoreRelease(RegistersSemaphore);
	return true;
}

/**
 * @brief Set the register as a vector with no fixed length
 *
 * @param input ID: register address
 * @return whether successful
 */
bool Registers::SetRegisterAsVector(uint16_t ID) {
	return SetRegisterAsArray(ID, 0);
}

/**
 * @brief Set the register access permissions
 *
 * @param input ID: register address
 * @param input permissions_: WRITE_PERMISSION or READ_PERMISSION
 * @return whether successful
 */
bool Registers::SetRegisterPermissions(uint16_t ID, uint8_t permissions_) {
	bool success;
	osSemaphoreAcquire(RegistersSemaphore, osWaitForever);
	RegisterConfiguration* target_register = FindRegisterConfiguration(ID, &success);
	if (!success) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	target_register->access.write = permissions_ & WRITE_PERMISSION;
	target_register->access.read = permissions_ & READ_PERMISSION;
	osSemaphoreRelease(RegistersSemaphore);
	return true;
}

/**
 * @brief Check the register exists
 *
 * @param input ID: register address to check
 * @return true if the register is found
 */
bool Registers::IsRegistered(uint16_t ID) {
	bool success;
	osSemaphoreAcquire(RegistersSemaphore, osWaitForever);
	FindRegisterConfiguration(ID, &success);
	osSemaphoreRelease(RegistersSemaphore);
	return success;
}

/**
 * @brief Get the size of the input type
 *
 * @param input type: the type ID
 * @return the size in byte of the input type
 */
uint8_t Registers::GetTypeSize(uint8_t type) {
	if (type == UINT8_TYPE || type == INT8_TYPE)
		return 1;
	if (type == UINT16_TYPE || type == INT16_TYPE)
		return 2;
	if (type == UINT32_TYPE || type == INT32_TYPE || type == FLOAT_TYPE)
		return 4;
	if (type == UINT64_TYPE || type == INT64_TYPE || type == DOUBLE_TYPE)
		return 8;

	return 0;
}
