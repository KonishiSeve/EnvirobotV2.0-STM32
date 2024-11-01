/*
 * Registers.hpp
 *
 *  Created on: 29 nov. 2022
 *      Author: bignet
 */

#pragma once

#include <vector>
#include <stm32h750xx.h>
#include <stm32h7xx_hal.h>
#include "cmsis_os.h"
#include <typeinfo>

#include "Configurations/RegistersConfiguration.h"
#include "Definitions/RegistersDefinition.h"
#include "RegisterMaps/RegisterMapRegisters.h"

// MACROS
#define __WRITE_CALLBACK(__CONTEXT__, __REGISTER__ ,__TYPE__, __INPUT__, __LENGTH__, __LOGIC__) 			[](void* __CONTEXT__, uint16_t __REGISTER__, __TYPE__* __INPUT__, uint16_t __LENGTH__) -> bool {__LOGIC__}
#define __READ_CALLBACK(__CONTEXT__, __REGISTER__, __TYPE__, __OUTPUT__, __LENGTH__, __LOGIC__) 			[](void* __CONTEXT__, uint16_t __REGISTER__, __TYPE__** __OUTPUT__, uint16_t* __LENGTH__) -> bool {__LOGIC__}
#define __GET_CONTEXT(__CONTEXT_TYPE__, __BASE_CONTEXT__, __NEW_CONTEXT__)									__CONTEXT_TYPE__* __NEW_CONTEXT__ = (__CONTEXT_TYPE__*) __BASE_CONTEXT__

typedef class Registers;

// Mode struct
// NOTE: a register can be both pointer and callback for writing access but not for read access
struct Mode {
	bool pointer;			// true = the register is linked to a pointer
	bool callbacks;			// true = the register is linked to read and/or write callbacks
};

// Permissions struct
struct Permissions {
	bool write = true;		// true = write access authorized
	bool read = true;		// true = read access authorized
};

// Register struct
struct Register {
	uint16_t address;		// register address
	uint8_t type;			// register type ID
	bool isArray;			// false = register singleton
	uint16_t length;		// length of the register if not a singleton. A 0 length with isArray==true means a vector with unknown length
};

// RegisterConfiguration struct
struct RegisterConfiguration {
	Mode mode;								// register mode
	Register register_info;					// register information
	Permissions access;						// register permissions
	uint16_t callbacks_ID;					// callbacks/pointer ID
	osSemaphoreId_t* register_semaphore;	// linked semaphore
};

// RegisterCallbacks struct
template <typename T>
struct RegisterCallbacks {
	T* pointer;										// memory pointer linked to the register
	void* context;									// context to forward to the callbacks
	bool (*write)(void*, uint16_t, T*, uint16_t);	// write callback
	bool (*read)(void*, uint16_t, T**, uint16_t*);	// read callback
};

// Registers class used to interface memory addresses to memory pointers and / or callbacks
class Registers {
public:
	Registers();
	void Init(void);
	void AddBaseRegisters(void);

	template <typename T>
	bool AddRegister(uint16_t ID);

//	bool RemoveRegister(uint16_t ID); // TODO

	template <typename T>
	bool AddRegisterPointer(uint16_t ID, T* pointer);

	template <typename T>
	bool AddVectorRegisterPointer(uint16_t ID, std::vector<T>* pointer);

	bool AddRegisterSemaphore(uint16_t ID, osSemaphoreId_t* register_semaphore);

//	bool RemoveRegisterPointer(uint16_t ID); // TODO

	template <typename T>
	bool AddWriteCallback(uint16_t ID, void* context, bool (*write)(void*, uint16_t, T*, uint16_t));

//	bool RemoveWriteCallback(uint16_t ID); // TODO

	template <typename T>
	bool AddReadCallback(uint16_t ID, void* context, bool (*read)(void*, uint16_t, T**, uint16_t*));

//	bool RemoveReadCallback(uint16_t ID); // TODO

	template <typename T>
	bool AddRegisterCallbacks(uint16_t ID, void* context, bool (*write)(void*, uint16_t, T*, uint16_t), bool (*read)(void*, uint16_t, T**, uint16_t*));

//	bool RemoveRegisterCallbacks(uint16_t ID); // TODO

	template <typename T>
	bool WriteRegister(uint16_t ID, T* value, uint16_t length = 0, bool use_semaphore = true);

	template <typename T>
	bool WriteVectorRegister(uint16_t ID, std::vector<T> value, bool use_semaphore = true);

	template <typename T>
	bool ReadRegister(uint16_t ID, T* output, uint16_t* length, bool use_semaphore = true);

	template <typename T>
	bool ReadVectorRegister(uint16_t ID, std::vector<T>* output, bool use_semaphore = true);

	bool IsRegistered(uint16_t ID);
	Register FindRegister(uint16_t ID, bool* success); // only returns a copy of the register for read-only purpose

	template <typename T>
	uint8_t GetTypeID (bool* success);
	uint8_t GetTypeSize(uint8_t type);

	bool GroupRegisterCallbacks(uint16_t ID, uint16_t base_ID);
	bool SetRegisterAsSingle(uint16_t ID);
	bool SetRegisterAsArray(uint16_t ID, uint16_t length_);
	bool SetRegisterAsVector(uint16_t ID);
	bool SetRegisterPermissions(uint16_t ID, uint8_t permissions_);

private:
	// OS
	osSemaphoreId_t RegistersSemaphore;

	RegisterConfiguration* FindRegisterConfiguration(uint16_t ID, bool* success);

	std::vector<RegisterConfiguration> registers;
	void* callbacks[NUMBER_OF_TYPES];

#ifdef USE_UINT8_REGISTER
	std::vector<RegisterCallbacks<uint8_t>> uint8_t_callbacks;
#endif
#ifdef USE_UINT16_REGISTER
	std::vector<RegisterCallbacks<uint16_t>> uint16_t_callbacks;
#endif
#ifdef USE_UINT32_REGISTER
	std::vector<RegisterCallbacks<uint32_t>> uint32_t_callbacks;
#endif
#ifdef USE_UINT64_REGISTER
	std::vector<RegisterCallbacks<uint64_t>> uint64_t_callbacks;
#endif
#ifdef USE_INT8_REGISTER
	std::vector<RegisterCallbacks<int8_t>> int8_t_callbacks;
#endif
#ifdef USE_INT16_REGISTER
	std::vector<RegisterCallbacks<int16_t>> int16_t_callbacks;
#endif
#ifdef USE_INT32_REGISTER
	std::vector<RegisterCallbacks<int32_t>> int32_t_callbacks;
#endif
#ifdef USE_INT64_REGISTER
	std::vector<RegisterCallbacks<int64_t>> int64_t_callbacks;
#endif
#ifdef USE_FLOAT_REGISTER
	std::vector<RegisterCallbacks<float>> float_callbacks;
#endif
#ifdef USE_DOUBLE_REGISTER
	std::vector<RegisterCallbacks<double>> double_callbacks;
#endif
};

/**
 * @brief Add a typed register
 *
 * @param input ID: the register address
 * @return whether successful
 */
template <typename T>
bool Registers::AddRegister(uint16_t ID) {
	bool success;

	// Check the register doesn't already exist
	osSemaphoreAcquire(RegistersSemaphore, osWaitForever);
	FindRegisterConfiguration(ID, &success);
	if (success) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Setup register
	RegisterConfiguration configuration_;
	Register register_;
	Permissions permissions_;
	Mode mode;
	mode.pointer = false;
	mode.callbacks = false;
	configuration_.mode = mode;
	register_.address = ID;
	register_.type = GetTypeID<T>(&success); 	// check that the type is supported
	register_.isArray = false;
	register_.length = 1;
	configuration_.register_info = register_;
	permissions_.write = false;
	permissions_.read = false;
	configuration_.access = permissions_;
	configuration_.callbacks_ID = 0;
	configuration_.register_semaphore = NULL;

	if (success) {
		registers.push_back(configuration_);
	}

	osSemaphoreRelease(RegistersSemaphore);
	return success;
}

/**
 * @brief Get the type ID of the template type T
 *
 * @param output success: whether successful
 * @return the type ID corresponding to the input template type
 */
template <typename T>
uint8_t Registers::GetTypeID (bool* success) {
	uint8_t type = 0;

	if (false) {}
#ifdef USE_UINT8_REGISTER
	else if (typeid(T) == typeid(uint8_t)) 		type = UINT8_TYPE;
#endif
#ifdef USE_UINT16_REGISTER
	else if (typeid(T) == typeid(uint16_t)) 	type = UINT16_TYPE;
#endif
#ifdef USE_UINT32_REGISTER
	else if (typeid(T) == typeid(uint32_t)) 	type = UINT32_TYPE;
#endif
#ifdef USE_UINT64_REGISTER
	else if (typeid(T) == typeid(uint64_t))	 	type = UINT64_TYPE;
#endif
#ifdef USE_INT8_REGISTER
	else if (typeid(T) == typeid(int8_t)) 		type = INT8_TYPE;
#endif
#ifdef USE_INT16_REGISTER
	else if (typeid(T) == typeid(int16_t)) 		type = INT16_TYPE;
#endif
#ifdef USE_INT32_REGISTER
	else if (typeid(T) == typeid(int32_t)) 		type = INT32_TYPE;
#endif
#ifdef USE_INT64_REGISTER
	else if (typeid(T) == typeid(int64_t)) 		type = INT64_TYPE;
#endif
#ifdef USE_FLOAT_REGISTER
	else if (typeid(T) == typeid(float)) 		type = FLOAT_TYPE;
#endif
#ifdef USE_DOUBLE_REGISTER
	else if (typeid(T) == typeid(double)) 		type = DOUBLE_TYPE;
#endif
	else {*success = false; return 0;}

	*success = true;
	return type;
}

/**
 * @brief Link a pointer to a register
 *
 * @param input ID: register address
 * @param input pointer: the memory pointer to link to the register
 * @return whether successful
 */
template <typename T>
bool Registers::AddRegisterPointer(uint16_t ID, T* pointer) {
	bool success;
	osSemaphoreAcquire(RegistersSemaphore, osWaitForever);
	// Find register
	RegisterConfiguration* target_register = FindRegisterConfiguration(ID, &success);
	if (!success) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Get the type ID
	uint8_t registered_type = GetTypeID<T>(&success);
	if (!success) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Check the template type and register type
	if (target_register->register_info.type != registered_type) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Check the type is supported
	if (callbacks[target_register->register_info.type] == NULL) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Get the typed callback list
	std::vector<RegisterCallbacks<T>>* target_callback_list;
	target_callback_list = reinterpret_cast <std::vector<RegisterCallbacks<T>>*> (callbacks[target_register->register_info.type]);

	// Add the pointer to the register
	if (not target_register->mode.pointer &&  not target_register->mode.callbacks) {
		RegisterCallbacks<T> callback;
		callback.pointer = pointer;
		target_register->callbacks_ID = (uint16_t) target_callback_list->size();
		target_callback_list->push_back(callback);
	} else {
		RegisterCallbacks<T>* callback = &(target_callback_list->at(target_register->callbacks_ID));
		callback->pointer = pointer;
	}

	// Set register mode and permissions
	target_register->mode.pointer = true;
	target_register->access.write = true;
	target_register->access.read = true;

	osSemaphoreRelease(RegistersSemaphore);
	return true;
}

/**
 * @brief Link a vector pointer to a register
 *
 * @param input ID: register address
 * @param input pointer: the memory pointer of vector to link to the register
 * @return whether successful
 */
template <typename T>
bool Registers::AddVectorRegisterPointer(uint16_t ID, std::vector<T>* pointer) {
	T* reinterpreted_pointer = reinterpret_cast<T*>(pointer);
	return AddRegisterPointer(ID, reinterpreted_pointer);
}

/**
 * @brief Link a write callback to a register
 *
 * @param input ID: register address
 * @param input context: the context pointer to forward to the callback
 * @param input write: the write callback to link to the register
 * 		@param input void* context: the context pointer
 * 		@param input uint16_t register_ID: the register address that summoned the write callback
 * 		@param input T* input: the input data
 * 		@param input uint16_t length: the size of the input data
 * @return whether successful
 */
template <typename T>
bool Registers::AddWriteCallback(uint16_t ID, void* context, bool (*write)(void*, uint16_t, T*, uint16_t)) {
	bool success;
	osSemaphoreAcquire(RegistersSemaphore, osWaitForever);
	// Find register
	RegisterConfiguration* target_register = FindRegisterConfiguration(ID, &success);
	if (!success) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Get the type ID
	uint8_t registered_type = GetTypeID<T>(&success);
	if (!success) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Check template type same as register type
	if (target_register->register_info.type != registered_type) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Check the type is supported
	if (callbacks[target_register->register_info.type] == NULL) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Get the typed callback list
	std::vector<RegisterCallbacks<T>>* target_callback_list;
	target_callback_list = reinterpret_cast <std::vector<RegisterCallbacks<T>>*> (callbacks[target_register->register_info.type]);

	// Add the write callback to the register
	if (not target_register->mode.pointer &&  not target_register->mode.callbacks) { // if (target_register->mode. == NONE_MODE) {
		RegisterCallbacks<T> callback;
		callback.context = context;
		callback.write = write;
		callback.read = NULL;
		target_register->callbacks_ID = (uint16_t) target_callback_list->size();
		target_callback_list->push_back(callback);
	} else {
		RegisterCallbacks<T>* callback = &(target_callback_list->at(target_register->callbacks_ID));
		callback->context = context;
		callback->write = write;
	}

	// Set register mode and permissions
	target_register->mode.callbacks = true;
	target_register->access.write = true;

	osSemaphoreRelease(RegistersSemaphore);
	return true;
}

/**
 * @brief Link a read callback to a register
 *
 * @param input ID: register address
 * @param input context: the context pointer to forward to the callback
 * @param input read: the read callback to link to the register
 * 		@param input void* context: the context pointer
 * 		@param input uint16_t register_ID: the register address that summoned the write callback
 * 		@param output T** output: the data to output
 * 		@param ouptut uint16_t* length: the size of the output data
 * @return whether successful
 */
template <typename T>
bool Registers::AddReadCallback(uint16_t ID, void* context, bool (*read)(void*, uint16_t, T**, uint16_t*)) {
	bool success;
	osSemaphoreAcquire(RegistersSemaphore, osWaitForever);
	// Find register
	RegisterConfiguration* target_register = FindRegisterConfiguration(ID, &success);
	if (!success) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Get the type ID
	uint8_t registered_type = GetTypeID<T>(&success);
	if (!success) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Check template type same as register type
	if (target_register->register_info.type != registered_type) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Check the type is supported
	if (callbacks[target_register->register_info.type] == NULL) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Get the typed callback list
	std::vector<RegisterCallbacks<T>>* target_callback_list;
	target_callback_list = reinterpret_cast <std::vector<RegisterCallbacks<T>>*> (callbacks[target_register->register_info.type]);

	// Add the write callback to the register
	if (not target_register->mode.pointer &&  not target_register->mode.callbacks) {
		RegisterCallbacks<T> callback;
		callback.context = context;
		callback.write = NULL;
		callback.read = read;
		target_register->callbacks_ID = (uint16_t) target_callback_list->size();
		target_callback_list->push_back(callback);
	} else {
		RegisterCallbacks<T>* callback = &(target_callback_list->at(target_register->callbacks_ID));
		callback->context = context;
		callback->read = read;
	}

	// Set register mode and permissions
	target_register->mode.callbacks = true;
	target_register->access.read = true;

	osSemaphoreRelease(RegistersSemaphore);
	return true;
}

/**
 * @brief Link write and read callbacks to a register
 *
 * @param input ID: register address
 * @param input context: the context pointer to forward to the callback
 * @param input write: the write callback to link to the register
 * @param input read: the read callback to link to the register
 * @return whether successful
 */
template <typename T>
bool Registers::AddRegisterCallbacks(uint16_t ID, void* context, bool (*write)(void*, uint16_t, T*, uint16_t), bool (*read)(void*, uint16_t, T**, uint16_t*)) {
	if (!AddWriteCallback<T>(ID, context, write)) return false;
	if (!AddReadCallback<T>(ID, context, read)) return false;
	return true;
}

/**
 * @brief Write access a register
 *
 * @param input ID: register address
 * @param input T: value to write
 * @param input length: length of the data to write. DEFAULT=0
 * @param input use_semaphore: false to disable the use of semaphores for this access. DEFAULT=true
 * @return whether successful
 */
template <typename T>
bool Registers::WriteRegister(uint16_t ID, T* value, uint16_t length, bool use_semaphore) {
	bool success;
	osSemaphoreAcquire(RegistersSemaphore, osWaitForever);
	// Find register
	RegisterConfiguration* target_register = FindRegisterConfiguration(ID, &success);
	if (!success) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Get type ID
	uint8_t registered_type = GetTypeID<T>(&success);
	if (!success) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Check template type same as register type
	if (target_register->register_info.type != registered_type) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Check the type is supported
	if (callbacks[target_register->register_info.type] == NULL) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	if (target_register->access.write) {
		// Get typed callback list
		success = false;
		std::vector<RegisterCallbacks<T>>* target_callback_list = reinterpret_cast <std::vector<RegisterCallbacks<T>>*> (callbacks[target_register->register_info.type]);

		// Get register callbacks and pointer
		RegisterCallbacks<T> callbacks = target_callback_list->at(target_register->callbacks_ID);

		// Take semaphore
		if (use_semaphore && target_register->register_semaphore != NULL) {
			osSemaphoreAcquire(*(target_register->register_semaphore), osWaitForever);
		}

		// Pointer first
		if (target_register->mode.pointer) {
			success = true;
			if (target_register->register_info.isArray && target_register->register_info.length == 0) {
				std::vector<T>* register_vector = reinterpret_cast<std::vector<T>*>(callbacks.pointer);
				register_vector->clear();
				register_vector->resize(length);
				for (uint16_t index = 0; index < length; index++) {
					(*register_vector)[index] = value[index];
				}
			} else if (target_register->register_info.isArray && target_register->register_info.length > 0) {
				std::copy(&value[0], &value[target_register->register_info.length], callbacks.pointer);
			} else {
				*(callbacks.pointer) = *value;
			}
		}

		// Write callback then
		if (target_register->mode.callbacks) {
			// If registered callback, process
			if (callbacks.write != NULL) {
				if (target_register->register_info.isArray && target_register->register_info.length == 0) {
					success = callbacks.write(callbacks.context, ID, value, length);
				} else if (target_register->register_info.isArray && target_register->register_info.length > 0) {
					success = callbacks.write(callbacks.context, ID, value, target_register->register_info.length);
				} else {
					success = callbacks.write(callbacks.context, ID, value, 1);
				}
			}
		}

		// Release semaphore
		if (use_semaphore && target_register->register_semaphore != NULL) {
			osSemaphoreRelease(*(target_register->register_semaphore));
		}

		osSemaphoreRelease(RegistersSemaphore);
		return success;
	}
	osSemaphoreRelease(RegistersSemaphore);
	return false;
}

/**
 * @brief Write access to a vector register (WriteRegister is also compatible)
 *
 * @param input ID: register address
 * @param input value: vector to write
 * @param input use_semaphore: false to disable the use of semaphores for this access. DEFAULT=true
 * @return whether successful
 */
template <typename T>
bool Registers::WriteVectorRegister(uint16_t ID, std::vector<T> value, bool use_semaphore) {
	bool success;
	osSemaphoreAcquire(RegistersSemaphore, osWaitForever);
	// Find register
	RegisterConfiguration* target_register = FindRegisterConfiguration(ID, &success);
	if (!success) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Check the register is a vector
	if (!(target_register->register_info.isArray && target_register->register_info.length == 0)) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}
	osSemaphoreRelease(RegistersSemaphore);

	return WriteRegister(ID, value.data(), value.size(), use_semaphore);
}

/**
 * @brief Read access to a register
 *
 * @param input ID: register address
 * @param output output: register value
 * @param output length: output length
 * @param input use_semaphore: false to disable the use of semaphores for this access. DEFAULT=true
 * @return whether successful
 */
template <typename T>
bool Registers::ReadRegister(uint16_t ID, T* output, uint16_t* length, bool use_semaphore) {
	bool success;
	osSemaphoreAcquire(RegistersSemaphore, osWaitForever);
	// Find register
	RegisterConfiguration* target_register = FindRegisterConfiguration(ID, &success);
	if (!success) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Get type ID
	uint8_t registered_type = GetTypeID<T>(&success);
	if (!success) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Check template type same as register type
	if (target_register->register_info.type != registered_type) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Check the type is supported
	if (callbacks[target_register->register_info.type] == NULL) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	if (target_register->access.read) {
		// Get typed callback list
		success = false; // false by default to use pointer mode if the callback doesn't update the flag
		std::vector<RegisterCallbacks<T>>* target_callback_list = reinterpret_cast <std::vector<RegisterCallbacks<T>>*> (callbacks[target_register->register_info.type]);

		// Get register callbacks and pointer
		RegisterCallbacks<T> callbacks = target_callback_list->at(target_register->callbacks_ID);

		// Take semaphore
		if (use_semaphore && target_register->register_semaphore != NULL) {
			osSemaphoreAcquire(*(target_register->register_semaphore), osWaitForever);
		}

		// Callback first
		if (target_register->mode.callbacks) {
			T** callback_output;
			T output_buffer[256];
			if (output != NULL) {
				*callback_output = output;
			} else {
				*callback_output = output_buffer; // use a temporary buffer to store values if not input. TODO the size is limited so there is possibly an issue if output == NULL and length > 256
			}

			if (callbacks.read != NULL) {
				success = callbacks.read(callbacks.context, ID, callback_output, length);

				// Overrides the returned length if register is not a vector. Output length only useful for vectors
				if (target_register->register_info.isArray && target_register->register_info.length > 0) {
					*length = target_register->register_info.length;
				} else if (!target_register->register_info.isArray) {
					*length = 1;
				}

				if (*callback_output != output && output != NULL) {
					for (uint16_t index = 0; index < *length; index++) {
						output[index] = (*callback_output )[index];
					}
				}
			}
		}

		// Then pointer if false returned by callback
		if (not success && target_register->mode.pointer) {
			success = true;
			if (target_register->register_info.isArray && target_register->register_info.length == 0) {
				std::vector<T>* register_vector = reinterpret_cast<std::vector<T>*>(callbacks.pointer);
				*length = register_vector->size();
				if (output != NULL) {
					for (uint16_t index = 0; index < *length; index++) {
						output[index] = (*register_vector)[index];
					}
				}
			} else if (target_register->register_info.isArray && target_register->register_info.length > 0) {
				*length = target_register->register_info.length;
				if (output != NULL) {
					std::copy(&callbacks.pointer[0], &callbacks.pointer[target_register->register_info.length], output);
				}
			} else {
				*length = 1;
				if (output != NULL) {
					*output = *callbacks.pointer;
				}
			}
		}

		// Release semaphore
		if (use_semaphore && target_register->register_semaphore != NULL) {
			osSemaphoreRelease(*(target_register->register_semaphore));
		}

		osSemaphoreRelease(RegistersSemaphore);
		return true;
	}
	osSemaphoreRelease(RegistersSemaphore);
	return true;
}

/**
 * @brief Read access to a vector register
 *
 * @param input ID: register address
 * @param output output: register value put into a vector
 * @param input use_semaphore: false to disable the use of semaphores for this access. DEFAULT=true
 * @return whether successful
 */
template <typename T>
bool Registers::ReadVectorRegister(uint16_t ID, std::vector<T>* output, bool use_semaphore) {
	bool success;
	// Find register
	osSemaphoreAcquire(RegistersSemaphore, osWaitForever);
	RegisterConfiguration* target_register = FindRegisterConfiguration(ID, &success);
	if (!success) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}

	// Check the register is a vector
	if (!(target_register->register_info.isArray && target_register->register_info.length == 0)) {
		osSemaphoreRelease(RegistersSemaphore);
		return false;
	}
	osSemaphoreRelease(RegistersSemaphore);

	// Retrieve register length first
	uint16_t length = 0;
	success = ReadRegister<T>(ID, NULL, &length, use_semaphore);

	// Get register content
	T values[length];
	success = ReadRegister<T>(ID, values, &length, use_semaphore);

	// Fill the output vector
	output->clear();
	output->resize(length);
	for (uint16_t index = 0; index < length; index++) {
		(*output)[index] = values[index];
	}

	return true;
}
