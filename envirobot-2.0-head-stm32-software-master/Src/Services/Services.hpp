/*
 * Services.hpp
 *
 *  Created on: 9 déc. 2022
 *      Author: bignet
 */

#pragma once

#define NUMBER_OF_SERVICES	2			// maximum number of services in parallel
#define SERVICES_BUFFER_SIZE 256			// buffer size used to access remote register values. Maximum readable value size in bytes

#include <vector>
#include <stm32h750xx.h>
#include <stm32h7xx_hal.h>
#include "cmsis_os.h"

#include "Registers/Registers.hpp"
#include "Publishers/Publishers.hpp"
#include "LEDS/LEDS.hpp"
#include "Communication/Communication.hpp"

#include "Configuration.h"
#include "Definitions/ServicesDefinition.h"
#include "Definitions/CommunicationDefinition.h"
#include "Configurations/RegistersConfiguration.h"
#include "Configurations/CommunicationConfiguration.h"

#include "PlatformLEDs/PlatformLEDs.hpp"

// ServiceInterface struct
struct ServiceInterface {
	uint8_t interface;			// interface used by the service
	uint8_t address;			// module address remotely accessed by the service
};

// ServiceConfiguration
struct ServiceConfiguration {
	Register register_;				// Register configuration of the service
	uint16_t length;				// data size in bytes associated to the service
	ServiceInterface interface;		// interface used by the service
	bool access;					// access mode of the service
};

// Service struct
struct Service {
	bool active;							// whether active or not
	bool reception_flag;					// reception flag to exit blocking loop
	ServiceConfiguration configuration;		// service configuration
	uint16_t timeout;						// service timeout
	uint32_t timeout_timestamp;				// timestamp from which a timeout is triggered. Automatically computed
	bool error;								// service access error flag
};

// Services class used to remotely access other module registers
class Services {
public:
	Services();
	void Init(Registers* registers_, Communication* communication_, LEDS* leds_);

	template<typename T>
	uint8_t WriteRemoteRegister(uint16_t register_ID, ServiceInterface interface, T* data, uint16_t timeout = 0, uint8_t delay_mode = OS_DELAY); // this version takes the internal definition of the register

	template<typename T>
	uint8_t WriteRemoteRegister(Register remote_register, ServiceInterface interface, T* data, uint16_t timeout = 0, uint8_t delay_mode = OS_DELAY); // this version overwrites the internal definition and expected a specific register definition

	template<typename T>
	uint8_t WriteRemoteVectorRegister(uint16_t register_ID, ServiceInterface interface, std::vector<T>* data, uint16_t timeout = 0, uint8_t delay_mode = OS_DELAY);

	template<typename T>
	uint8_t WriteRemoteVectorRegister(Register remote_register, ServiceInterface interface, std::vector<T>* data, uint16_t timeout = 0, uint8_t delay_mode = OS_DELAY);

	template<typename T>
	uint8_t ReadRemoteRegister(uint16_t register_ID, ServiceInterface interface, T* output, uint16_t* length, uint16_t timeout = 0, uint8_t delay_mode = OS_DELAY);

	template<typename T>
	uint8_t ReadRemoteRegister(Register remote_register, ServiceInterface interface, T* output, uint16_t* length, uint16_t timeout = 0, uint8_t delay_mode = OS_DELAY);

	template<typename T>
	uint8_t ReadRemoteVectorRegister(uint16_t register_ID, ServiceInterface interface, std::vector<T>* output, uint16_t timeout = 0, uint8_t delay_mode = OS_DELAY);

	template<typename T>
	uint8_t ReadRemoteVectorRegister(Register remote_register, ServiceInterface interface, std::vector<T>* output, uint16_t timeout = 0, uint8_t delay_mode = OS_DELAY);

	void ReceiveWrite(ServiceConfiguration information, uint8_t status);//Register remote_register, uint8_t interface, uint8_t address, uint8_t status);

	template<typename T>
	void ReceiveRead(ServiceConfiguration information, T* data);//Register remote_register, uint16_t length, uint8_t interface, uint8_t address, T* data);

	void ReceiveError(ServiceConfiguration information, uint8_t error);//Register remote_register, uint8_t interface, uint8_t address, bool access, uint8_t error);

	Register FindRemoteRegister(Register remote_register, ServiceInterface interface, bool access, bool* success);

private:
	template<typename T>
	uint8_t AccessRemoteRegister(uint8_t* queue_index, Register remote_register, ServiceInterface interface,  bool access, T* data, uint16_t timeout, uint8_t delay_mode);

	void ResetQueue(void);
	uint8_t FindQueueIndex(uint8_t start_index, Register remote_register, ServiceInterface interface, bool access, bool* success);
	bool QueueAvailable(Register remote_register, ServiceInterface interface, bool access);
	uint8_t FindAvailableIndex(bool* success);

	uint8_t AddToQueue(Register remote_register, ServiceInterface interface, bool access, uint16_t timeout, bool* success);
	bool RemoveFromQueue(uint8_t index);

	template<typename T>
	void TranferData(T* from, T* to, uint16_t length);

	// OS
	osSemaphoreId_t ServiceSemaphore;

	uint8_t queue_size;
	Service services[NUMBER_OF_SERVICES];

	uint8_t buffer_uint8[NUMBER_OF_SERVICES][SERVICES_BUFFER_SIZE]; 			// this buffer is always required as it stores error and write reception_flags too
#if defined(USE_UINT16_COMMUNICATION) && defined(USE_UINT16_REGISTER)
	uint16_t buffer_uint16[NUMBER_OF_SERVICES][SERVICES_BUFFER_SIZE];
#endif
#if defined(USE_UINT32_COMMUNICATION) && defined(USE_UINT32_REGISTER)
	uint32_t buffer_uint32[NUMBER_OF_SERVICES][SERVICES_BUFFER_SIZE];
#endif
#if defined(USE_UINT64_COMMUNICATION) && defined(USE_UINT64_REGISTER)
	uint64_t buffer_uint64[NUMBER_OF_SERVICES][SERVICES_BUFFER_SIZE];
#endif
#if defined(USE_INT8_COMMUNICATION) && defined(USE_INT8_REGISTER)
	int8_t buffer_int8[NUMBER_OF_SERVICES][SERVICES_BUFFER_SIZE];
#endif
#if defined(USE_INT16_COMMUNICATION) && defined(USE_INT16_REGISTER)
	int16_t buffer_int16[NUMBER_OF_SERVICES][SERVICES_BUFFER_SIZE];
#endif
#if defined(USE_INT32_COMMUNICATION) && defined(USE_INT32_REGISTER)
	int32_t buffer_int32[NUMBER_OF_SERVICES][SERVICES_BUFFER_SIZE];
#endif
#if defined(USE_INT64_COMMUNICATION) && defined(USE_INT64_REGISTER)
	int64_t buffer_int64[NUMBER_OF_SERVICES][SERVICES_BUFFER_SIZE];
#endif
#if defined(USE_FLOAT_COMMUNICATION) && defined(USE_FLOAT_REGISTER)
	float buffer_float[NUMBER_OF_SERVICES][SERVICES_BUFFER_SIZE];
#endif
#if defined(USE_DOUBLE_COMMUNICATION) && defined(USE_DOUBLE_REGISTER)
	double buffer_double[NUMBER_OF_SERVICES][SERVICES_BUFFER_SIZE];
#endif

	Registers* registers;
	Communication* communication;
	LEDS* leds;
};

/**
 * @brief Remote write access to a register internally defined in Registers
 *
 * @param input register_ID: the register address from Registers to retrieve the register information (type, isArray, length)
 * @param input interface: interface to access the remote register. The address targets a specific module.
 * @param input data: data to write
 * @param input timeout: time left to access the remote register DEFAULT=0(no timeout)
 * @param input delay_mode: select the delay function. DEFAULT=OS_DELAY for freeRTOS tasks. Use HAL_Delay if not in freeRTOS task.
 * @return the remote write access acknowledgment flag
 */
template<typename T>
uint8_t Services::WriteRemoteRegister(uint16_t register_ID, ServiceInterface interface, T* data, uint16_t timeout, uint8_t delay_mode) {
	bool success;
	Register internal_register = registers->FindRegister(register_ID, &success);

	if (not success) return ERROR;

	// Check type
	uint8_t declared_type = registers->GetTypeID<T>(&success);

	if (not success) return ERROR;

	if (internal_register.type != declared_type) return ERROR;

	return WriteRemoteRegister(internal_register, interface, data, timeout, delay_mode);
}

/**
 * @brief Remote write access to a register NOT internally defined in Registers
 *
 * @param input remote_register: the full register configuration to access
 * @param input interface: interface to access the remote register. The address targets a specific module.
 * @param input data: data to write
 * @param input timeout: time left to access the remote register DEFAULT=0(no timeout)
 * @param input delay_mode: select the delay function. DEFAULT=OS_DELAY for freeRTOS tasks. Use HAL_Delay if not in freeRTOS task.
 * @return the write remote access acknowledgment
 */
template<typename T>
uint8_t Services::WriteRemoteRegister(Register remote_register, ServiceInterface interface, T* data, uint16_t timeout, uint8_t delay_mode) {
	uint8_t queue_index;
	uint8_t flag = AccessRemoteRegister<T>(&queue_index, remote_register, interface, WRITE, data, timeout, delay_mode);

	if (flag != OK) return flag;

	osSemaphoreAcquire(ServiceSemaphore, osWaitForever);
	uint8_t status = buffer_uint8[queue_index][0];
	osSemaphoreRelease(ServiceSemaphore);

	RemoveFromQueue(queue_index);

	return status;
}

/**
 * @brief Remote write access to a vector register internally defined in Registers
 *
 * @param input register_ID: the register address from Registers to retrieve the register information (type, isArray, length)
 * @param input interface: interface to access the remote register. The address targets a specific module.
 * @param input data: data to write
 * @param input timeout: time left to access the remote register DEFAULT=0(no timeout)
 * @param input delay_mode: select the delay function. DEFAULT=OS_DELAY for freeRTOS tasks. Use HAL_Delay if not in freeRTOS task.
 * @return the write remote access acknowledgment
 */
template<typename T>
uint8_t Services::WriteRemoteVectorRegister(uint16_t register_ID, ServiceInterface interface, std::vector<T>* data, uint16_t timeout, uint8_t delay_mode) {
	bool success;
	Register internal_register = registers->FindRegister(register_ID, &success);

	if (not success) return ERROR;

	// Check type
	uint8_t declared_type = registers->GetTypeID<T>(&success);

	if (not success) return ERROR;

	if (internal_register.type != declared_type) return ERROR;

	// Check the register is a vector
	if (!(internal_register.isArray && internal_register.isArray)) return ERROR;

	return WriteRemoteVectorRegister(internal_register, interface, data, timeout, delay_mode);
}

/**
 * @brief Remote write access to a vector register NOT internally defined in Registers
 *
 * @param input remote_register: the full register configuration to access
 * @param input interface: interface to access the remote register. The address targets a specific module.
 * @param input data: data to write
 * @param input timeout: time left to access the remote register DEFAULT=0(no timeout)
 * @param input delay_mode: select the delay function. DEFAULT=OS_DELAY for freeRTOS tasks. Use HAL_Delay if not in freeRTOS task.
 * @return the write remote access acknowledgment
 */
template<typename T>
uint8_t Services::WriteRemoteVectorRegister(Register remote_register, ServiceInterface interface, std::vector<T>* data, uint16_t timeout, uint8_t delay_mode) {
	remote_register.isArray = true;
	remote_register.length = 0;
	T* reinterpreted_data = reinterpret_cast<T*>(data);
	return WriteRemoteRegister(remote_register, interface, reinterpreted_data, timeout, delay_mode);
}

/**
 * @brief Remote read access to a register internally defined in Registers
 *
 * @param input register_ID: the register address from Registers to retrieve the register information (type, isArray, length)
 * @param input interface: interface to access the remote register. The address targets a specific module.
 * @param output output: remote register value
 * @param output length: remote register length
 * @param input timeout: time left to access the remote register DEFAULT=0(no timeout)
 * @param input delay_mode: select the delay function. DEFAULT=OS_DELAY for freeRTOS tasks. Use HAL_Delay if not in freeRTOS task.
 * @return the access status
 */
template<typename T>
uint8_t Services::ReadRemoteRegister(uint16_t register_ID, ServiceInterface interface, T* output, uint16_t* length, uint16_t timeout, uint8_t delay_mode) {
	bool success;
	Register internal_register = registers->FindRegister(register_ID, &success);

	if (not success) return ERROR;

	// Check type
	uint8_t declared_type = registers->GetTypeID<T>(&success);

	if (not success) return ERROR;

	if (internal_register.type != declared_type) return ERROR;

	return ReadRemoteRegister(internal_register, interface, output, length, timeout, delay_mode);
}

/**
 * @brief Remote read access to a register NOT internally defined in Registers
 *
 * @param input remote_register: the full register configuration to access
 * @param input interface: interface to access the remote register. The address targets a specific module.
 * @param output output: remote register value
 * @param output length: remote register length
 * @param input timeout: time left to access the remote register DEFAULT=0(no timeout)
 * @param input delay_mode: select the delay function. DEFAULT=OS_DELAY for freeRTOS tasks. Use HAL_Delay if not in freeRTOS task.
 * @return the access status
 */
template<typename T>
uint8_t Services::ReadRemoteRegister(Register remote_register, ServiceInterface interface, T* output, uint16_t* length, uint16_t timeout, uint8_t delay_mode) {
	uint8_t queue_index;
	uint8_t flag = AccessRemoteRegister<T>(&queue_index, remote_register, interface, READ, NULL, timeout, delay_mode);

	if (flag != OK) return flag;

	osSemaphoreAcquire(ServiceSemaphore, osWaitForever);

	 // If error flag detected in message, returns the error flag
	if (services[queue_index].error) {
		uint8_t error = buffer_uint8[queue_index][0];
		osSemaphoreRelease(ServiceSemaphore);
		return error;
	}

	// Transfer the read value to the output
	*length = services[queue_index].configuration.length;
	switch (services[queue_index].configuration.register_.type) {
	case UINT8_TYPE:
#if defined(USE_UINT8_COMMUNICATION) && defined(USE_UINT8_REGISTER)
		TranferData<uint8_t>(&buffer_uint8[queue_index][0], (uint8_t*) output, services[queue_index].configuration.length);
		break;
#endif
#if defined(USE_UINT16_COMMUNICATION) && defined(USE_UINT16_REGISTER)
	case UINT16_TYPE:
		TranferData<uint16_t>(&buffer_uint16[queue_index][0], (uint16_t*) output, services[queue_index].configuration.length);
		break;
#endif
#if defined(USE_UINT32_COMMUNICATION) && defined(USE_UINT32_REGISTER)
	case UINT32_TYPE:
		TranferData<uint32_t>(&buffer_uint32[queue_index][0], (uint32_t*) output, services[queue_index].configuration.length);
		break;
#endif
#if defined(USE_UINT64_COMMUNICATION) && defined(USE_UINT64_REGISTER)
	case UINT64_TYPE:
		TranferData<uint64_t>(&buffer_uint64[queue_index][0], (uint64_t*) output, services[queue_index].configuration.length);
		break;
#endif
#if defined(USE_INT8_COMMUNICATION) && defined(USE_INT8_REGISTER)
	case INT8_TYPE:
		TranferData<int8_t>(&buffer_int8[queue_index][0], (int8_t*) output, services[queue_index].configuration.length);
		break;
#endif
#if defined(USE_INT16_COMMUNICATION) && defined(USE_INT16_REGISTER)
	case INT16_TYPE:
		TranferData<int16_t>(&buffer_int16[queue_index][0], (int16_t*) output, services[queue_index].configuration.length);
		break;
#endif
#if defined(USE_INT32_COMMUNICATION) && defined(USE_INT32_REGISTER)
	case INT32_TYPE:
		TranferData<int32_t>(&buffer_int32[queue_index][0], (int32_t*) output, services[queue_index].configuration.length);
		break;
#endif
#if defined(USE_INT64_COMMUNICATION) && defined(USE_INT64_REGISTER)
	case INT64_TYPE:
		TranferData<int64_t>(&buffer_int64[queue_index][0], (int64_t*) output, services[queue_index].configuration.length);
		break;
#endif
#if defined(USE_FLOAT_COMMUNICATION) && defined(USE_FLOAT_REGISTER)
	case FLOAT_TYPE:
		TranferData<float>(&buffer_float[queue_index][0], (float*) output, services[queue_index].configuration.length);
		break;
#endif
#if defined(USE_DOUBLE_COMMUNICATION) && defined(USE_DOUBLE_REGISTER)
	case DOUBLE_TYPE:
		TranferData<double>(&buffer_double[queue_index][0], (double*) output, services[queue_index].configuration.length);
		break;
#endif
	default:
		break;
	}
	osSemaphoreRelease(ServiceSemaphore);

	// Remove the service, free the index
	RemoveFromQueue(queue_index);

	return OK;
}

/**
 * @brief Remote read access to a vector register internally defined in Registers
 *
 * @param input register_ID: the register address from Registers to retrieve the register information (type, isArray, length)
 * @param input interface: interface to access the remote register. The address targets a specific module.
 * @param output output: remote register value
 * @param input timeout: time left to access the remote register DEFAULT=0(no timeout)
 * @param input delay_mode: select the delay function. DEFAULT=OS_DELAY for freeRTOS tasks. Use HAL_Delay if not in freeRTOS task.
 * @return the access status
 */
template<typename T>
uint8_t Services::ReadRemoteVectorRegister(uint16_t register_ID, ServiceInterface interface, std::vector<T>* output, uint16_t timeout, uint8_t delay_mode) {
	bool success;
	Register internal_register = registers->FindRegister(register_ID, &success);

	if (not success) return ERROR;

	// Check type
	uint8_t declared_type = registers->GetTypeID<T>(&success);

	if (not success) return ERROR;

	if (internal_register.type != declared_type) return ERROR;

	// Check vector declaration
	if (!(internal_register.isArray && internal_register.isArray)) return ERROR;

	return ReadRemoteVectorRegister(internal_register, interface, output, timeout, delay_mode);
}

/**
 * @brief Remote read access to a vector register NOT internally defined in Registers
 *
 * @param input remote_register: the full register configuration to access
 * @param input interface: interface to access the remote register. The address targets a specific module.
 * @param output output: remote register value
 * @param input timeout: time left to access the remote register DEFAULT=0(no timeout)
 * @param input delay_mode: select the delay function. DEFAULT=OS_DELAY for freeRTOS tasks. Use HAL_Delay if not in freeRTOS task.
 * @return the access status
 */
template<typename T>
uint8_t Services::ReadRemoteVectorRegister(Register remote_register, ServiceInterface interface, std::vector<T>* output, uint16_t timeout, uint8_t delay_mode) {
	remote_register.isArray = true;
	remote_register.length = 0;

	uint16_t length;
	T output_array[SERVICES_BUFFER_SIZE];
	uint8_t status = ReadRemoteRegister(remote_register, interface, output_array, &length, timeout, delay_mode);

	if (status != OK) return status;

	std::vector<T> output_data(output_array, output_array + length);
	*output = output_data;

	return OK;
}

/**
 * @brief Receive a service feedback. Find the corresponding services, transfer the read value and unlock services.
 *
 * @param input information: the service feedback information
 * @param input data: returned data from the remote module
 */
template<typename T>
void Services::ReceiveRead(ServiceConfiguration information, T* data) {
	bool success = true;
	uint8_t start_index = 0;
	while (success) {
		uint8_t index = FindQueueIndex(start_index, information.register_, information.interface, information.access, &success);
		if (not success) return;

		if (information.length > SERVICES_BUFFER_SIZE) return;

#ifdef USE_COM_LED_SERVICES_RECEPTION
		CommunicationLED(leds, information.interface.interface);
#endif

		osSemaphoreAcquire(ServiceSemaphore, osWaitForever);
		services[index].configuration.length = information.length;

		// Transfer data to the service buffer
		switch (information.register_.type) {
		case UINT8_TYPE:
#if defined(USE_UINT8_COMMUNICATION) && defined(USE_UINT8_REGISTER)
			TranferData<uint8_t>((uint8_t*) data, &buffer_uint8[index][0], information.length);
			break;
#endif
#if defined(USE_UINT16_COMMUNICATION) && defined(USE_UINT16_REGISTER)
		case UINT16_TYPE:
			TranferData<uint16_t>((uint16_t*) data, &buffer_uint16[index][0], information.length);
			break;
#endif
#if defined(USE_UINT32_COMMUNICATION) && defined(USE_UINT32_REGISTER)
		case UINT32_TYPE:
			TranferData<uint32_t>((uint32_t*) data, &buffer_uint32[index][0], information.length);
			break;
#endif
#if defined(USE_UINT64_COMMUNICATION) && defined(USE_UINT64_REGISTER)
		case UINT64_TYPE:
			TranferData<uint64_t>((uint64_t*) data, &buffer_uint64[index][0], information.length);
			break;
#endif
#if defined(USE_INT8_COMMUNICATION) && defined(USE_INT8_REGISTER)
		case INT8_TYPE:
			TranferData<int8_t>((int8_t*) data, &buffer_int8[index][0], information.length);
			break;
#endif
#if defined(USE_INT16_COMMUNICATION) && defined(USE_INT16_REGISTER)
		case INT16_TYPE:
			TranferData<int16_t>((int16_t*) data, &buffer_int16[index][0], information.length);
			break;
#endif
#if defined(USE_INT32_COMMUNICATION) && defined(USE_INT32_REGISTER)
		case INT32_TYPE:
			TranferData<int32_t>((int32_t*) data, &buffer_int32[index][0], information.length);
			break;
#endif
#if defined(USE_INT64_COMMUNICATION) && defined(USE_INT64_REGISTER)
		case INT64_TYPE:
			TranferData<int64_t>((int64_t*) data, &buffer_int64[index][0], information.length);
			break;
#endif
#if defined(USE_FLOAT_COMMUNICATION) && defined(USE_FLOAT_REGISTER)
		case FLOAT_TYPE:
			TranferData<float>((float*) data, &buffer_float[index][0], information.length);
			break;
#endif
#if defined(USE_DOUBLE_COMMUNICATION) && defined(USE_DOUBLE_REGISTER)
		case DOUBLE_TYPE:
			TranferData<double>((double*) data, &buffer_double[index][0], information.length);
			break;
#endif
		default:
			break;
		}

		// Unlock the blocking loop of the service
		services[index].reception_flag = true;
		osSemaphoreRelease(ServiceSemaphore);

		start_index = index + 1;
	}
}

/**
 * @brief Tranfer a certain amount of data from a pointer to another
 *
 * @param input from: base pointer to copy data from
 * @param output to: base pointer to copy data to
 * @param input length: number of elements to copy
 */
template<typename T>
void Services::TranferData(T* from, T* to, uint16_t length) {
	for (uint16_t buffer_index = 0; buffer_index < length; buffer_index++) {
		to[buffer_index] = from[buffer_index];
	}
}

/**
 * @brief Remote access to a register
 *
 * @param output queue_index: service queue index
 * @param input remote_register: remote register to access
 * @param input interface: interface used to access the remote register
 * @param input access: remote register access mode, either WRITE or READ
 * @param input data: data to write in case of write access
 * @param input timeout: time left to process the service access
 * @param input delay_mode: defines the delay function used
 */
template<typename T>
uint8_t Services::AccessRemoteRegister(uint8_t* queue_index, Register remote_register, ServiceInterface interface,  bool access, T* data, uint16_t timeout, uint8_t delay_mode) {
	// Check that the register can be added to the queue. Mainly refuses if the register address is already queued with different type or length
	if (not QueueAvailable(remote_register, interface, access)) return false;

	// Override the register type
	bool success;
	remote_register.type = registers->GetTypeID<T>(&success);

	if (not success) return ERROR;

	// Add service to queue
	*queue_index = AddToQueue(remote_register, interface, access, timeout, &success);

	if (not success) return ERROR;

	// Send access request
	MessageHeader request;
	request.interface = interface.interface;
	request.target_address = interface.address;

	MessageElement element;
	element.ack = false;
	element.command = true;
	element.access = access;
	element.element_register = remote_register;
	if (access == WRITE) element.data = communication->BuildVector<T>(communication->BuildTVector(element, data));

	// Build message
	if (not communication->BuildPayload(&request, element)) return ERROR;

	// Send remote register access
	communication->Send(request);

	// Wait acknowledgment or timeout
	while (1) {
		osSemaphoreAcquire(ServiceSemaphore, osWaitForever);
		if (services[*queue_index].reception_flag) {
			osSemaphoreRelease(ServiceSemaphore);
			break;
		}
		if (services[*queue_index].timeout > 0 && HAL_GetTick() > services[*queue_index].timeout_timestamp) {
			osSemaphoreRelease(ServiceSemaphore);
			return TIMEOUT;
		}
		osSemaphoreRelease(ServiceSemaphore);

		if (delay_mode == OS_DELAY)
			osDelay(10);
		else if (delay_mode == HAL_DELAY)
			HAL_Delay(10);
	}

	return OK;
}
