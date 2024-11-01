/*
 * Communication.hpp
 *
 *  Created on: 30 nov. 2022
 *      Author: bignet
 */

#pragma once

#include <vector>
#include <cstring>
#include <stm32h750xx.h>
#include <stm32h7xx_hal.h>
#include "cmsis_os.h"

#include "Protocols/Communication/Core/CommunicationProtocol.hpp"
#include "Registers/Registers.hpp"
#include "Subscribers/MasterSubscribers.hpp"
#include "LEDS/LEDS.hpp"

#include "Definitions/CommunicationDefinition.h"
#include "Configurations/RegistersConfiguration.h"
#include "Configurations/CommunicationConfiguration.h"
#include "RegisterMaps/RegisterMapCommunication.h"

#include "PlatformLEDs/PlatformLEDs.hpp"

class Services;

struct MessageElement {
	Register element_register;

	bool ack;
	bool command; // false = command, true = ack
	bool access; // false = write, true = read for commands

	std::vector<uint8_t> data; // TODO, set as the register type with template
};

// Communication Class to communication with other modules
class Communication {
public:
	Communication();
	void Init(Registers* registers, Services* services, MasterSubscribers* subscribers, LEDS* leds);
	void AddRegisters(void);

	void Spin(void);
	void AddProtocol(CommunicationProtocol* protocol);

	HAL_StatusTypeDef Send(MessageHeader message);
	uint16_t Process(uint8_t ID, std::vector<uint8_t> data, bool* success);

	uint8_t GetID(void* handle, bool* success);
	std::vector<uint8_t>* GetBuffer(uint8_t ID, bool* success);
	uint16_t* GetBufferIndex(uint8_t ID, bool* success);
	uint16_t GetProtocolPayloadMaxLength(uint8_t ID, bool* success);

	HAL_StatusTypeDef ActivateReception(uint8_t ID);

	template<typename T>
	std::vector<uint8_t> ReadElement(MessageElement* element, bool* success);

	bool BuildPayload(MessageHeader* message, MessageElement element);

	template<typename T>
	std::vector<T> BuildTVector(MessageElement element, T* data);

	template<typename T>
	std::vector<uint8_t> BuildSingle(T input);

	template<typename T>
	std::vector<uint8_t> BuildVector(std::vector<T> inputs);

	bool SetModuleAddress(uint8_t address);
	bool AddGroupAddress(uint8_t address);
	bool RemoveGroupAddress(uint8_t address);
	bool ClearGroupAddress(void);

private:
	// OS
	osSemaphoreId_t CommunicationSemaphore;

	void ParseMessage(MessageHeader message);
	bool FindRegister(MessageHeader message, MessageHeader* response, MessageElement* element);
	CommunicationProtocol* FindProtocol(uint8_t ID, bool* success);

	bool AcceptMessage(MessageHeader message);
	bool InGroupAddresses(uint8_t address);

	template<typename T>
	T ExtractSingle(std::vector<uint8_t> payload, uint16_t start_index, bool* success);

	template<typename T>
	void ExtractArray(T* data, std::vector<uint8_t> payload, uint16_t start_index, uint16_t length, bool* success);

	template<typename T>
	std::vector<T> ExtractVector(std::vector<uint8_t> payload, uint16_t start_index, uint16_t length, bool* success);

	template<typename T>
	uint16_t ProcessElement(MessageHeader message, MessageHeader* response, MessageElement element, uint16_t start_index);

	template<typename T>
	uint16_t ProcessCommand(MessageHeader message, MessageHeader* response, MessageElement element, uint16_t start_index);

	template<typename T>
	uint16_t ProcessWrite(MessageHeader message, MessageHeader* response, MessageElement response_content, MessageElement element, uint16_t start_index);

	template<typename T>
	uint16_t ProcessRead(MessageHeader message, MessageHeader* response, MessageElement response_content, MessageElement element, uint16_t start_index);

	template<typename T>
	uint16_t ProcessService(MessageHeader message, MessageElement element, uint16_t start_index);

	template<typename T>
	uint16_t ProcessSubscriber(MessageHeader message, MessageElement element, uint16_t start_index);

	Registers* registers;
	Services* services;
	MasterSubscribers* subscribers;
	LEDS* leds;

	uint8_t address; // bus address of the STM32
	std::vector<uint8_t> group_addresses;
	std::vector<CommunicationProtocol*> protocols;
	std::vector<MessageHeader> pending_messages;
};

/**
 * @brief Extract a typed value from a payload as of a start index
 *
 * @param input payload: the list of bytes to extract a value from
 * @param input start_index: the index to start the extraction
 * @param output success: whether successful
 * @return the typed extracted value
 */
template<typename T>
T Communication::ExtractSingle(std::vector<uint8_t> payload, uint16_t start_index, bool* success) {
	uint64_t data = 0;
	uint8_t type_length = sizeof(T);

	// Check payload length
	if (payload.size() < start_index + type_length) {
		*success = false;
		return 0;
	}

	// Fill a uint64 variable
	for (uint8_t index = 0; index < type_length; index++) {
		data |= (payload[start_index + index] << ((type_length - 1 - index) << 3));
	}

	// Recast to the expected type
    T* reinterpreted_data = reinterpret_cast<T*>(&data);

    // Return data
    *success = true;
	return *reinterpreted_data;
}

/**
 * @brief Extract an array of typed values from a payload as of a start index
 *
 * @param output data: a pointer to store the extracted array
 * @param input payload: the list of bytes to extract a value from
 * @param input start_index: the index to start the extraction
 * @param input length: the number of values to extract
 * @param output success: whether successful
 */
template<typename T>
void Communication::ExtractArray(T* data, std::vector<uint8_t> payload, uint16_t start_index, uint16_t length, bool* success) {
	uint8_t type_length = sizeof(T);

	for (uint8_t index = 0; index < length; index++) {
		T element = ExtractSingle<T>(payload, start_index + (uint16_t) (index * type_length), success);
		if (not *success)
			return;
		data[index] = element;
	}
}

/**
 * @brief Extract a vector of typed values from a payload as of a start index
 *
 * @param input payload: the list of bytes to extract a value from
 * @param input start_index: the index to start the extraction
 * @param input length: the number of values to extract
 * @param output success: whether successful
 * @return a vector with extracted values
 */
template<typename T>
std::vector<T> Communication::ExtractVector(std::vector<uint8_t> payload, uint16_t start_index, uint16_t length, bool* success) {
	std::vector<T> data;
	uint8_t type_length = sizeof(T);

	for (uint8_t index = 0; index < length; index++) {
		T element = ExtractSingle<T>(payload, start_index + (uint16_t) (index * type_length), success);
		if (not *success)
			return data;
		data.push_back(element);
	}
	return data;
}

/**
 * @brief Translate a typed array into a typed vector based on the associated register
 *
 * @param input element: stores the associated register information
 * @param input data: input data array
 * @return a vector with typed values
 */
template<typename T>
std::vector<T> Communication::BuildTVector(MessageElement element, T* data) {
	std::vector<T> vector;
	// single case
	if (not element.element_register.isArray) {
		vector = std::vector<T>{*data};
	// array case
	} else if (element.element_register.isArray && element.element_register.length > 0)
		vector = std::vector<T>(data, data + element.element_register.length);
	// vector case
	else if (element.element_register.isArray && element.element_register.length == 0)
		vector = *reinterpret_cast<std::vector<T>*>(data);
	return vector;
}

/**
 * @brief Translate a typed value into a vector of bytes
 *
 * @param input input: typed input
 * @return a byte vector corresponding to the input value
 */
template<typename T>
std::vector<uint8_t> Communication::BuildSingle(T input) {
	std::vector<uint8_t> data;
	uint8_t type_length = sizeof(T);
    uint64_t reinterpreted_input = 0;
    std::memcpy(&reinterpreted_input, &input, sizeof(T)); // this is mandatory to get rid of the memory alignment problem. A simple cast can't work here

	for (uint8_t index = 0; index < type_length; index++) {
		data.push_back((uint8_t) ((reinterpreted_input >> ((type_length - 1 - index) << 3)) & 0xFF)); // previously (8 * (type_length - 1 - index))) & 0xFF))
	}

	return data;
}

/**
 * @brief Translate a vector of typed value into a vector of bytes
 *
 * @param input inputs: vector typed inputs
 * @return a byte vector corresponding to the input values in a row
 */
template<typename T>
std::vector<uint8_t> Communication::BuildVector(std::vector<T> inputs) {
    std::vector<uint8_t> data;

    for (T input : inputs) {
        std::vector<uint8_t> single = BuildSingle<T>(input);
        data.insert(data.end(), single.begin(), single.end());
    }

    return data;
}

/**
 * @brief Access the module registers based on the element access. Build a vector of bytes based on the read value.
 *
 * @param input element: the register to read
 * @return a byte vector corresponding to the register value
 */
template<typename T>
std::vector<uint8_t> Communication::ReadElement(MessageElement* element, bool* success) {
	std::vector<uint8_t> data;

	T register_read;
	uint16_t length;
	*success = registers->ReadRegister<T>(element->element_register.address, &register_read, &length);

	if (not *success)
		return data;

	// Get vector of type T
	std::vector<T> register_read_vector_cast = BuildTVector(*element, &register_read);

	// transform it in vector of uint8_t
	data = BuildVector<T>(register_read_vector_cast);

	return data;
}

