/*
 * Protocol.hpp
 *
 *  Created on: 30 nov. 2022
 *      Author: bignet
 */

#pragma once

#include <vector>
#include <stm32h750xx.h>
#include <stm32h7xx_hal.h>

#include "Configurations/CommunicationConfiguration.h"

#define BUFFER_SIZE 256				// read and write buffer size in bytes

struct MessageHeader {
	uint8_t interface;
	std::vector<uint8_t> payload;
	uint8_t target_address;
	uint8_t source_address;
};

// CommunicationProtocol class used as a basis to derive custom Protocols
class CommunicationProtocol {
public:
	void AddInterfaceID(uint8_t ID);
	void AddInterfaceBuffer();

	virtual MessageHeader DecodeProtocol(uint8_t ID, std::vector<uint8_t> message, uint16_t* index, bool* success); // virtual MessageHeader Process(std::vector<uint8_t> data, uint16_t* index, bool* success);
	virtual HAL_StatusTypeDef ActivateReception(uint8_t ID);
	virtual HAL_StatusTypeDef Send(MessageHeader message);

	bool InterfaceExists(uint8_t ID);
	virtual uint8_t GetID(void* pointer, bool* success);

	std::vector<uint8_t>* GetBuffer(uint8_t ID, bool* success);
	uint16_t* GetBufferIndex(uint8_t ID, bool* success);

	void SetProtocolPayloadMaxLength(uint16_t length);
	uint16_t GetProtocolPayloadMaxLength(void);

protected:
	uint8_t GetInterfaceIndex(uint8_t ID, bool* success);
	uint8_t GetIDFromIndex(uint8_t index, bool* success);

	std::vector<std::vector<uint8_t>> buffers_to_transfer; // send buffers

private:
	std::vector<uint8_t> interfaces_id;
	uint16_t max_payload_length;

	std::vector<std::vector<uint8_t>> buffers; // reception buffers
	std::vector<uint16_t> buffers_index;
};
