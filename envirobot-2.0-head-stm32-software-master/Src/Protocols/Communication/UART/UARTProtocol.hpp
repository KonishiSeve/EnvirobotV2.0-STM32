/*
 * UARTProtocol.hpp
 *
 *  Created on: 30 nov. 2022
 *      Author: bignet
 */

#pragma once

#define PROTOCOL_START {0xC3, 0x3C}
#define PROTOCOL_END {0xFF}

#include <vector>
#include <stm32h750xx.h>
#include <stm32h7xx_hal.h>

#include "Protocols/Communication/Core/TemplateCommunicationProtocol.hpp"

#include "Definitions/CommunicationDefinition.h"

// UARTProtocol class used to interface with every UART interface
class UARTProtocol : public TemplateCommunicationProtocol<UART_HandleTypeDef> {
public:
	UARTProtocol();
	HAL_StatusTypeDef ActivateReception(uint8_t ID);
	MessageHeader DecodeProtocol(uint8_t ID, std::vector<uint8_t> message, uint16_t* index, bool* success); //remove the protocol part of the received message
	HAL_StatusTypeDef Send(MessageHeader message);
	bool SetAsRS485(uint8_t ID);

private:
	std::vector<uint8_t> CodeProtocol(MessageHeader message); //add the protocol part to get data to send

	uint8_t GetChecksum(std::vector<uint8_t> message);
	bool Checksum(uint16_t start_index, uint16_t end_index, std::vector<uint8_t> message);
	uint16_t DetectPattern(std::vector<uint8_t> message, std::vector<uint8_t> pattern, uint16_t start_index, bool* success);
	bool PatternPresentAt(uint16_t start_index, std::vector<uint8_t> message, std::vector<uint8_t> pattern);
	bool IsRS485(uint8_t ID);

	std::vector<uint8_t> protocol_start = PROTOCOL_START;
	std::vector<uint8_t> protocol_end = PROTOCOL_END;
	std::vector<uint8_t> RS485_interfaces;
};
