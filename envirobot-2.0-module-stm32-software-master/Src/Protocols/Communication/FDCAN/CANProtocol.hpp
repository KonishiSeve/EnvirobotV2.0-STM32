/*
 * CANProtocol.hpp
 *
 *  Created on: 30 nov. 2022
 *      Author: bignet
 */

#pragma once

#include <vector>
#include <stm32h750xx.h>
#include <stm32h7xx_hal.h>

#include "Protocols/Communication/Core/TemplateCommunicationProtocol.hpp"

// CANProtocol class used to interface with every CANFD interface
class CANProtocol : public TemplateCommunicationProtocol<FDCAN_HandleTypeDef> {
public:
	CANProtocol();
	void AddInterface(uint8_t ID, FDCAN_HandleTypeDef* interface);
	HAL_StatusTypeDef ActivateReception(uint8_t ID);
	MessageHeader DecodeProtocol(uint8_t ID, std::vector<uint8_t> message, uint16_t* index, bool* success); //remove the protocol part of the received message
	HAL_StatusTypeDef Send(MessageHeader message);

	HAL_StatusTypeDef ConfigureFilters(uint8_t ID);
	HAL_StatusTypeDef Start(uint8_t ID);

private:
	std::vector<uint8_t> CodeProtocol(MessageHeader message, FDCAN_TxHeaderTypeDef* TxHeader); //add the protocol part to get data to send
	uint32_t GetProtocolLength(uint16_t length);

	uint32_t marker;
	std::vector<FDCAN_TxHeaderTypeDef> FDCANTxHeaders;
};
