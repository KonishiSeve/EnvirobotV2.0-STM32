/*
 * Protocol.cpp
 *
 *  Created on: 30 nov. 2022
 *      Author: bignet
 */

#include <Protocols/Communication/Core/CommunicationProtocol.hpp>

/**
 * @brief Add interface ID to the protocol
 *
 * @param input ID_: interface ID to add
 */
void CommunicationProtocol::AddInterfaceID(uint8_t ID_) {
	interfaces_id.push_back(ID_);
}

/**
 * @brief Add the interface allocated buffer and index
 */
void CommunicationProtocol::AddInterfaceBuffer() {
	buffers_index.push_back(0);
	std::vector<uint8_t> buffer(BUFFER_SIZE);
	buffers.push_back(buffer);

	std::vector<uint8_t> data_to_send(BUFFER_SIZE);
	buffers_to_transfer.push_back(data_to_send);
	buffers_to_transfer.back().reserve(BUFFER_SIZE);
}

/**
 * @brief Check whether the interface ID is registered
 *
 * @return true if the interface ID is found
 */
bool CommunicationProtocol::InterfaceExists(uint8_t ID) {
	bool success;
	GetInterfaceIndex(ID, &success);
	return success;
}

/**
 * @brief Find the interface index based on the interface ID
 *
 * @param input ID_: interface ID
 * @param output success: true if interface found
 * @return interface index
 */
uint8_t CommunicationProtocol::GetInterfaceIndex(uint8_t ID_, bool* success) {
	uint8_t counter = 0;
	for (uint8_t ID : interfaces_id) {
		if (ID == ID_) {
			*success = true;
			return counter;
		}
		counter++;
	}
	*success = false;
	return 0;
}

/**
 * @brief WEAK Decode an incomming message
 *
 * @param input ID: interface ID
 * @param input data: message to decode
 * @param output index: processed length in the data
 * @param output success: true if message decoded
 * @return the decoded and parsed message based on the custom protocol
 */
MessageHeader CommunicationProtocol::DecodeProtocol(uint8_t ID, std::vector<uint8_t> data, uint16_t* index, bool* success) {*success = false; MessageHeader empty_message; return empty_message;}

/**
 * @brief WEAK Activate reception of the associated hardware interface
 *
 * @param input ID: interface ID
 * @return the interface access status
 */
HAL_StatusTypeDef CommunicationProtocol::ActivateReception(uint8_t ID) {return HAL_ERROR;}

/**
 * @brief WEAK Send a message over an interface with this protocol
 *
 * @param input ID: message to send
 * @return the interface access status
 */
HAL_StatusTypeDef CommunicationProtocol::Send(MessageHeader message) {return HAL_ERROR;}

/**
 * @brief WEAK Send a message over an interface with this protocol
 *
 * @param input pointer: the hardware interface handle pointer
 * @param output success: whether successful
 * @return the interface ID associated to the input handle
 */
uint8_t CommunicationProtocol::GetID(void* pointer, bool* success) {*success = false; return 0;}

/**
 * @brief Get the interface ID from the interface index
 *
 * @param input index: interface index in the vector
 * @param output success: whether successful
 * @return the associated interface ID
 */
uint8_t CommunicationProtocol::GetIDFromIndex(uint8_t index, bool* success) {
	if (index < interfaces_id.size()) {
		*success = true;
		return interfaces_id[index];
	} else {
		*success = false;
		return 0;
	}
}

/**
 * @brief Get the memory area base pointer allocated to the input interface ID
 *
 * @param input ID: interface ID
 * @param output success: whether successful
 * @return the associated buffer memory base pointer
 */
std::vector<uint8_t>* CommunicationProtocol::GetBuffer(uint8_t ID, bool* success) {
	uint8_t index = GetInterfaceIndex(ID, success);
	if (*success)
		return &buffers[index];
	return NULL;
}

/**
 * @brief Get the pointer of the memory index allocated to the input interface ID
 *
 * @param input ID: interface ID
 * @param output success: whether successful
 * @return the associated buffer index pointer
 */
uint16_t* CommunicationProtocol::GetBufferIndex(uint8_t ID, bool* success) {
	uint8_t index = GetInterfaceIndex(ID, success);
	if (*success)
		return &buffers_index[index];
	return NULL;
}

/**
 * @brief Set the maximum payload length for the protocol
 *
 * @param input length: the maximum payload length
 */
void CommunicationProtocol::SetProtocolPayloadMaxLength(uint16_t length) {
	max_payload_length = length;
}

/**
 * @brief Get the maximum payload length for the protocol
 *
 * @return the maximum payload length of the protocol
 */
uint16_t CommunicationProtocol::GetProtocolPayloadMaxLength(void) {
	return max_payload_length;
}
