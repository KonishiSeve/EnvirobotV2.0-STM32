/*
 * UARTProtocol.cpp
 *
 *  Created on: 30 nov. 2022
 *      Author: bignet
 */

#include <Protocols/Communication/UART/UARTProtocol.hpp>

/**
 * @brief Class constructor
 */
UARTProtocol::UARTProtocol() {

}

/**
 * @brief OVERRIDE Activate reception of the associated hardware interface
 *
 * @param input ID: interface ID
 * @return the interface access status
 */
HAL_StatusTypeDef UARTProtocol::ActivateReception(uint8_t ID) {
	bool success;
	UART_HandleTypeDef* handle = GetInterface(ID, &success);
	if (not success) return HAL_ERROR;
	std::vector<uint8_t>* buffer = GetBuffer(ID, &success);
	if (not success) return HAL_ERROR;

	HAL_StatusTypeDef status;
	status = UART_CheckIdleState(handle); // to reset UART RxState
	do {
		status = HAL_UARTEx_ReceiveToIdle_DMA(handle, buffer->data(), BUFFER_SIZE);
	} while (status == HAL_BUSY);
	return status;
}

/**
 * @brief OVERRIDE Send a message over an interface with this protocol
 *
 * @param input ID: message to send
 * @return the interface access status
 */
HAL_StatusTypeDef UARTProtocol::Send(MessageHeader message) {
	bool success;
	UART_HandleTypeDef* interface_ = GetInterface(message.interface, &success);

	if (!success) return HAL_ERROR;

	if (interface_->hdmatx->State != HAL_DMA_STATE_READY || interface_->gState != HAL_UART_STATE_READY) return HAL_BUSY;

	uint8_t index = GetInterfaceIndex(message.interface, &success);

	buffers_to_transfer[index] = CodeProtocol(message); //std::vector<uint8_t>

	return HAL_UART_Transmit_DMA(interface_, &(buffers_to_transfer[index])[0], buffers_to_transfer[index].size());
}

/**
 * @brief Code a message into a vector of bytes with the protocol
 *
 * @param input message: message to code
 * @return a vector of byte corresponding to the message coded with the protocol
 */
std::vector<uint8_t> UARTProtocol::CodeProtocol(MessageHeader message) {
	std::vector<uint8_t> coded_message;
	coded_message.insert(coded_message.end(), std::begin(protocol_start), std::end(protocol_start));
	if (IsRS485(message.interface)) coded_message.push_back(message.target_address);
	coded_message.push_back(message.source_address);
	coded_message.push_back((uint8_t) message.payload.size());
	coded_message.insert(coded_message.end(), message.payload.begin(), message.payload.end());
	coded_message.push_back(GetChecksum(coded_message));
	coded_message.insert(coded_message.end(), std::begin(protocol_end), std::end(protocol_end));
	return coded_message;
}

/**
 * @brief OVERRIDE Decode an incomming message
 *
 * @param input ID: interface ID
 * @param input data: message to decode
 * @param output index: processed length in the data
 * @param output success: true if message decoded
 * @return the decoded and parsed message based on the custom protocol
 */
MessageHeader UARTProtocol::DecodeProtocol(uint8_t ID, std::vector<uint8_t> message, uint16_t* index, bool* success) {
	MessageHeader decoded_message;

	uint16_t RS485_offset = 0;
	if (IsRS485(ID)) RS485_offset = 1;

	uint16_t start_index = DetectPattern(message, protocol_start, *index, success);
	if (*success) {
		// Check message length
		if (message.size() <= start_index + protocol_start.size() + 1 + RS485_offset) {*success = false; return decoded_message;} // Error - discard message

		// Retrieve header information
		if (IsRS485(ID))
			decoded_message.target_address = message[start_index + protocol_start.size()];
		else
			decoded_message.target_address = ALL;
		decoded_message.source_address = message[start_index + protocol_start.size() + RS485_offset];
		uint16_t length = message[start_index + protocol_start.size() + 1 + RS485_offset];

		// Check message length
		if (message.size() <= start_index + protocol_start.size() + length + 2 + RS485_offset + protocol_end.size()) {*success = false; return decoded_message;} // Error - discard message

		// Retrieve payload based on length byte
		decoded_message.payload = std::vector<uint8_t>(message.begin() + start_index + protocol_start.size() + 2 + RS485_offset, message.begin() + start_index + protocol_start.size() + 2 + RS485_offset + length);

		// Checksum
		if (!Checksum(start_index, start_index + protocol_start.size() + length + 3 + RS485_offset, message)) {*success = false; return decoded_message;} // Checksum Error - discard message

		// Check end of protocol
		if (!PatternPresentAt(start_index + protocol_start.size() + length + 3 + RS485_offset, message, protocol_end)) {*success = false; return decoded_message;} // Checksum Error - discard message

		*success = true;
		*index = start_index + protocol_start.size() + length + 3 + RS485_offset + protocol_end.size();
	}
	return decoded_message;
}

/**
 * @brief Detect the pattern sequence in a message as of a start index
 *
 * @param input message: message used to find the pattern
 * @param input pattern: patter to detect
 * @param input start_index: the detection of the pattern starts at this index
 * @param output success: whether successful
 * @return the base index of the first iteration of the detected pattern in message
 */
uint16_t UARTProtocol::DetectPattern(std::vector<uint8_t> message, std::vector<uint8_t> pattern, uint16_t start_index, bool* success) {
	uint8_t counter = 0;
	for (uint16_t index = start_index; index < message.size(); index++) {
		if (message[index] != pattern[counter]) {
			counter = 0;
			continue;
		}
		counter++;
		if (counter >= pattern.size()) {
			*success = true;
			return index - counter + 1;
		}
	}
	*success = false;
	return 0;
}

/**
 * @brief Detect if a pattern sequence is present in a message as of a start index
 *
 * @param input start_index: the detection of the pattern starts at this index
 * @param input message: message used to find the pattern
 * @param input pattern: patter to detect
 * @param output success: true if pattern detected
 */
bool UARTProtocol::PatternPresentAt(uint16_t start_index, std::vector<uint8_t> message, std::vector<uint8_t> pattern) {
    uint16_t index = start_index;
    for (uint8_t byte : pattern) {
        if (byte != message[index])
            return false;
        index++;
    }
    return true;
}

/**
 * @brief Compute the 1-byte checksum of a message
 *
 * @param input message: vector of bytes to derive the checksum
 * @return the 1-byte checksum
 */
uint8_t UARTProtocol::GetChecksum(std::vector<uint8_t> message) {
	uint8_t checksum = 0;
	for (uint8_t byte : message)
		checksum += byte;
	checksum = ~checksum + 1;
	return checksum;
}

/**
 * @brief Check the checksum
 *
 * @param input start_index: index (included) from which the checksum check is started
 * @param input end_index: index (excluded) from which the checksum check is finished
 * @param input message: byte vector message
 * @return true if the checksum is valid
 */
bool UARTProtocol::Checksum(uint16_t start_index, uint16_t end_index, std::vector<uint8_t> message) {
	uint8_t sum = 0;
	if (start_index > message.size()) return false;
	if (end_index > message.size()) return false;
	if (start_index > end_index) return false;
	for (uint16_t index = start_index; index < end_index; index++) {
		sum += message[index];
	}
	return (sum == 0);
}

/**
 * @brief Set the interface with ID as RS485. Adds the target address in the protocol
 *
 * @param input ID: interface ID to set as RS485
 * @return whether successful
 */
bool UARTProtocol::SetAsRS485(uint8_t ID) {
	if (IsRS485(ID)) return false;

	RS485_interfaces.push_back(ID);
	return true;
}

/**
 * @brief Check if the interface with ID is RS485
 *
 * @param input ID: interface ID
 * @return true if interface is RS485
 */
bool UARTProtocol::IsRS485(uint8_t ID) {
	for (uint8_t &RS485_interface : RS485_interfaces) {
		if (RS485_interface == ID) return true;
	}
	return false;
}
