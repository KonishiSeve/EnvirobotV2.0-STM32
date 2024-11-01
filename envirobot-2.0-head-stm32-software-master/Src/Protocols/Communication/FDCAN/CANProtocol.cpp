/*
 * CANProtocol.cpp
 *
 *  Created on: 30 nov. 2022
 *      Author: bignet
 */

#include <Protocols/Communication/FDCAN/CANProtocol.hpp>

/**
 * @brief Class constructor
 */
CANProtocol::CANProtocol() {
	marker = 0;
}

/**
 * @brief OVERRIDE Add an hardware interface with a specific ID
 *
 * @param input ID: interface ID
 * @param input interface: the pointer to the hardware interface handle
 */
void CANProtocol::AddInterface(uint8_t ID, FDCAN_HandleTypeDef* interface) {
	AddInterfaceID(ID);
	AddInterfaceBuffer();
	interfaces.push_back(interface);

	// Setup interface
	FDCAN_TxHeaderTypeDef FDCANTxHeader;
	FDCANTxHeader.IdType = FDCAN_STANDARD_ID; 				// FDCAN_STANDARD_ID or FDCAN_CLASSIC_CAN
	FDCANTxHeader.TxFrameType = FDCAN_DATA_FRAME; 			// FDCAN_DATA_FRAME or FDCAN_REMOTE_FRAME
	FDCANTxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE; 	// FDCAN_ESI_PASSIVE or FDCAN_ESI_ACTIVE
	FDCANTxHeader.BitRateSwitch = FDCAN_BRS_ON; 				// FDCAN_BRS_ON FDCAN_BRS_OFF
	FDCANTxHeader.FDFormat = FDCAN_FD_CAN; 					// FDCAN_FD_CAN or FDCAN_CLASSIC_CAN
	FDCANTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; 	// FDCAN_NO_TX_EVENTS or FDCAN_STORE_TX_EVENTS

	// Add interface
	FDCANTxHeaders.push_back(FDCANTxHeader);
}

/**
 * @brief Setup the CANFD filters
 *
 * @param input ID: interface ID
 * @return the interface access status
 */
HAL_StatusTypeDef CANProtocol::ConfigureFilters(uint8_t ID) {
	bool success;
	FDCAN_HandleTypeDef* handle = GetInterface(ID, &success);
	if (not success) return HAL_ERROR;
	return HAL_FDCAN_ConfigGlobalFilter(handle, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
}

/**
 * @brief OVERRIDE Activate reception of the associated hardware interface
 *
 * @param input ID: interface ID
 * @return the interface access status
 */
HAL_StatusTypeDef CANProtocol::ActivateReception(uint8_t ID) {
	bool success;
	FDCAN_HandleTypeDef* handle = GetInterface(ID, &success);
	if (not success) return HAL_ERROR;
	return HAL_FDCAN_ActivateNotification(handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/**
 * @brief Start the CANFD interface
 *
 * @param input ID: interface ID
 * @return the interface access status
 */
HAL_StatusTypeDef CANProtocol::Start(uint8_t ID) {
	bool success;
	FDCAN_HandleTypeDef* handle = GetInterface(ID, &success);
	if (not success) return HAL_ERROR;
	return HAL_FDCAN_Start(handle);
}

/**
 * @brief OVERRIDE Send a message over an interface with this protocol
 *
 * @param input ID: message to send
 * @return the interface access status
 */
HAL_StatusTypeDef CANProtocol::Send(MessageHeader message) {
	bool success;
	FDCAN_HandleTypeDef* interface_ = GetInterface(message.interface, &success);

	if (!success) return HAL_ERROR;

	if (interface_->State != HAL_FDCAN_STATE_BUSY) return HAL_ERROR;

	uint8_t index = GetInterfaceIndex(message.interface, &success);
	FDCAN_TxHeaderTypeDef TxHeader = FDCANTxHeaders[index];
	buffers_to_transfer[index] = CodeProtocol(message, &TxHeader);

	if (buffers_to_transfer[index].size() > 64) return HAL_ERROR;

	return HAL_FDCAN_AddMessageToTxFifoQ(interface_, &TxHeader, &(buffers_to_transfer[index])[0]);
}

/**
 * @brief Code a message into a vector of bytes with the protocol
 *
 * @param input message: message to code
 * @param output TxHeader: the CANFD header that will includes the coded message
 * @return a vector of byte corresponding to the message coded with the protocol
 */
std::vector<uint8_t> CANProtocol::CodeProtocol(MessageHeader message, FDCAN_TxHeaderTypeDef* TxHeader) {
	TxHeader->Identifier = message.target_address;
	TxHeader->MessageMarker = marker;
	if (marker >= 4294967295)
		marker = 0;
	else
		marker++;

	std::vector<uint8_t> coded_message;
	coded_message.push_back(message.source_address);
	coded_message.push_back((uint8_t) message.payload.size());
	coded_message.insert(coded_message.end(), message.payload.begin(), message.payload.end());

	TxHeader->DataLength = GetProtocolLength(coded_message.size());

	return coded_message;
}

/**
 * @brief Get the smallest pre-defined DLC value based on the payload length
 *
 * @param input length: the CAN payload length
 * @return the minimum DLC value that fits the given length inside
 */
uint32_t CANProtocol::GetProtocolLength(uint16_t length) {
	if (length <= 0) return FDCAN_DLC_BYTES_0;
	if (length <= 1) return FDCAN_DLC_BYTES_1;
	if (length <= 2) return FDCAN_DLC_BYTES_2;
	if (length <= 3) return FDCAN_DLC_BYTES_3;
	if (length <= 4) return FDCAN_DLC_BYTES_4;
	if (length <= 5) return FDCAN_DLC_BYTES_5;
	if (length <= 6) return FDCAN_DLC_BYTES_6;
	if (length <= 7) return FDCAN_DLC_BYTES_7;
	if (length <= 8) return FDCAN_DLC_BYTES_8;
	if (length <= 12) return FDCAN_DLC_BYTES_12;
	if (length <= 16) return FDCAN_DLC_BYTES_16;
	if (length <= 20) return FDCAN_DLC_BYTES_20;
	if (length <= 24) return FDCAN_DLC_BYTES_24;
	if (length <= 32) return FDCAN_DLC_BYTES_32;
	if (length <= 48) return FDCAN_DLC_BYTES_48;
	if (length <= 64) return FDCAN_DLC_BYTES_64;
	return FDCAN_DLC_BYTES_64;
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
MessageHeader CANProtocol::DecodeProtocol(uint8_t ID, std::vector<uint8_t> message, uint16_t* index, bool* success) {
	MessageHeader decoded_message;

	decoded_message.target_address = message[0];
	decoded_message.source_address = message[1];
	uint16_t length = message[2];

	// Retrieve payload based on length byte
	decoded_message.payload = std::vector<uint8_t>(message.begin() + 3, message.begin() + 3 + length);

	*success = true;
	*index = message.size();

	return decoded_message;
}
