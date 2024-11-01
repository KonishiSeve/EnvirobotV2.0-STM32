/*
 * Communication.cpp
 *
 *  Created on: 30 nov. 2022
 *      Author: bignet
 */

#include <Communication/Communication.hpp>
#include <Services/Services.hpp>

/**
 * @brief class Constructor
 */
Communication::Communication() {
	address = UNKNOWN;
}

/**
 * @brief Initialize the class with references to other classes
 *
 * @param input registers_: the Registers instance
 * @param input services_: the Services instance
 * @param input subscribers_: the MasterSubscribers instance
 * @param input leds_: the LEDS instance
 */
void Communication::Init(Registers* registers_, Services* services_, MasterSubscribers* subscribers_, LEDS* leds_) {
	registers = registers_;
	services = services_;
	subscribers = subscribers_;
	leds = leds_;

	CommunicationSemaphore = osSemaphoreNew(1,1,NULL);
	osSemaphoreRelease(CommunicationSemaphore);
}

/**
 * @brief Add class related registers
 */
void Communication::AddRegisters(void) {
	// Register to set module address and propagates to the next module
	registers->AddRegister<uint8_t>(REG_COM_ID_PROPAGATION);
	registers->SetRegisterAsSingle(REG_COM_ID_PROPAGATION);
	registers->AddWriteCallback<uint8_t>(REG_COM_ID_PROPAGATION, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Communication* self = (Communication*) context;
			bool success;

			// Set module address and update leds to tell the user ID has been correctly assigned
			self->SetModuleAddress(*input);
			IDFoundLEDS(self->leds);

			// Services not used here because don't wait for a feedback, there is possibly no module behind so do not accumulate timeouts
			MessageHeader message;
			message.interface = UART_BACK;

			MessageElement element;
			element.ack = false;
			element.access = WRITE;
			element.command = true;
			element.data.push_back(*input + 1);
			element.element_register = self->registers->FindRegister(REG_COM_ID_PROPAGATION, &success);

			self->BuildPayload(&message, element);

			self->Send(message);

			return true;
		}
	);

	// Register to set module address
	registers->AddRegister<uint8_t>(REG_COM_ADDRESS);
	registers->SetRegisterAsSingle(REG_COM_ADDRESS);
	registers->AddRegisterSemaphore(REG_COM_ADDRESS, &CommunicationSemaphore);
	registers->AddRegisterPointer(REG_COM_ADDRESS, &address);

	// Register to add a group address
	registers->AddRegister<uint8_t>(REG_COM_ADD_GROUP_ADDRESS);
	registers->SetRegisterAsSingle(REG_COM_ADD_GROUP_ADDRESS);
	registers->AddWriteCallback<uint8_t>(REG_COM_ADD_GROUP_ADDRESS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Communication* self = (Communication*) context;
			return self->AddGroupAddress(*input);
		}
	);

	// Register to remove a specific address from the group address list
	registers->AddRegister<uint8_t>(REG_COM_REMOVE_GROUP_ADDRESS);
	registers->SetRegisterAsSingle(REG_COM_REMOVE_GROUP_ADDRESS);
	registers->AddWriteCallback<uint8_t>(REG_COM_REMOVE_GROUP_ADDRESS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Communication* self = (Communication*) context;
			return self->RemoveGroupAddress(*input);
		}
	);

	// Register to clear the list of group address
	registers->AddRegister<uint8_t>(REG_COM_CLEAR_GROUP_ADDRESS);
	registers->SetRegisterAsSingle(REG_COM_CLEAR_GROUP_ADDRESS);
	registers->AddWriteCallback<uint8_t>(REG_COM_CLEAR_GROUP_ADDRESS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Communication* self = (Communication*) context;
			return self->ClearGroupAddress();
		}
	);

	// Register to get the list of group address
	registers->AddRegister<uint8_t>(REG_COM_GET_GROUP_ADDRESS);
	registers->SetRegisterAsVector(REG_COM_GET_GROUP_ADDRESS);
	registers->AddVectorRegisterPointer<uint8_t>(REG_COM_GET_GROUP_ADDRESS, &group_addresses);
	registers->SetRegisterPermissions(REG_COM_GET_GROUP_ADDRESS, READ_PERMISSION);

	// Register to forward the payload to a specific interface
	// payload = [interface, target_address, payload to forward]
	registers->AddRegister<uint8_t>(REG_COM_FORWARD_MESSAGE);
	registers->SetRegisterAsVector(REG_COM_FORWARD_MESSAGE);
	registers->AddWriteCallback<uint8_t>(REG_COM_FORWARD_MESSAGE, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Communication* self = (Communication*) context;
			if (length < 3) return false;

			MessageHeader msg;
			msg.interface = input[0];
			msg.target_address = input[1];
			msg.payload = std::vector<uint8_t> (&input[2], &input[length]);
			return self->Send(msg);
		}
	);
}

/**
 * @brief Function called repeatedly in a freeRTOS task to send pending messages that could not have been successfully sent before
 */
void Communication::Spin(void) {
	bool success;
	uint8_t index = 0;
	osSemaphoreAcquire(CommunicationSemaphore, osWaitForever);

	// Try to send pending messages
	while (index < pending_messages.size()) {
		MessageHeader message = pending_messages[index];
		CommunicationProtocol* protocol = FindProtocol(message.interface, &success);

		if (!success)
			pending_messages.erase(pending_messages.begin() + index);

		message.source_address = address; // set the source address as the module address
		HAL_StatusTypeDef status = protocol->Send(message);

		if (status == HAL_OK || status == HAL_ERROR)
			pending_messages.erase(pending_messages.begin() + index);
		else
			index++;
	}

	osSemaphoreRelease(CommunicationSemaphore);

	osDelay(DEFAULT_COMMUNICATION_PERIOD);
}

/**
 * @brief Add a communication protocol to the communication interfaces.
 *
 * @param input protocol: the user defined class derived from CommunicationProtocol. It interfaces with the hardware, code and decode a specific protocol.
 */
void Communication::AddProtocol(CommunicationProtocol* protocol) {
	osSemaphoreAcquire(CommunicationSemaphore, osWaitForever);
	protocols.push_back(protocol);
	osSemaphoreRelease(CommunicationSemaphore);
}

/**
 * @brief Return the pointer of the protocol with ID
 *
 * @param input ID: the protocol ID to find
 * @param output success: true if protocol found
 * @return the protocol pointer
 */
CommunicationProtocol* Communication::FindProtocol(uint8_t ID, bool* success) {
	for (CommunicationProtocol* &protocol : protocols) {
		if (protocol->InterfaceExists(ID)) {
			CommunicationProtocol* protocol_pointer = protocol;
			*success = true;
			return protocol_pointer;
		}
	}
	*success = false;
	return NULL;
}

/**
 * @brief Send a message according to the details in MessageHeader
 *
 * @param input message:
 */
HAL_StatusTypeDef Communication::Send(MessageHeader message) {
	bool success;
	CommunicationProtocol* protocol = FindProtocol(message.interface, &success);
	if (!success) return HAL_ERROR;

	message.source_address = address; // set the source address as the module address

#ifdef USE_COM_LED_SEND
	CommunicationLED(leds, message.interface);
#endif

	osSemaphoreAcquire(CommunicationSemaphore, osWaitForever);
	HAL_StatusTypeDef status = protocol->Send(message);
	if (status == HAL_BUSY || status == HAL_TIMEOUT) {
		pending_messages.push_back(message);
	}
	osSemaphoreRelease(CommunicationSemaphore);

	return status;
}

/**
 * @brief Process an incomming message. Called by hardware interface ISR after reception.
 *
 * @param input ID: the protocol ID related to the incomming message
 * @param input data: vector of bytes representing the received payload
 * @param output success: true if message correctly processed
 * @return the successfully processed length of the vector data
 */
uint16_t Communication::Process(uint8_t ID, std::vector<uint8_t> data, bool* success) {
	uint16_t processed_length = 0;
	CommunicationProtocol* protocol = FindProtocol(ID, success);
	if (not *success)
		return 0;

	// TODO process until not success
	osSemaphoreAcquire(CommunicationSemaphore, osWaitForever); // TODO ADDED
	while (processed_length < data.size()) {// && processed_length < data.size()) {
		MessageHeader message = protocol->DecodeProtocol(ID, data, &processed_length, success);
		if (not *success) break;

		if (!AcceptMessage(message)) {
			*success = false;
			break;
		}

		message.interface = ID;
		ParseMessage(message);
	}
	osSemaphoreRelease(CommunicationSemaphore);  // TODO ADDED
	return processed_length;
}

/**
 * @brief Check the message is addressed to the module.
 *
 * @param input message: message to filter
 * @return true if the module address fits the message target address, if the target address is part of registered group addresses or if the target address targets every module (ALL)
 */
bool Communication::AcceptMessage(MessageHeader message) {
	return (message.target_address == ALL || message.target_address == address || InGroupAddresses(message.target_address));
}

/**
 * @brief Check that the input address is part of the registered group addresses
 *
 * @param input address: a 8-bit address
 * @return true if the input adddress is part of the group addresses
 */
bool Communication::InGroupAddresses(uint8_t address) {
	for (uint8_t group_address : group_addresses) {
		if (group_address == address) return true;
	}
	return false;
}

/**
 * @brief Find the registered interface ID that goes with the input hardware interface handle
 *
 * @param input handle: base pointer of an hardware interface
 * @param output success: return true if the protocol has been successfully found
 * @return the registered interface ID of the input hardware handle
 */
uint8_t Communication::GetID(void* handle, bool* success) {
	uint8_t ID;
	for (CommunicationProtocol* protocol : protocols) {
		ID = protocol->GetID(handle, success);
		if (*success)
			return ID;
	}
	return 0;
}

/**
 * @brief Get access to the memory area allocated to the hardware interface
 *
 * @param input ID: the interface ID
 * @param output success: return true if the memory pointer successfully found
 * @return the memory pointer allocated to the hardware interface
 */
std::vector<uint8_t>* Communication::GetBuffer(uint8_t ID, bool* success) {
	CommunicationProtocol* protocol = FindProtocol(ID, success);
	if (*success) {
		std::vector<uint8_t>* buffer = protocol->GetBuffer(ID, success);
		if (*success)
			return buffer;
	}
}

/**
 * @brief Get access to the variable that stores the current index in the buffer memory area
 *
 * @param input ID: the interface ID
 * @param output success: return true if the index has been successfully found
 * @return the pointer to the index variable
 */
uint16_t* Communication::GetBufferIndex(uint8_t ID, bool* success) {
	CommunicationProtocol* protocol = FindProtocol(ID, success);
	if (*success) {
		uint16_t* buffer_index = protocol->GetBufferIndex(ID, success);
		if (*success)
			return buffer_index;
	}
	return NULL;
}

/**
 * @brief Get the maximum allowed payload length for a given protocol
 *
 * @param input ID: the protocol ID
 * @param output success: return true if protocol found
 * @return the maximum payload size of the input protocol
 */
uint16_t Communication::GetProtocolPayloadMaxLength(uint8_t ID, bool* success) {
	CommunicationProtocol* protocol = FindProtocol(ID, success);
	if (not *success)
		return 0;

	return protocol->GetProtocolPayloadMaxLength();
}

/**
 * @brief Activate the hardware reading of the input protocol
 *
 * @param input ID: the protocol ID
 * @return the status of the reading activation request
 */
HAL_StatusTypeDef Communication::ActivateReception(uint8_t ID) {
	bool success;
	CommunicationProtocol* protocol = FindProtocol(ID, &success);
	if (not success) return HAL_ERROR;
	return protocol->ActivateReception(ID);
}

/**
 * @brief Fills the payload of a message based on the input element
 *
 * @param output message
 * @param input element: the register access to put into the message
 * @return the function success
 */
bool Communication::BuildPayload(MessageHeader* message, MessageElement element) {
	std::vector<uint8_t> coded_element;
	uint8_t header = 0;
	if (element.ack) header|= ACK_MASK;
	if (element.command) header|= COMMAND_MASK;
	if (element.access) header|= ACCESS_MASK;

	coded_element.push_back(header | ((element.element_register.address >> 8) & ~(ACK_MASK | COMMAND_MASK | ACCESS_MASK)));
	coded_element.push_back(element.element_register.address & 0xFF);

	// push vector size
	if (element.element_register.isArray && element.element_register.length == 0)
		coded_element.push_back(element.data.size());

	coded_element.insert(coded_element.end(), element.data.begin(), element.data.end()); // Big-Endian representation, MSB first

	// Check there is enough space in the payload
	bool success;
	uint16_t max_length = GetProtocolPayloadMaxLength(message->interface, &success);

	if (not success) return false;

	if (message->payload.size() + coded_element.size() > max_length)
		return false;

	message->payload.insert(message->payload.end(), coded_element.begin(), coded_element.end());
	return true;
}

/**
 * @brief Parse an incomming message. Read the header and extract the register accesses in the payload and finally process every access
 *
 * @param input message: message to parse
 */
void Communication::ParseMessage(MessageHeader message) {
	MessageElement element;
	MessageHeader response;

	response.interface = message.interface;
	response.target_address = message.source_address;

	uint16_t index = 0;
	while (index < message.payload.size()) {

		// Check Length
		if (index + 1 >= (uint16_t) message.payload.size())
			break;

		// Extract header
		element.ack = message.payload[index] & ACK_MASK;
		element.command = message.payload[index] & COMMAND_MASK;
		element.access = message.payload[index] & ACCESS_MASK;

		// Extract Register
		element.element_register.address = (message.payload[index] & ~(ACK_MASK | COMMAND_MASK | ACCESS_MASK)) << 8 | message.payload[index + 1];

		// Find Register information
		bool success = FindRegister(message, &response, &element);
		if (not success) break;

		// Process the register access
		switch (element.element_register.type) {
#if defined(USE_UINT8_COMMUNICATION) && defined(USE_UINT8_REGISTER)
		case UINT8_TYPE:
			index = ProcessElement<uint8_t>(message, &response, element, index + 2);
			break;
#endif
#if defined(USE_UINT16_COMMUNICATION) && defined(USE_UINT16_REGISTER)
		case UINT16_TYPE:
			index = ProcessElement<uint16_t>(message, &response, element, index + 2);
			break;
#endif
#if defined(USE_UINT32_COMMUNICATION) && defined(USE_UINT32_REGISTER)
		case UINT32_TYPE:
			index = ProcessElement<uint32_t>(message, &response, element, index + 2);
			break;
#endif
#if defined(USE_UINT64_COMMUNICATION) && defined(USE_UINT64_REGISTER)
		case UINT64_TYPE:
			index = ProcessElement<uint64_t>(message, &response, element, index + 2);
			break;
#endif
#if defined(USE_INT8_COMMUNICATION) && defined(USE_INT8_REGISTER)
		case INT8_TYPE:
			index = ProcessElement<int8_t>(message, &response, element, index + 2);
			break;
#endif
#if defined(USE_INT16_COMMUNICATION) && defined(USE_INT16_REGISTER)
		case INT16_TYPE:
			index = ProcessElement<int16_t>(message, &response, element, index + 2);
			break;
#endif
#if defined(USE_INT32_COMMUNICATION) && defined(USE_INT32_REGISTER)
		case INT32_TYPE:
			index = ProcessElement<int32_t>(message, &response, element, index + 2);
			break;
#endif
#if defined(USE_INT64_COMMUNICATION) && defined(USE_INT64_REGISTER)
		case INT64_TYPE:
			index = ProcessElement<int64_t>(message, &response, element, index + 2);
			break;
#endif
#if defined(USE_FLOAT_COMMUNICATION) && defined(USE_FLOAT_REGISTER)
		case FLOAT_TYPE:
			index = ProcessElement<float>(message, &response, element, index + 2);
			break;
#endif
#if defined(USE_DOUBLE_COMMUNICATION) && defined(USE_DOUBLE_REGISTER)
		case DOUBLE_TYPE:
			index = ProcessElement<double>(message, &response, element, index + 2);
			break;
#endif
		default:
			index += 2;
			break;
		}
	}

	// Send a response if there is any
	if (response.payload.size() > 0)
		Send(response);
}

/**
 * @brief Find a register based on the access, depending whether it is register access, publisher or action acknowledgment
 *
 * @param input message: stores the information related to the register access
 * @param output response: response message filled in case of register access
 * @param output element: the element that stores the found register
 * @return whether successfull
 */
bool Communication::FindRegister(MessageHeader message, MessageHeader* response, MessageElement* element) {
	bool success;
	uint16_t register_id = element->element_register.address;

	// Command access
	if (not element->ack && element->command) {
		if (registers->IsRegistered(register_id))
			element->element_register = registers->FindRegister(register_id, &success);
		else {
			MessageElement response_content;
			response_content.element_register.address = register_id;
			response_content.ack = true;
			response_content.command = true;
			response_content.access = element->access;
			response_content.data = std::vector<uint8_t>{UNKNOWN_REGISTER};

			// if no space available, send the data and then rebuild a new response
			if (not BuildPayload(response, response_content)) {
				Send(*response);
				response->payload.clear();
				BuildPayload(response, response_content);
			}

			return false;
		}
	// Action feedback
	} else if (element->ack) {
		element->element_register = services->FindRemoteRegister(Register{.address=register_id}, ServiceInterface{.interface=message.interface, .address=message.source_address}, element->access, &success);
	// Data from a publisher, send to Subscribers
	} else if (not element->ack && not element->command) {
		element->element_register = subscribers->FindRemoteRegister(Register{.address=register_id}, SubscriberInterface{.interface=message.interface, .address=message.source_address}, &success);
	} else return false;

	return success;
}

/**
 * @brief Process an element from a message
 *
 * @param input message
 * @param output response: response message filled if necessary
 * @param input element: the element to process
 * @param input start_index: the current index in message payload
 * @return the process status
 */
template<typename T>
uint16_t Communication::ProcessElement(MessageHeader message, MessageHeader* response, MessageElement element, uint16_t start_index) {
	if (not element.ack && element.command) {
		return ProcessCommand<T>(message, response, element, start_index);
	} else if (element.ack) {
		return ProcessService<T>(message, element, start_index);
	} else {
		return ProcessSubscriber<T>(message, element, start_index);
	}
}

/**
 * @brief Process a command access (no ack and cmd bits)
 *
 * @param input message
 * @param output response: response message filled if necessary
 * @param input element: the element to process
 * @param input start_index: the current index in message payload
 * @return the process status
 */
template<typename T>
uint16_t Communication::ProcessCommand(MessageHeader message, MessageHeader* response, MessageElement element, uint16_t start_index) {
	MessageElement response_content;
	response_content.element_register = element.element_register;
	response_content.ack = true;
	response_content.command = false;
	response_content.access = element.access;

#ifdef USE_COM_LED_REGISTER_ACCESS
	CommunicationLED(leds, message.interface);
#endif

	// send to registers
	if (element.access == WRITE) {
		return ProcessWrite<T>(message, response, response_content, element, start_index);
	} else if (element.access == READ) { // if read command, there is no data to extract from the input message
		return ProcessRead<T>(message, response, response_content, element, start_index);
	}

	return start_index;
}

/**
 * @brief Process a register write access
 *
 * @param input message
 * @param output response: response message filled if necessary
 * @param input response_content: a pre-filled response element to store the access feedback that is pushed into the response message in the end of the function
 * @param input element: the element to process
 * @param input start_index: the current index in message payload
 * @return the process status
 */
template<typename T>
uint16_t Communication::ProcessWrite(MessageHeader message, MessageHeader* response, MessageElement response_content, MessageElement element, uint16_t start_index) {
	bool success;
	uint16_t length_to_extract = 0;

	// Extract value from payload
	if (not element.element_register.isArray) {
		length_to_extract = 1;
		T value = ExtractSingle<T>(message.payload, start_index, &success);
		if (success)
			success = registers->WriteRegister<T>(element.element_register.address, &value);
	} else if (element.element_register.isArray && element.element_register.length > 0) {
		length_to_extract = element.element_register.length;
		T values[length_to_extract];
		ExtractArray<T>(values, message.payload, start_index, length_to_extract, &success);
		if (success)
			success = registers->WriteRegister<T>(element.element_register.address, values);
	} else if (element.element_register.isArray && element.element_register.length == 0) {
		length_to_extract = message.payload[start_index];
		start_index++;
		std::vector<T> values = ExtractVector<T> (message.payload, start_index, length_to_extract, &success);
		if (success)
			success = registers->WriteVectorRegister<T>(element.element_register.address, values); // previously &values
	}

	// Set the acknowledgment
	if (success)
		response_content.data = std::vector<uint8_t>{OK};
	else
		response_content.data = std::vector<uint8_t>{ERROR};

	// if no space available, send the data and then rebuild a new response
	if (not BuildPayload(response, response_content)) {
		Send(*response);
		response->payload.clear();
		BuildPayload(response, response_content);
	}

	return start_index + sizeof(T) * length_to_extract;
}

/**
 * @brief Process a register read access
 *
 * @param input message
 * @param output response: response message filled if necessary
 * @param input response_content: a pre-filled response element to store the access feedback that is pushed into the response message in the end of the function
 * @param input element: the element to process
 * @param input start_index: the current index in message payload
 * @return the process status
 */
template<typename T>
uint16_t Communication::ProcessRead(MessageHeader message, MessageHeader* response, MessageElement response_content, MessageElement element, uint16_t start_index) {
	bool success;
	response_content.data = ReadElement<T>(&element, &success);

	if (not success) {
		response_content.command = true; // this is an error flag for a read ack
		response_content.data = std::vector<uint8_t>{ERROR};
	}

	if (not BuildPayload(response, response_content) && response->payload.size() > 0) { // if no space available, send the data and then rebuild a new response
		Send(*response);
		response->payload.clear();
		BuildPayload(response, response_content);
	}

	return start_index;
}

/**
 * @brief Process a service feedback. No response expected here.
 *
 * @param input message
 * @param input element: the element to process
 * @param input start_index: the current index in message payload
 * @return the process status
 */
template<typename T>
uint16_t Communication::ProcessService(MessageHeader message, MessageElement element, uint16_t start_index) {
	ServiceConfiguration service_information;
	service_information.register_ = element.element_register;
	service_information.interface = ServiceInterface{.interface=message.interface, .address=message.source_address};
	service_information.access = element.access;
	service_information.length = 1;

	bool success;
	// Error reception
	if (element.command) {
		uint8_t error = ExtractSingle<uint8_t>(message.payload, start_index, &success); // extract the feedback message
		if (success)
			services->ReceiveError(service_information, error);//services->ReceiveError(element.element_register, message.interface, message.source_address, element.access, error);
		return start_index + 1;
	}
	// Write reception
	if (element.access == WRITE) {
		uint8_t value = ExtractSingle<uint8_t>(message.payload, start_index, &success); // extract the feedback message
		if (success)
			services->ReceiveWrite(service_information, value);//services->ReceiveWrite(element.element_register, message.interface, message.source_address, value);
		return start_index + 1;
	}
	// Read reception
	else if (element.access == READ) {

		uint16_t length_to_extract = 0;

		// Extract value from payload
		T single_value;
		T array_values[element.element_register.length];
		std::vector<T> vector_values;
		T* data;

		bool success;
		if (not element.element_register.isArray) {
			length_to_extract = 1;
			single_value = ExtractSingle<T>(message.payload, start_index, &success);
			data = &single_value;
		} else if (element.element_register.isArray && element.element_register.length > 0) {
			length_to_extract = element.element_register.length;
			ExtractArray<T>(array_values, message.payload, start_index, length_to_extract, &success);
			data = array_values;
		} else if (element.element_register.isArray && element.element_register.length == 0) {
			length_to_extract = message.payload[start_index];
			start_index++;
			vector_values = ExtractVector<T> (message.payload, start_index, length_to_extract, &success);
			data = vector_values.data();
		}

		service_information.length = length_to_extract;

		if (success)
			services->ReceiveRead<T>(service_information, data);

		return start_index + sizeof(T) * length_to_extract;
	}
}

/**
 * @brief Process a publisher input to be forwarded to publishers. No response message expected here.
 *
 * @param input message
 * @param input element: the element to process
 * @param input start_index: the current index in message payload
 * @return the process status
 */
template<typename T>
uint16_t Communication::ProcessSubscriber(MessageHeader message, MessageElement element, uint16_t start_index) {
	uint16_t length_to_extract = 0;

	// Extract value from payload
	T single_value;
	T array_values[element.element_register.length];
	std::vector<T> vector_values;
	T* data;

	bool success;
	if (not element.element_register.isArray) {
		length_to_extract = 1;
		single_value = ExtractSingle<T>(message.payload, start_index, &success);
		data = &single_value;
	} else if (element.element_register.isArray && element.element_register.length > 0) {
		length_to_extract = element.element_register.length;
		ExtractArray<T>(array_values, message.payload, start_index, length_to_extract, &success);
		data = array_values;
	} else if (element.element_register.isArray && element.element_register.length == 0) { //TODO length never == 0
		length_to_extract = message.payload[start_index];
		start_index++;
		vector_values = ExtractVector<T> (message.payload, start_index, length_to_extract, &success);
		data = vector_values.data();
	}

	SubscriberInput subscriber_information;
	subscriber_information.interface.interface = message.interface;
	subscriber_information.interface.address = message.source_address;
	subscriber_information.register_ = element.element_register;
	subscriber_information.length = length_to_extract;

	subscribers->Receive<T>(subscriber_information, data);

	return start_index + sizeof(T) * length_to_extract;
}

/**
 * @brief Set the module address
 *
 * @param input address_: the new module address
 * @return whether successful
 */
bool Communication::SetModuleAddress(uint8_t address_) {
	osSemaphoreAcquire(CommunicationSemaphore, osWaitForever);
	address = address_;
	osSemaphoreRelease(CommunicationSemaphore);
	return true;
}

/**
 * @brief Add a group address
 *
 * @param input address: a new group address
 * @return whether successful
 */
bool Communication::AddGroupAddress(uint8_t address) {
	osSemaphoreAcquire(CommunicationSemaphore, osWaitForever);
	group_addresses.push_back(address);
	osSemaphoreRelease(CommunicationSemaphore);
	return true;
}

/**
 * @brief Remove a group address
 *
 * @param input address_: the group address to remove
 * @return whether successful
 */
bool Communication::RemoveGroupAddress(uint8_t address_) {
	osSemaphoreAcquire(CommunicationSemaphore, osWaitForever);
	uint8_t index = 0;
	for (uint8_t address : group_addresses) {
		if (address == address_) {
			group_addresses.erase(group_addresses.begin() + index);
			return true;
		}
		index++;
	}
	osSemaphoreRelease(CommunicationSemaphore);
	return false;
}

/**
 * @brief Clear the list of group addresses
 *
 * @return whether successful
 */
bool Communication::ClearGroupAddress(void) {
	osSemaphoreAcquire(CommunicationSemaphore, osWaitForever);
	group_addresses.clear();
	osSemaphoreRelease(CommunicationSemaphore);
	return true;
}
