/*
 * Publishers.cpp
 *
 *  Created on: 7 déc. 2022
 *      Author: bignet
 */

#include <Publishers/Publishers.hpp>

/**
 * @brief Class constructor
 */
Publishers::Publishers() {
	publishers.clear();
}

/**
 * @brief Initialize the class with references to other classes. Set default values.
 *
 * @param input registers_: the Registers instance
 * @param input communication_: the Communication instance
 */
void Publishers::Init(Registers* registers_, Communication* communication_) {
	registers = registers_;
	communication = communication_;

	PublishersSemaphore = osSemaphoreNew(1,1,NULL);
	osSemaphoreRelease(PublishersSemaphore);
}

/**
 * @brief Add class related registers
 */
void Publishers::AddRegisters(void) {
	// Register to set a publisher status
	registers->AddRegister<uint8_t>(REG_PUB_SET_STATUS);
	registers->SetRegisterAsArray(REG_PUB_SET_STATUS, 2);
	registers->AddWriteCallback<uint8_t>(REG_PUB_SET_STATUS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Publishers* self = (Publishers*) context;
			if (length != 2) return false;
			self->SetPublisherStatus(input[0], (bool) input[1]);
			return true;
		}
	);

	// Register to set a topic status
	registers->AddRegister<uint16_t>(REG_PUB_SET_TOPIC_STATUS);
	registers->SetRegisterAsArray(REG_PUB_SET_TOPIC_STATUS, 3);
	registers->AddWriteCallback<uint16_t>(REG_PUB_SET_TOPIC_STATUS, (void*) this,
		[](void* context, uint16_t register_id, uint16_t* input, uint16_t length) -> bool {
			Publishers* self = (Publishers*) context;
			if (length != 3) return false;
			self->SetTopicStatus((uint8_t) input[0], input[1], (bool) input[2]);
			return true;
		}
	);

	// Register to set the publisher prescaler
	registers->AddRegister<uint16_t>(REG_PUB_SET_PRESCALER);
	registers->SetRegisterAsArray(REG_PUB_SET_PRESCALER, 2);
	registers->AddWriteCallback<uint16_t>(REG_PUB_SET_PRESCALER, (void*) this,
		[](void* context, uint16_t register_id, uint16_t* input, uint16_t length) -> bool {
			Publishers* self = (Publishers*) context;
			if (length != 2) return false;
			self->SetPublisherPrescaler((uint8_t) input[0], input[1]);
			return true;
		}
	);

	// Register to stop all topics of a given publisher
	registers->AddRegister<uint8_t>(REG_PUB_STOP_TOPICS);
	registers->SetRegisterAsSingle(REG_PUB_STOP_TOPICS);
	registers->AddWriteCallback<uint8_t>(REG_PUB_STOP_TOPICS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Publishers* self = (Publishers*) context;
			self->StopTopics(*input);
			return true;
		}
	);

	// Register to link an interface to a publisher
	registers->AddRegister<uint8_t>(REG_PUB_LINK_INTERFACE);
	registers->SetRegisterAsArray(REG_PUB_LINK_INTERFACE, 2);
	registers->AddWriteCallback<uint8_t>(REG_PUB_LINK_INTERFACE, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Publishers* self = (Publishers*) context;
			if (length != 2) return false;
			self->LinkToInterface(input[0], input[1]);
			return true;
		}
	);

	// Register to unlink an interface from a publisher
	registers->AddRegister<uint8_t>(REG_PUB_UNLINK_INTERFACE);
	registers->SetRegisterAsArray(REG_PUB_UNLINK_INTERFACE, 2);
	registers->AddWriteCallback<uint8_t>(REG_PUB_UNLINK_INTERFACE, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Publishers* self = (Publishers*) context;
			if (length != 2) return false;
			self->UnlinkFromInterface(input[0], input[1]);
			return true;
		}
	);

	// Register to clear all interfaces from a publisher
	registers->AddRegister<uint8_t>(REG_PUB_CLEAR_INTERFACES);
	registers->SetRegisterAsSingle(REG_PUB_CLEAR_INTERFACES);
	registers->AddWriteCallback<uint8_t>(REG_PUB_CLEAR_INTERFACES, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Publishers* self = (Publishers*) context;
			self->UnlinkFromAllInterfaces(*input);
			return true;
		}
	);

	// Register to set the target address of a publisher for a specific communication interface
	registers->AddRegister<uint8_t>(REG_PUB_SET_TARGET_ADDRESS);
	registers->SetRegisterAsArray(REG_PUB_SET_TARGET_ADDRESS, 3);
	registers->AddWriteCallback<uint8_t>(REG_PUB_SET_TARGET_ADDRESS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Publishers* self = (Publishers*) context;
			if (length != 3) return false;
			self->SetPublishAddress(input[0], input[1], input[2]);
			return true;
		}
	);
}

/**
 * @brief Add a publisher
 *
 * @param input ID: the publisher ID to add
 * @return whether successful
 */
bool Publishers::AddPublisher(uint8_t ID) {
	bool success;

	// Check the publisher is not already registered
	FindPublisher(ID, &success);
	if (success)
		return false;

	// Setup publisher
	Publisher publisher;
	publisher.activated = false;
	publisher.ID = ID;
	publisher.prescaler = 1;
	publisher.counter = 0;

	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	// Add publisher
	publishers.push_back(publisher);
	osSemaphoreRelease(PublishersSemaphore);
	return true;
}

/**
 * @brief Remove a publisher
 *
 * @param input ID: the publisher ID
 * @return whether successful
 */
bool Publishers::RemovePublisher(uint8_t ID) {
	bool success;
	uint8_t index = FindPublisherIndex(ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	publishers.erase(publishers.begin() + index);
	osSemaphoreRelease(PublishersSemaphore);
	return true;
}

/**
 * @brief Set the status of publisher
 *
 * @param input ID: the publisher ID
 * @param input status: whether the publisher is active or not
 * @return whether successful
 */
bool Publishers::SetPublisherStatus(uint8_t ID, bool status) {
	bool success;
	Publisher* publisher = FindPublisher(ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	publisher->counter = 0;
	publisher->activated = status;
	osSemaphoreRelease(PublishersSemaphore);
	return true;
}

/**
 * @brief Activate a publisher
 *
 * @param input ID: the publisher ID
 * @return whether successful
 */
bool Publishers::ActivatePublisher(uint8_t ID) {
	return SetPublisherStatus(ID, true);
}

/**
 * @brief Deactivate a publisher
 *
 * @param input ID: the publisher ID
 * @return whether successful
 */
bool Publishers::DeactivatePublisher(uint8_t ID) {
	return SetPublisherStatus(ID, false);
}

/**
 * @brief Set the publisher prescaler
 *
 * @param input ID: the publisher ID
 * @param input prescaler: the publisher prescaler with respect to the Spin period
 * @return whether successful
 */
bool Publishers::SetPublisherPrescaler(uint8_t ID, uint16_t prescaler) {
	bool success;
	Publisher* publisher = FindPublisher(ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	publisher->prescaler = prescaler;
	osSemaphoreRelease(PublishersSemaphore);
	return true;
}

/**
 * @brief Add a topic to a publisher
 *
 * @param input ID: the publisher ID
 * @param input topic_ID: register address to link to the publisher. The register has to be readable
 * @return whether successful
 */
bool Publishers::AddTopic(uint8_t ID, uint16_t topic_ID) {
	bool success;
	Publisher* publisher = FindPublisher(ID, &success);
	if (!success) return false;

	FindTopic(ID, topic_ID, &success);
	if (success) return false;

	Topic topic;
	topic.activated = false;
	topic.ID = topic_ID;

	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	publisher->topics.push_back(topic);
	osSemaphoreRelease(PublishersSemaphore);
	return true;
}

/**
 * @brief Remove a topic to a publisher
 *
 * @param input ID: the publisher ID
 * @param input topic_ID: register address to remove from the publisher
 * @return whether successful
 */
bool Publishers::RemoveTopic(uint8_t ID, uint16_t topic_ID) {
	bool success;
	Publisher* publisher = FindPublisher(ID, &success);
	if (!success) return false;

	uint16_t index = FindTopicIndex(ID, topic_ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	publisher->topics.erase(publisher->topics.begin() + index);
	osSemaphoreRelease(PublishersSemaphore);
	return true;
}

/**
 * @brief Set a topic status
 *
 * @param input ID: the publisher ID
 * @param input topic_ID: register address
 * @param status: whether the topic is active or not
 * @return whether successful
 */
bool Publishers::SetTopicStatus(uint8_t ID, uint16_t topic_ID, bool status) {
	bool success;
	Topic* topic = FindTopic(ID, topic_ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	topic->activated = status;
	osSemaphoreRelease(PublishersSemaphore);
	return true;
}

/**
 * @brief Activate a topic
 *
 * @param input ID: the publisher ID
 * @param input topic_ID: register address to link to the publisher. The register has to be readable
 * @return whether successful
 */
bool Publishers::ActivateTopic(uint8_t ID, uint16_t topic_ID) {
	return SetTopicStatus(ID, topic_ID, true);
}

/**
 * @brief Deactivate a topic
 *
 * @param input ID: the publisher ID
 * @param input topic_ID: register address to link to the publisher. The register has to be readable
 * @return whether successful
 */
bool Publishers::DeactivateTopic(uint8_t ID, uint16_t topic_ID) {
	return SetTopicStatus(ID, topic_ID, false);
}

/**
 * @brief Stop all topics for a given publisher
 *
 * @param input ID: the publisher ID
 * @return whether successful
 */
bool Publishers::StopTopics(uint8_t ID) {
	bool success;
	Publisher* publisher = FindPublisher(ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	for (Topic &topic : publisher->topics) {
		topic.activated = false;
	}
	osSemaphoreRelease(PublishersSemaphore);
	return true;
}

/**
 * @brief Link a communication interface to a publisher
 *
 * @param input ID: the publisher ID
 * @param input interface_ID: the communication ID from Communications to send data
 * @return whether successful
 */
bool Publishers::LinkToInterface(uint8_t ID, uint8_t interface_ID) {
	bool success;
	Publisher* publisher = FindPublisher(ID, &success);
	if (!success) return false;

	FindPublishInterface(ID, interface_ID, &success);
	if (success) return false;

	PublishInterface interface;
	interface.interface = interface_ID;
	interface.address = ALL;

	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	publisher->interfaces.push_back(interface);
	osSemaphoreRelease(PublishersSemaphore);
	return true;
}

/**
 * @brief Unlink a communication interface from a publisher
 *
 * @param input ID: the publisher ID
 * @param input interface_ID: the interface ID to remove
 * @return whether successful
 */
bool Publishers::UnlinkFromInterface(uint8_t ID, uint8_t interface_ID) {
	bool success;
	Publisher* publisher = FindPublisher(ID, &success);
	if (!success) return false;

	uint8_t index = FindPublishInterfaceIndex(ID, interface_ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	publisher->interfaces.erase(publisher->interfaces.begin() + index);
	osSemaphoreRelease(PublishersSemaphore);
	return true;
}

/**
 * @brief Clear all linked communication interfaces from a publisher
 *
 * @param input ID: the publisher ID
 * @return whether successful
 */
bool Publishers::UnlinkFromAllInterfaces(uint8_t ID) {
	bool success;
	Publisher* publisher = FindPublisher(ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	publisher->interfaces.clear();
	osSemaphoreRelease(PublishersSemaphore);
	return true;
}

/**
 * @brief Set the target address for a given communication interface linked to a publisher
 *
 * @param input ID: the publisher ID
 * @param input interface_ID: the interface ID
 * @param input address: the address to publish to on the selected interface
 * @return whether successful
 */
bool Publishers::SetPublishAddress(uint8_t ID, uint8_t interface_ID, uint8_t address) {
	bool success;
	PublishInterface* interface = FindPublishInterface(ID, interface_ID, &success);
	if (success) return false;

	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	interface->address = address;
	osSemaphoreRelease(PublishersSemaphore);
	return true;
}

/**
 * @brief Process a publisher step. If the publisher is active and that there is a prescaler counter overflow, then send the activated topics via the linked interfaces
 *
 * @param input ID: the publisher ID to spin
 */
void Publishers::SpinPublisher(uint8_t ID) {
	bool success;
	// find publisher
	Publisher* publisher = FindPublisher(ID, &success);
	if (!success) return;

	// check publisher is active
	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	if (not publisher->activated) {
		osSemaphoreRelease(PublishersSemaphore);
		return;
	}

	// publisher prescaler
	publisher->counter += 1;
	if (publisher->counter >= publisher->prescaler) {
		publisher->counter = 0;

		// group data and send on interfaces
		for (PublishInterface interface : publisher->interfaces) {
			MessageHeader message;
			message.interface = interface.interface;
			message.target_address = interface.address;

			// Read topics and build a message
			for (Topic topic : publisher->topics) {
				if (topic.activated) {
					MessageElement element;
					element.ack = false;
					element.command = false;
					element.access = false;

					Register element_register = registers->FindRegister(topic.ID, &success);
					if (not success)
						continue;

					element.element_register = element_register;

					std::vector<uint8_t> register_read;
					switch (element.element_register.type) {
#if defined(USE_UINT8_COMMUNICATION) && defined(USE_UINT8_REGISTER)
					case UINT8_TYPE:
						register_read = communication->ReadElement<uint8_t>(&element, &success);
						break;
#endif
#if defined(USE_UINT16_COMMUNICATION) && defined(USE_UINT16_REGISTER)
					case UINT16_TYPE:
						register_read = communication->ReadElement<uint16_t>(&element, &success);
						break;
#endif
#if defined(USE_UINT32_COMMUNICATION) && defined(USE_UINT32_REGISTER)
					case UINT32_TYPE:
						register_read = communication->ReadElement<uint32_t>(&element, &success);
						break;
#endif
#if defined(USE_UINT64_COMMUNICATION) && defined(USE_UINT64_REGISTER)
					case UINT64_TYPE:
						register_read = communication->ReadElement<uint64_t>(&element, &success);
						break;
#endif
#if defined(USE_INT8_COMMUNICATION) && defined(USE_INT8_REGISTER)
					case INT8_TYPE:
						register_read = communication->ReadElement<int8_t>(&element, &success);
						break;
#endif
#if defined(USE_INT16_COMMUNICATION) && defined(USE_INT16_REGISTER)
					case INT16_TYPE:
						register_read = communication->ReadElement<int16_t>(&element, &success);
						break;
#endif
#if defined(USE_INT32_COMMUNICATION) && defined(USE_INT32_REGISTER)
					case INT32_TYPE:
						register_read = communication->ReadElement<int32_t>(&element, &success);
						break;
#endif
#if defined(USE_INT64_COMMUNICATION) && defined(USE_INT64_REGISTER)
					case INT64_TYPE:
						register_read = communication->ReadElement<int64_t>(&element, &success);
						break;
#endif
#if defined(USE_FLOAT_COMMUNICATION) && defined(USE_FLOAT_REGISTER)
					case FLOAT_TYPE:
						register_read = communication->ReadElement<float>(&element, &success);
						break;
#endif
#if defined(USE_DOUBLE_COMMUNICATION) && defined(USE_DOUBLE_REGISTER)
					case DOUBLE_TYPE:
						register_read = communication->ReadElement<double>(&element, &success);
						break;
#endif
					default:
						success = false;
						break;
					}

					if (not success)
						continue;

					element.data = register_read;

					// if no space available, send the data and then rebuild a new response
					if (not communication->BuildPayload(&message, element) && message.payload.size() > 0) {
						communication->Send(message);
						message.payload.clear();
						communication->BuildPayload(&message, element);
					}
				}
			}

			// Send topic values if any
			if (message.payload.size() > 0) {
				communication->Send(message);
			}
		}
	}
	osSemaphoreRelease(PublishersSemaphore);
}

/**
 * @brief Find a publisher from an ID
 *
 * @param input ID: the publisher ID
 * @param output success: whether successful
 * @return the publisher pointer
 */
Publisher* Publishers::FindPublisher(uint8_t ID, bool* success) {
	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	for (Publisher &publisher : publishers) {
		if (publisher.ID == ID) {
			Publisher* publisher_pointer = &publisher;
			osSemaphoreRelease(PublishersSemaphore);
			*success = true;
			return publisher_pointer;
		}
	}
	osSemaphoreRelease(PublishersSemaphore);
	*success = false;
	return NULL;
}

/**
 * @brief Find a publisher index from an ID
 *
 * @param input ID: the publisher ID
 * @param output success: whether successful
 * @return the publisher index
 */
uint8_t Publishers::FindPublisherIndex(uint8_t ID, bool* success) {
	uint8_t index = 0;
	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	for (Publisher &publisher : publishers) {
		if (publisher.ID == ID) {
			osSemaphoreRelease(PublishersSemaphore);
			*success = true;
			return index;
		}
		index++;
	}
	osSemaphoreRelease(PublishersSemaphore);
	*success = false;
	return 0;
}

/**
 * @brief Find a topic of a publisher from a register address
 *
 * @param input ID: the publisher ID
 * @param input topic_ID: the topic ID / register address to find
 * @param output success: whether successful
 * @return the topic pointer
 */
Topic* Publishers::FindTopic(uint8_t ID, uint16_t topic_ID, bool* success) {
	Publisher* publisher = FindPublisher(ID, success);
	if (not *success)
		return NULL;

	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	for (Topic &topic : publisher->topics) {
		if (topic.ID == topic_ID) {
			Topic* topic_pointer = &topic;
			osSemaphoreRelease(PublishersSemaphore);
			*success = true;
			return topic_pointer;
		}
	}
	osSemaphoreRelease(PublishersSemaphore);
	*success = false;
	return NULL;
}

/**
 * @brief Find the topic index of a publisher from a register address
 *
 * @param input ID: the publisher ID
 * @param input topic_ID: the topic ID / register address to find
 * @param output success: whether successful
 * @return the topic index
 */
uint16_t Publishers::FindTopicIndex(uint8_t ID, uint16_t topic_ID, bool* success) {
	Publisher* publisher = FindPublisher(ID, success);
	if (not *success)
		return 0;

	uint16_t index = 0;
	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	for (Topic &topic : publisher->topics) {
		if (topic.ID == topic_ID) {
			osSemaphoreRelease(PublishersSemaphore);
			*success = true;
			return index;
		}
		index++;
	}
	osSemaphoreRelease(PublishersSemaphore);
	*success = false;
	return 0;
}

/**
 * @brief Find a linked interface of a publisher from an interface ID
 *
 * @param input ID: the publisher ID
 * @param input interface_ID: the interface ID to find
 * @param output success: whether successful
 * @return the interface information pointer
 */
PublishInterface* Publishers::FindPublishInterface(uint8_t ID, uint8_t interface_ID, bool* success) {
	Publisher* publisher = FindPublisher(ID, success);
	if (not *success)
		return NULL;

	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	for (PublishInterface &interface : publisher->interfaces) {
		if (interface.interface == interface_ID) {
			PublishInterface* publish_interface_pointer = &interface;
			osSemaphoreRelease(PublishersSemaphore);
			*success = true;
			return publish_interface_pointer;
		}
	}
	osSemaphoreRelease(PublishersSemaphore);
	*success = false;
	return NULL;
}

/**
 * @brief Find the linked interface index of a publisher from an interface ID
 *
 * @param input ID: the publisher ID
 * @param input interface_ID: the interface ID to find
 * @param output success: whether successful
 * @return the interface information index
 */
uint8_t Publishers::FindPublishInterfaceIndex(uint8_t ID, uint8_t interface_ID, bool* success) {
	Publisher* publisher = FindPublisher(ID, success);
	if (not *success)
		return 0;

	uint8_t index = 0;
	osSemaphoreAcquire(PublishersSemaphore, osWaitForever);
	for (PublishInterface &interface : publisher->interfaces) {
		if (interface.interface == interface_ID) {
			osSemaphoreRelease(PublishersSemaphore);
			*success = true;
			return index;
		}
		index++;
	}
	osSemaphoreRelease(PublishersSemaphore);
	*success = false;
	return 0;
}
