/*
 * Publishers.hpp
 *
 *  Created on: 7 déc. 2022
 *      Author: bignet
 */

#pragma once

#include <vector>
#include <stm32h750xx.h>
#include <stm32h7xx_hal.h>
#include "cmsis_os.h"

#include "Registers/Registers.hpp"
#include "Protocols/Communication/Core/CommunicationProtocol.hpp"
#include "Communication/Communication.hpp"

#include "Configurations/PublishersConfiguration.h"
#include "Configurations/RegistersConfiguration.h"
#include "Configurations/CommunicationConfiguration.h"
#include "RegisterMaps/RegisterMapPublishers.h"

// PublishInterface struct
struct PublishInterface {
	uint8_t interface;		// interface ID
	uint8_t address;		// target address to publish to for the associated interface
};

// Topic struct
struct Topic {
	bool activated;			// whether active or not
	uint16_t ID; 			// register address
};

// Publisher struct
struct Publisher {
	uint8_t ID;									// publisher ID
	bool activated;								// whether active or not
	uint16_t prescaler; 						// publisher prescaler
	uint16_t counter;							// publisher prescaler counter
	std::vector<PublishInterface> interfaces; 	// links interfaces to the publisher
	std::vector<Topic> topics;					// topics of the publisher
};

// Publishers class used to broadcast registers on the bus
class Publishers {
public:
	Publishers();
	void Init(Registers* registers_, Communication* communication_);
	void AddRegisters(void);

	bool AddPublisher(uint8_t ID);
	bool RemovePublisher(uint8_t ID);
	bool SetPublisherStatus(uint8_t ID, bool status);
	bool ActivatePublisher(uint8_t ID);
	bool DeactivatePublisher(uint8_t ID);
	bool SetPublisherPrescaler(uint8_t ID, uint16_t prescaler);

	bool AddTopic(uint8_t ID, uint16_t topic_ID);
	bool RemoveTopic(uint8_t ID, uint16_t topic_ID);
	bool SetTopicStatus(uint8_t ID, uint16_t topic_ID, bool status);
	bool ActivateTopic(uint8_t ID, uint16_t topic_ID);
	bool DeactivateTopic(uint8_t ID, uint16_t topic_ID);
	bool StopTopics(uint8_t ID);

	void SpinPublisher(uint8_t ID);

	bool LinkToInterface(uint8_t ID, uint8_t interface_ID);
	bool UnlinkFromInterface(uint8_t ID, uint8_t interface_ID);
	bool UnlinkFromAllInterfaces(uint8_t ID);
	bool SetPublishAddress(uint8_t ID, uint8_t interface_ID, uint8_t address);

private:
	// OS
	osSemaphoreId_t PublishersSemaphore;

	Publisher* FindPublisher(uint8_t ID, bool* success);
	uint8_t FindPublisherIndex(uint8_t ID, bool* success);
	Topic* FindTopic(uint8_t ID, uint16_t topic_ID, bool* success);
	uint16_t FindTopicIndex(uint8_t ID, uint16_t topic_ID, bool* success);
	PublishInterface* FindPublishInterface(uint8_t ID, uint8_t interface_ID, bool* success);
	uint8_t FindPublishInterfaceIndex(uint8_t ID, uint8_t interface_ID, bool* success);

	Registers* registers;
	Communication* communication;

	std::vector<Publisher> publishers;
};
