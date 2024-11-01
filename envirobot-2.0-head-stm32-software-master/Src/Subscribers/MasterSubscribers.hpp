/*
 * MasterSubscribers.hpp
 *
 *  Created on: 7 déc. 2022
 *      Author: bignet
 */

#pragma once

#include <vector>
#include <stm32h750xx.h>
#include <stm32h7xx_hal.h>
#include "cmsis_os.h"

#include "Subscribers/Core/Subscriber.hpp"
#include "Registers/Registers.hpp"
#include "LEDS/LEDS.hpp"
#include "Protocols/Communication/Core/CommunicationProtocol.hpp"

#include "Definitions/CommunicationDefinition.h"
#include "Configurations/RegistersConfiguration.h"
#include "Configurations/CommunicationConfiguration.h"
#include "RegisterMaps/RegisterMapSubscribers.h"

#include "PlatformLEDs/PlatformLEDs.hpp"

// SubscriberConfiguration struct
struct SubscriberConfiguration {
	uint8_t ID;										// subscriber ID
	bool activated;									// subscriber whether active or not
	std::vector<Register> remote_registers;			// subscribed remote register configurations
	std::vector<SubscriberInterface> interfaces;    // interfaces associated to remote_registers (index by index)
	std::vector<uint8_t> filters;					// list of module address filters
};

// MasterSubscribers class used to receive published data from other modules
class MasterSubscribers {
public:
	MasterSubscribers();
	void Init(Registers* registers, LEDS* leds);
	void AddRegisters(void);

	bool AddSubscriber(uint8_t ID, Subscriber* subscriber);
	bool RemoveSubscriber(uint8_t ID);
	bool SetSubscriberStatus(uint8_t ID, bool status);
	bool ActivateSubscriber(uint8_t ID);
	bool DeactivateSubscriber(uint8_t ID);

	bool SubscribeToRemoteRegister(uint8_t ID, uint8_t register_ID);
	bool SubscribeToRemoteRegister(uint8_t ID, Register remote_register);
	bool SubscribeToRemoteRegister(uint8_t ID, uint8_t register_ID, SubscriberInterface interface);
	bool SubscribeToRemoteRegister(uint8_t ID, Register remote_register, SubscriberInterface interface);
	bool UnsubscribeFromRemoteRegister(uint8_t ID, uint8_t remote_register_ID);
	bool UnsubscribeFromAllRemoteRegisters(uint8_t ID);

	bool AddFilter(uint8_t ID, uint8_t address);
	bool RemoveFilter(uint8_t ID, uint8_t address);
	bool ClearFilters(uint8_t ID);

	template<typename T>
	void Receive(SubscriberInput information, T* data);

	bool AddressAccepted(uint8_t ID, uint8_t address);
	Register FindRemoteRegister(Register remote_register, SubscriberInterface interface, bool* success);

private:
	Registers* registers;
	LEDS* leds;

	// OS
	osSemaphoreId_t SubscribersSemaphore;

	bool RemoteRegisterSubscriptionAvailable(Register remote_register, SubscriberInterface interface);
	uint8_t FindSubscriberIndex(uint8_t ID, bool* success);
	bool SubscribedToRemoteRegister(uint8_t ID, Register remote_register, SubscriberInterface interface);
	uint8_t FindRemoteRegisterIndex(uint8_t ID, Register remote_register, SubscriberInterface interface, bool* success);

	std::vector<SubscriberConfiguration> configurations;
	std::vector<Subscriber*> subscribers;
};

/**
 * @brief Function called when the data from a publisher is received
 *
 * @param input information: input register and interface information
 * @param input data: published data for the input register
 */
template<typename T>
void MasterSubscribers::Receive(SubscriberInput information, T* data) {
	uint8_t index = 0;
	for (SubscriberConfiguration configuration : configurations) {
		// Check that the subscriber is active
		if (configuration.activated) {
			// Check that the sucriber is subscribed to the input published register
			if (SubscribedToRemoteRegister(configuration.ID, information.register_, information.interface)) {
				// Check that the address passes the filters
				if (AddressAccepted(configuration.ID, information.interface.address)) {

#ifdef USE_COM_LED_SUBSCRIBER_INPUT
					CommunicationLED(leds, information.interface.interface);
#endif

					// Forward the data to the subscriber
					switch (information.register_.type) {
#if defined(USE_UINT8_COMMUNICATION) && defined(USE_UINT8_REGISTER)
					case UINT8_TYPE:
						subscribers[index]->ReceiveUINT8(information, (uint8_t*) data);
						break;
#endif
#if defined(USE_UINT16_COMMUNICATION) && defined(USE_UINT16_REGISTER)
					case UINT16_TYPE:
						subscribers[index]->ReceiveUINT16(information, (uint16_t*) data);
						break;
#endif
#if defined(USE_UINT32_COMMUNICATION) && defined(USE_UINT32_REGISTER)
					case UINT32_TYPE:
						subscribers[index]->ReceiveUINT32(information, (uint32_t*) data);
						break;
#endif
#if defined(USE_UINT64_COMMUNICATION) && defined(USE_UINT64_REGISTER)
					case UINT64_TYPE:
						subscribers[index]->ReceiveUINT64(information, (uint64_t*) data);
						break;
#endif
#if defined(USE_INT8_COMMUNICATION) && defined(USE_INT8_REGISTER)
					case INT8_TYPE:
						subscribers[index]->ReceiveINT8(information, (int8_t*) data);
						break;
#endif
#if defined(USE_INT16_COMMUNICATION) && defined(USE_INT16_REGISTER)
					case INT16_TYPE:
						subscribers[index]->ReceiveINT16(information, (int16_t*) data);
						break;
#endif
#if defined(USE_INT32_COMMUNICATION) && defined(USE_INT32_REGISTER)
					case INT32_TYPE:
						subscribers[index]->ReceiveINT32(information, (int32_t*) data);
						break;
#endif
#if defined(USE_INT64_COMMUNICATION) && defined(USE_INT64_REGISTER)
					case INT64_TYPE:
						subscribers[index]->ReceiveINT64(information, (int64_t*) data);
						break;
#endif
#if defined(USE_FLOAT_COMMUNICATION) && defined(USE_FLOAT_REGISTER)
					case FLOAT_TYPE:
						subscribers[index]->ReceiveFLOAT(information, (float*) data);
						break;
#endif
#if defined(USE_DOUBLE_COMMUNICATION) && defined(USE_DOUBLE_REGISTER)
					case DOUBLE_TYPE:
						subscribers[index]->ReceiveDOUBLE(information, (double*) data);
						break;
#endif
					default:
						break;
					}
				}
			}
		}
		index++;
	}
}
