/*
 * Subscribers.cpp
 *
 *  Created on: 7 déc. 2022
 *      Author: bignet
 */

#include <Subscribers/MasterSubscribers.hpp>

/**
 * @brief Class constructor
 */
MasterSubscribers::MasterSubscribers() {
}

/**
 * @brief Initialize the class with references to other classes. Set default values.
 *
 * @param input registers_: the Registers instance
 * @param input leds_: the LEDS instance
 */
void MasterSubscribers::Init(Registers* registers_, LEDS* leds_) {
	registers = registers_;
	leds = leds_;

	SubscribersSemaphore = osSemaphoreNew(1,1,NULL);
	osSemaphoreRelease(SubscribersSemaphore);
}

/**
 * @brief Add class related registers
 */
void MasterSubscribers::AddRegisters(void) {
	// Register to set the subscriber status
	registers->AddRegister<uint8_t>(REG_SUB_SET_STATUS);
	registers->SetRegisterAsArray(REG_SUB_SET_STATUS, 2);
	registers->AddWriteCallback<uint8_t>(REG_SUB_SET_STATUS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			MasterSubscribers* self = (MasterSubscribers*) context;
			if (length != 2) return false;
			self->SetSubscriberStatus(input[0], (bool) input[1]);
			return true;
		}
	);

	// Register to add an address filter to a subscriber
	registers->AddRegister<uint8_t>(REG_SUB_ADD_FILTER);
	registers->SetRegisterAsArray(REG_SUB_ADD_FILTER, 2);
	registers->AddWriteCallback<uint8_t>(REG_SUB_ADD_FILTER, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			MasterSubscribers* self = (MasterSubscribers*) context;
			if (length != 2) return false;
			self->AddFilter(input[0], input[1]);
			return true;
		}
	);

	// Register to clear all filters from a subscriber
	registers->AddRegister<uint8_t>(REG_SUB_CLEAR_FILTERS);
	registers->SetRegisterAsSingle(REG_SUB_CLEAR_FILTERS);
	registers->AddWriteCallback<uint8_t>(REG_SUB_CLEAR_FILTERS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			MasterSubscribers* self = (MasterSubscribers*) context;
			self->ClearFilters(*input);
			return true;
		}
	);
}

/**
 * @brief Add a subscriber
 *
 * @param input ID: subscriber ID
 * @param input subscriber: pointer to the custom subscriber to add
 * @return whether successful
 */
bool MasterSubscribers::AddSubscriber(uint8_t ID, Subscriber* subscriber) {
	bool success;

	// Check there is no subscriber already registered with the same ID
	FindSubscriberIndex(ID, &success);
	if (success)
		return false;

	// Setup subscriber
	SubscriberConfiguration configuration;
	configuration.ID = ID;
	configuration.activated = false;

	// Add subscriber
	osSemaphoreAcquire(SubscribersSemaphore, osWaitForever);
	configurations.push_back(configuration);
	subscribers.push_back(subscriber);
	osSemaphoreRelease(SubscribersSemaphore);
	return true;
}

/**
 * @brief Remove a subscriber
 *
 * @param input ID: subscriber ID
 * @return whether successful
 */
bool MasterSubscribers::RemoveSubscriber(uint8_t ID) {
	bool success;
	uint8_t index = FindSubscriberIndex(ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(SubscribersSemaphore, osWaitForever);
	configurations.erase(configurations.begin() + index);
	subscribers.erase(subscribers.begin() + index);
	osSemaphoreRelease(SubscribersSemaphore);
	return true;
}

/**
 * @brief Set the subscriber status
 *
 * @param input ID: subscriber ID
 * @param input status: whether active or not
 * @return whether successful
 */
bool MasterSubscribers::SetSubscriberStatus(uint8_t ID, bool status) {
	bool success;
	uint8_t index = FindSubscriberIndex(ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(SubscribersSemaphore, osWaitForever);
	configurations[index].activated = status;
	osSemaphoreRelease(SubscribersSemaphore);
	return true;
}

/**
 * @brief Activate a subscriber
 *
 * @param input ID: subscriber ID
 * @return whether successful
 */
bool MasterSubscribers::ActivateSubscriber(uint8_t ID) {
	return SetSubscriberStatus(ID, true);
}

/**
 * @brief Deactivate a subscriber
 *
 * @param input ID: subscriber ID
 * @return whether successful
 */
bool MasterSubscribers::DeactivateSubscriber(uint8_t ID) {
	return SetSubscriberStatus(ID, false);
}

/**
 * @brief Subscribe a subscriber to a remote register that is internally registered. Accept data from all interfaces
 *		  Accepts only once subscriber register configuration for a given ID. Return false if another configuration is found with the same ID than remote_register.
 *		  This enables to make sure the Communication module has the correct configuration to parseMessage.
 *
 * @param input ID: subscriber ID
 * @param input register_ID: register address from Registers that is internally registered to retrieve register information (type, isArray, length)
 * @return whether successful
 */
bool MasterSubscribers::SubscribeToRemoteRegister(uint8_t ID, uint8_t register_ID) {
	SubscriberInterface interface;
	interface.interface = ALL_INTERFACES;
	interface.address = ALL;

	return SubscribeToRemoteRegister(ID, register_ID, interface);
}

/**
 * @brief Subscribe a subscriber to a remote register that is NOT internally registered. Accept data from all interfaces
 *		  Accepts only once subscriber register configuration for a given ID. Return false if another configuration is found with the same ID than remote_register.
 *		  This enables to make sure the Communication module has the correct configuration to parseMessage.
 *
 * @param input ID: subscriber ID
 * @param input remote_register: full register configuration to subscribe to
 * @return whether successful
 */
bool MasterSubscribers::SubscribeToRemoteRegister(uint8_t ID, Register remote_register) {
	SubscriberInterface interface;
	interface.interface = ALL_INTERFACES;
	interface.address = ALL;

	return SubscribeToRemoteRegister(ID, remote_register, interface);
}

/**
 * @brief Subscribe a subscriber to a remote register that is internally registered. Accept data from a specific interface only
 *		  Accepts only once subscriber register configuration for a given ID. Return false if another configuration is found with the same ID than the corresponding register_ID.
 *		  This enables to make sure the Communication module has the correct configuration to parseMessage.
 *
 * @param input ID: subscriber ID
 * @param input register_ID: register address from Registers that is internally registered to retrieve register information (type, isArray, length)
 * @param input interface: interface information to filter incomming data
 * @return whether successful
 */
bool MasterSubscribers::SubscribeToRemoteRegister(uint8_t ID, uint8_t register_ID, SubscriberInterface interface) {
	bool success;
	Register internal_register = registers->FindRegister(register_ID, &success);
	if (!success) return false;

	return SubscribeToRemoteRegister(ID, internal_register, interface);
}

/**
 * @brief Subscribe a subscriber to a remote register that is NOT internally registered. Accept data from all interfaces
 *		  Accepts only once subscriber register configuration for a given ID. Return false if another configuration is found with the same ID than remote_register.
 *		  This enables to make sure the Communication module has the correct configuration to parseMessage.
 *
 * @param input ID: subscriber ID
 * @param input remote_register: full register configuration to subscribe to
 * @param input interface: interface information to filter incomming data
 * @return whether successful
 */
bool MasterSubscribers::SubscribeToRemoteRegister(uint8_t ID, Register remote_register, SubscriberInterface interface) {
	bool success;
	uint8_t index = FindSubscriberIndex(ID, &success);
	if (!success) return false;

	// Check register not already registered for the given subscriber
	if (SubscribedToRemoteRegister(ID, remote_register, interface))
		return false;

	if (!RemoteRegisterSubscriptionAvailable(remote_register, interface)) return false;

	osSemaphoreAcquire(SubscribersSemaphore, osWaitForever);
	configurations[index].remote_registers.push_back(remote_register);
	configurations[index].interfaces.push_back(interface);
	osSemaphoreRelease(SubscribersSemaphore);
	return true;
}

/**
 * @brief Unsubscribe a subscriber from a remote register
 *
 * @param input ID: subscriber ID
 * @param input remote_register_ID: subscribed register ID
 * @return whether successful
 */
bool MasterSubscribers::UnsubscribeFromRemoteRegister(uint8_t ID, uint8_t remote_register_ID) {
	bool success;
	uint8_t index = FindSubscriberIndex(ID, &success);
	if (!success) return false;

	uint8_t register_index = FindRemoteRegisterIndex(ID, Register{.address=remote_register_ID}, SubscriberInterface{.interface=ALL_INTERFACES, .address=ALL}, &success);
	if (!success) return false;

	osSemaphoreAcquire(SubscribersSemaphore, osWaitForever);
	configurations[index].remote_registers.erase(configurations[index].remote_registers.begin() + register_index);
	configurations[index].interfaces.erase(configurations[index].interfaces.begin() + register_index);
	osSemaphoreRelease(SubscribersSemaphore);
	return true;
}

/**
 * @brief Unsubscribe a subscriber from all remote registers
 *
 * @param input ID: subscriber ID
 * @return whether successful
 */
bool MasterSubscribers::UnsubscribeFromAllRemoteRegisters(uint8_t ID) {
	bool success;
	uint8_t index = FindSubscriberIndex(ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(SubscribersSemaphore, osWaitForever);
	configurations[index].remote_registers.clear();
	osSemaphoreRelease(SubscribersSemaphore);
	return true;
}

/**
 * @brief Add an address filter to a subscriber
 *
 * @param input ID: subscriber ID
 * @param input address: module address to filter
 * @return whether successful
 */
bool MasterSubscribers::AddFilter(uint8_t ID, uint8_t address) {
	bool success;
	uint8_t index = FindSubscriberIndex(ID, &success);
	if (!success) return false;

	if (configurations[index].filters.size() > 0 && AddressAccepted(index, address)) // if address already filtered, return
		return false;

	osSemaphoreAcquire(SubscribersSemaphore, osWaitForever);
	configurations[index].filters.push_back(address);
	osSemaphoreRelease(SubscribersSemaphore);
	return true;
}

/**
 * @brief Remove an address filter from a subscriber
 *
 * @param input ID: subscriber ID
 * @param input address: module address filter to remove
 * @return whether successful
 */
bool MasterSubscribers::RemoveFilter(uint8_t ID, uint8_t address) {
	bool success;
	uint8_t index = FindSubscriberIndex(ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(SubscribersSemaphore, osWaitForever);
	for (uint8_t filter_index = 0; filter_index < configurations[index].filters.size(); filter_index++) {
		if (configurations[index].filters[filter_index] == address) {
			configurations[index].filters.erase(configurations[index].filters.begin() + filter_index);
			osSemaphoreRelease(SubscribersSemaphore);
			return true;
		}
	}
	osSemaphoreRelease(SubscribersSemaphore);
	return false;
}

/**
 * @brief Clear the registered address filters from a subscriber
 *
 * @param input ID: subscriber ID
 * @return whether successful
 */
bool MasterSubscribers::ClearFilters(uint8_t ID) {
	bool success;
	uint8_t index = FindSubscriberIndex(ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(SubscribersSemaphore, osWaitForever);
	configurations[index].filters.clear();
	osSemaphoreRelease(SubscribersSemaphore);
	return true;
}

/**
 * @brief Check that the address is accepted by the subscriber
 *
 * @param input ID: subscriber ID
 * @param input address: module address
 * @return true if address is accepted
 */
bool MasterSubscribers::AddressAccepted(uint8_t ID, uint8_t address) {
	bool success;
	uint8_t index = FindSubscriberIndex(ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(SubscribersSemaphore, osWaitForever);
	// If no filter setup, accept the input data
	if (configurations[index].filters.size() == 0) {
		osSemaphoreRelease(SubscribersSemaphore);
		return true;
	}

	// Check every filter
	for (uint8_t filtered_address : configurations[index].filters) {
		if (filtered_address == address) {
			osSemaphoreRelease(SubscribersSemaphore);
			return true;
		}
	}
	osSemaphoreRelease(SubscribersSemaphore);
	return false;
}

/**
 * @brief Get the subscriber index
 *
 * @param input ID: subscriber ID
 * @param output success: whether successful
 * @return the subscriber index
 */
uint8_t MasterSubscribers::FindSubscriberIndex(uint8_t ID, bool* success) {
	osSemaphoreAcquire(SubscribersSemaphore, osWaitForever);
	for (uint8_t index = 0; index < configurations.size(); index++) {
		if (configurations[index].ID == ID) {
			osSemaphoreRelease(SubscribersSemaphore);
			*success = true;
			return index;
		}
	}
	osSemaphoreRelease(SubscribersSemaphore);
	*success = false;
	return 0;
}

/**
 * @brief Check whether the subscriber is subscribed to the remote register configuration and interface
 *
 * @param input ID: subscriber ID
 * @param input remote_register: published register configuration
 * @param input interface: interface from which the published data comes from
 * @return true if the subscriber is subscribed to the input register configuration and interface
 */
bool MasterSubscribers::SubscribedToRemoteRegister(uint8_t ID, Register remote_register, SubscriberInterface interface) {
	bool success;
	FindRemoteRegisterIndex(ID, remote_register, interface, &success);
	return success;
}

/**
 * @brief Check if a subscriber can subscribe to the input register configuration and interface
 *
 * @param input remote_register: register configuration
 * @param input interface: interface from which the published data comes from
 * @return true if a subscriber can subscribe to the input register configuration and interface
 */
bool MasterSubscribers::RemoteRegisterSubscriptionAvailable(Register remote_register, SubscriberInterface interface) {
	osSemaphoreAcquire(SubscribersSemaphore, osWaitForever);
	for (SubscriberConfiguration &configuration : configurations) {
		for (uint8_t index = 0; index < configuration.remote_registers.size(); index++) {
			Register subscribed_register = configuration.remote_registers[index];
			SubscriberInterface subscribed_interface = configuration.interfaces[index];

			if (remote_register.address == subscribed_register.address) {
				if (interface.interface == subscribed_interface.interface || subscribed_interface.interface == ALL_INTERFACES || interface.interface == ALL_INTERFACES) {
					if (interface.address == subscribed_interface.address || subscribed_interface.address == ALL || interface.address == ALL) {
						if (remote_register.type != subscribed_register.type || remote_register.isArray != subscribed_register.isArray || (remote_register.isArray && (remote_register.length != subscribed_register.length))) {
							osSemaphoreRelease(SubscribersSemaphore);
							return false;
						}
					}
				}
			}
		}
	}
	osSemaphoreRelease(SubscribersSemaphore);
	return true;
}

/**
 * @brief Find a register configuration based on the register address and interface
 *
 * @param input remote_register: register configuration with only address filled
 * @param input interface: interface from which the published data comes from
 * @param output success: whether successful
 * @return the complete register configuration
 */
Register MasterSubscribers::FindRemoteRegister(Register remote_register, SubscriberInterface interface, bool* success) {
	uint8_t index;
	osSemaphoreAcquire(SubscribersSemaphore, osWaitForever);
	for (SubscriberConfiguration &configuration : configurations) {
		index = FindRemoteRegisterIndex(configuration.ID, remote_register, interface, success);
		if (*success) {
			Register found_register = configuration.remote_registers[index];
			osSemaphoreRelease(SubscribersSemaphore);
			return found_register;
		}
	}
	osSemaphoreRelease(SubscribersSemaphore);
	*success = false;
	return Register();
}

/**
 * @brief Find the register configuration index from a subscriber based on the register address and interface
 *
 * @param input ID: subscriber ID
 * @param input remote_register: register configuration with only address filled
 * @param input interface: interface from which the published data comes from
 * @param output success: whether successful
 * @return the register configuration index from the subscriber
 */
uint8_t MasterSubscribers::FindRemoteRegisterIndex(uint8_t ID, Register remote_register, SubscriberInterface interface, bool* success) {
	uint8_t subscriber_index = FindSubscriberIndex(ID, success);
	if (not *success)
		return 0;

	osSemaphoreAcquire(SubscribersSemaphore, osWaitForever);
	for (uint8_t index = 0; index < configurations[subscriber_index].remote_registers.size(); index++) {
		Register subscribed_register = configurations[subscriber_index].remote_registers[index];
		SubscriberInterface subscribed_interface = configurations[subscriber_index].interfaces[index];

		if (remote_register.address == subscribed_register.address) {
			if (interface.interface == subscribed_interface.interface || subscribed_interface.interface == ALL_INTERFACES || interface.interface == ALL_INTERFACES) {
				if (interface.address == subscribed_interface.address || subscribed_interface.address == ALL || interface.address == ALL) {
//					if (remote_register.type == subscribed_register.type && remote_register.isArray == subscribed_register.isArray && (!remote_register.isArray || (remote_register.length == subscribed_register.length))) {
					osSemaphoreRelease(SubscribersSemaphore);
					*success = true;
					return index;
//					}
				}
			}
		}
	}
	osSemaphoreRelease(SubscribersSemaphore);
	*success = false;
	return 0;

}
