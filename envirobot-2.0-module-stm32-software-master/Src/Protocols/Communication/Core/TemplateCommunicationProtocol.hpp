/*
 * TemplateProtocol.hpp
 *
 *  Created on: 30 nov. 2022
 *      Author: bignet
 */

#pragma once

#include "Protocols/Communication/Core/CommunicationProtocol.hpp"

// TemplateCommunicationProtocol class used as a basis to derive custom typed protocols
template<typename H>
class TemplateCommunicationProtocol : public CommunicationProtocol {
public:
	virtual void AddInterface(uint8_t ID, H* interface);

protected:
	H* GetInterface(uint8_t ID, bool* success);

	uint8_t GetID(void* pointer, bool* success);

	std::vector<H*> interfaces;
};

/**
 * @brief WEAK Add an hardware interface with a specific ID
 *
 * @param input ID: interface ID
 * @param input interface: the pointer to the hardware interface handle
 */
template<typename H>
void TemplateCommunicationProtocol<H>::AddInterface(uint8_t ID, H* interface) {
	AddInterfaceID(ID);
	AddInterfaceBuffer();
	interfaces.push_back(interface);
}

/**
 * @brief Get the hardware interface with a specific ID
 *
 * @param input ID: interface ID
 * @param output success: true whether successful
 * @return the pointer to the hardware interface handle associated to the input ID
 */
template<typename H>
H* TemplateCommunicationProtocol<H>::GetInterface(uint8_t ID, bool* success) {
	uint8_t index = GetInterfaceIndex(ID, success);
	if (*success) {
		return interfaces[index];
	}
	return NULL;
}

/**
 * @brief OVERRIDE Send a message over an interface with this protocol
 *
 * @param input pointer: the hardware interface handle pointer
 * @param output success: whether successful
 * @return the interface ID associated to the input handle
 */
template<typename H>
uint8_t TemplateCommunicationProtocol<H>::GetID(void* pointer, bool* success) {
	uint8_t index = 0;
	for (H* interface : interfaces) {
		if (interface == pointer)
			return GetIDFromIndex(index, success);
		index += 1;
	}
	*success = false;
	return 0;
}
