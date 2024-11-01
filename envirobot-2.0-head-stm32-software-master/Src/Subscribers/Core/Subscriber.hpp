/*
 * Subscriber.hpp
 *
 *  Created on: 7 déc. 2022
 *      Author: bignet
 */

#pragma once

#include <vector>
#include <stm32h750xx.h>
#include <stm32h7xx_hal.h>

#include "Registers/Registers.hpp"

#include "Configuration.h"
#include "Configurations/RegistersConfiguration.h"
#include "Configurations/CommunicationConfiguration.h"

// SubscriberInterface struct
struct SubscriberInterface {
	uint8_t interface;					// interface used to retrieve publisher inputs from which the publisher data comes from
	uint8_t address;					// address of the module from which the publisher data comes from
};

// SubscriberInput struct
struct SubscriberInput {
	Register register_;					// published register input
	SubscriberInterface interface;		// interface information for the input of the subscriber
	uint16_t length;					// data length of the incomming register
};

// Subscriber class used as a basis to derive custom subscribers
class Subscriber {
public:
	Subscriber();

#if defined(USE_UINT8_COMMUNICATION) && defined(USE_UINT8_REGISTER)
	/**
	 * @brief WEAK receive a uint8_t published register
	 */
	virtual void ReceiveUINT8(SubscriberInput information, const uint8_t* data) {}
#endif

#if defined(USE_UINT16_COMMUNICATION) && defined(USE_UINT16_REGISTER)
	/**
	 * @brief WEAK receive a uint16_t published register
	 */
	virtual void ReceiveUINT16(SubscriberInput information, const uint16_t* data) {}
#endif

#if defined(USE_UINT32_COMMUNICATION) && defined(USE_UINT32_REGISTER)
	/**
	 * @brief WEAK receive a uint32_t published register
	 */
	virtual void ReceiveUINT32(SubscriberInput information, const uint32_t* data) {}
#endif

#if defined(USE_UINT64_COMMUNICATION) && defined(USE_UINT64_REGISTER)
	/**
	 * @brief WEAK receive a uint64_t published register
	 */
	virtual void ReceiveUINT64(SubscriberInput information, const uint64_t* data) {}
#endif

#if defined(USE_INT8_COMMUNICATION) && defined(USE_INT8_REGISTER)
	/**
	 * @brief WEAK receive a int8_t published register
	 */
	virtual void ReceiveINT8(SubscriberInput information, const int8_t* data) {}
#endif

#if defined(USE_INT16_COMMUNICATION) && defined(USE_INT16_REGISTER)
	/**
	 * @brief WEAK receive a int16_t published register
	 */
	virtual void ReceiveINT16(SubscriberInput information, const int16_t* data) {}
#endif

#if defined(USE_INT32_COMMUNICATION) && defined(USE_INT32_REGISTER)
	/**
	 * @brief WEAK receive a int32_t published register
	 */
	virtual void ReceiveINT32(SubscriberInput information, const int32_t* data) {}
#endif

#if defined(USE_INT64_COMMUNICATION) && defined(USE_INT64_REGISTER)
	/**
	 * @brief WEAK receive a int64_t published register
	 */
	virtual void ReceiveINT64(SubscriberInput information, const int64_t* data) {}
#endif

#if defined(USE_FLOAT_COMMUNICATION) && defined(USE_FLOAT_REGISTER)
	/**
	 * @brief WEAK receive a float published register
	 */
	virtual void ReceiveFLOAT(SubscriberInput information, const float* data) {}
#endif

#if defined(USE_DOUBLE_COMMUNICATION) && defined(USE_DOUBLE_REGISTER)
	/**
	 * @brief WEAK receive a double published register
	 */
	virtual void ReceiveDOUBLE(SubscriberInput information, const double* data) {}
#endif
};
