/*
 * ExampleSubscriber.hpp
 *
 *  Created on: 8 déc. 2022
 *      Author: bignet
 */

#pragma once

#define USE_TEMPLATE_RECEIVE // comment to remove the template function that groups all typed receive functions

#include <vector>
#include <stm32h750xx.h>
#include <stm32h7xx_hal.h>

#include "Subscribers/Core/Subscriber.hpp"

#include "Configuration.h"
#include "Configurations/RegistersConfiguration.h"
#include "Configurations/CommunicationConfiguration.h"

class ExampleSubscriber : public Subscriber {
public:
	ExampleSubscriber();
private:

#ifdef USE_TEMPLATE_RECEIVE
	template<typename T>
	void Receive(SubscriberInput information, const T* data); // receives single and arrays
#endif

#if defined(USE_UINT8_COMMUNICATION) && defined(USE_UINT8_REGISTER)
	void ReceiveUINT8(SubscriberInput information, const uint8_t* data);
#endif
#if defined(USE_UINT16_COMMUNICATION) && defined(USE_UINT16_REGISTER)
	void ReceiveUINT16(SubscriberInput information, const uint16_t* data);
#endif
#if defined(USE_UINT32_COMMUNICATION) && defined(USE_UINT32_REGISTER)
	void ReceiveUINT32(SubscriberInput information, const uint32_t* data);
#endif
#if defined(USE_UINT64_COMMUNICATION) && defined(USE_UINT64_REGISTER)
	void ReceiveUINT64(SubscriberInput information, const uint64_t* data);
#endif
#if defined(USE_INT8_COMMUNICATION) && defined(USE_INT8_REGISTER)
	void ReceiveINT8(SubscriberInput information, const int8_t* data);
#endif
#if defined(USE_INT16_COMMUNICATION) && defined(USE_INT16_REGISTER)
	void ReceiveINT16(SubscriberInput information, const int16_t* data);
#endif
#if defined(USE_INT32_COMMUNICATION) && defined(USE_INT32_REGISTER)
	void ReceiveINT32(SubscriberInput information, const int32_t* data);
#endif
#if defined(USE_INT64_COMMUNICATION) && defined(USE_INT64_REGISTER)
	void ReceiveINT64(SubscriberInput information, const int64_t* data);
#endif
#if defined(USE_FLOAT_COMMUNICATION) && defined(USE_FLOAT_REGISTER)
	void ReceiveFLOAT(SubscriberInput information, const float* data);
#endif
#if defined(USE_DOUBLE_COMMUNICATION) && defined(USE_DOUBLE_REGISTER)
	void ReceiveDOUBLE(SubscriberInput information, const double* data);
#endif
};

#ifdef USE_TEMPLATE_RECEIVE
template<typename T>
void ExampleSubscriber::Receive(SubscriberInput information, const T* data) {

}

#endif
