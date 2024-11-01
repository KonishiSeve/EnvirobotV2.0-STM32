/*
 * ExampleSubscriber.cpp
 *
 *  Created on: 8 déc. 2022
 *      Author: bignet
 */

#include <Subscribers/Example/ExampleSubscriber.hpp>

ExampleSubscriber::ExampleSubscriber() {
	// TODO Auto-generated constructor stub

}

#ifdef USE_TEMPLATE_RECEIVE

#if defined(USE_UINT8_COMMUNICATION) && defined(USE_UINT8_REGISTER)
void ExampleSubscriber::ReceiveUINT8(SubscriberInput information, const uint8_t* data) {
	Receive<uint8_t>(information, data);
}
#endif
#if defined(USE_UINT16_COMMUNICATION) && defined(USE_UINT16_REGISTER)
void ExampleSubscriber::ReceiveUINT16(SubscriberInput information, const uint16_t* data) {
	Receive<uint16_t>(information, data);
}
#endif
#if defined(USE_UINT32_COMMUNICATION) && defined(USE_UINT32_REGISTER)
void ExampleSubscriber::ReceiveUINT32(SubscriberInput information, const uint32_t* data) {
	Receive<uint32_t>(information, data);
}
#endif
#if defined(USE_UINT64_COMMUNICATION) && defined(USE_UINT64_REGISTER)
void ExampleSubscriber::ReceiveUINT64(SubscriberInput information, const uint64_t* data) {
	Receive<uint64_t>(information, data);
}
#endif
#if defined(USE_INT8_COMMUNICATION) && defined(USE_INT8_REGISTER)
void ExampleSubscriber::ReceiveINT8(SubscriberInput information, const int8_t* data) {
	Receive<int8_t>(information, data);
}
#endif
#if defined(USE_INT16_COMMUNICATION) && defined(USE_INT16_REGISTER)
void ExampleSubscriber::ReceiveINT16(SubscriberInput information, const int16_t* data) {
	Receive<int16_t>(information, data);
}
#endif
#if defined(USE_INT32_COMMUNICATION) && defined(USE_INT32_REGISTER)
void ExampleSubscriber::ReceiveINT32(SubscriberInput information, const int32_t* data) {
	Receive<int32_t>(information, data);
}
#endif
#if defined(USE_INT64_COMMUNICATION) && defined(USE_INT64_REGISTER)
void ExampleSubscriber::ReceiveINT64(SubscriberInput information, const int64_t* data) {
	Receive<int64_t>(information, data);
}
#endif
#if defined(USE_FLOAT_COMMUNICATION) && defined(USE_FLOAT_REGISTER)
void ExampleSubscriber::ReceiveFLOAT(SubscriberInput information, const float* data) {
	Receive<float>(information, data);
}
#endif
#if defined(USE_DOUBLE_COMMUNICATION) && defined(USE_DOUBLE_REGISTER)
void ExampleSubscriber::ReceiveDOUBLE(SubscriberInput information, const double* data){
	Receive<double>(information, data);
}
#endif

#endif
