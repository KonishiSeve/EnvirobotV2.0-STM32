/*
 * User.hpp
 *
 *  Created on: Jan 30, 2023
 *      Author: bignet
 */

#pragma once

#include "cmsis_os.h"

#include "Registers/Registers.hpp"
#include "Communication/Communication.hpp"
#include "Services/Services.hpp"
#include "Publishers/Publishers.hpp"
#include "Subscribers/MasterSubscribers.hpp"
#include "LEDS/LEDs.hpp"
#include "Sensors/Sensors.hpp"
#include "HardwareDelay/HardwareDelay.hpp"

class User {
public:
	User(	Registers* registers,
			MasterSubscribers* subscribers,
			Communication* communication,
			Services* services,
			Publishers* publishers,
			HardwareDelay* hardware_delay,
			Sensors* sensors,
			LEDS* leds
	);
	void Init(void);
	void AddOSThreads(void);

private:
	Registers* registers;
	MasterSubscribers* subscribers;
	Communication* communication;
	Services* services;
	Publishers* publishers;
	HardwareDelay* hardware_delay;
	Sensors* sensors;
	LEDS* leds;
};

// === User Code === //

typedef struct class_instances {
	Registers* registers;
	MasterSubscribers* subscribers;
	Communication* communication;
	Services* services;
	Publishers* publishers;
	HardwareDelay* hardware_delay;
	Sensors* sensors;
	LEDS* leds;
} class_instances;

typedef struct reg_cb_context {
	Publishers* publishers;
	void* pointer;
} reg_cb_context;

typedef struct test_context {
	Publishers* _publishers;
	void* pointer;
} test_context;

static void UserTask(void *argument);
