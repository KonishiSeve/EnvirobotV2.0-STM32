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
#include "Controller/Controller.hpp"
#include "Servomotors/Servomotors.hpp"
#include "LEDS/LEDs.hpp"
#include "Sensors/Sensors.hpp"
#include "HardwareDelay/HardwareDelay.hpp"

// User class used to implement new user freeRTOS tasks in parallel to the Core ones
class User {
public:
	User(	Registers* registers,
			MasterSubscribers* subscribers,
			Communication* communication,
			Services* services,
			Publishers* publishers,
			HardwareDelay* hardware_delay,
			Sensors* sensors,
			LEDS* leds,
			Controller* controller,
			Servomotors* servomotors
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
	Controller* controller;
	Servomotors* servomotors;
};

// ===== User Code ===== //

typedef struct class_instances {
	Registers* registers;
	MasterSubscribers* subscribers;
	Communication* communication;
	Services* services;
	Publishers* publishers;
	HardwareDelay* hardware_delay;
	Sensors* sensors;
	LEDS* leds;
	Controller* controller;
	Servomotors* servomotors;
} class_instances;

static void UserTask(void *argument);
