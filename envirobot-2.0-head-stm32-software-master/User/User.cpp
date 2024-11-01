/*
 * User.cpp
 *
 *  Created on: Jan 30, 2023
 *      Author: bignet
 */

#include "User.hpp"

User::User(	Registers* registers_,
			MasterSubscribers* subscribers_,
			Communication* communication_,
			Services* services_,
			Publishers* publishers_,
			HardwareDelay* hardware_delay_,
			Sensors* sensors_,
			LEDS* leds_) {
	registers = registers_;
	subscribers = subscribers_;
	communication = communication_;
	services = services_;
	publishers = publishers_;
	hardware_delay = hardware_delay_;
	sensors = sensors_;
	leds = leds_;
}

void User::Init(void) {
	// Insert USER Code Here
	// Called just before kernel init
}

void User::AddOSThreads(void) {
	// == save the class instance pointers in a struct to pass it to the tasks created == //
	static class_instances class_instances_argument = {
			.registers = this->registers,
			.subscribers = this->subscribers,
			.communication = this->communication,
			.services = this->services,
			.publishers = this->publishers,
			.hardware_delay = this->hardware_delay,
			.sensors = this->sensors,
			.leds = this->leds,
	};

	// == osThreadNew User freeRTOS threads == //
	//UserTask
	const osThreadAttr_t UserTask_attributes = {
		.name = "Test",
		.stack_size = 512 * 4,
		.priority = (osPriority_t) osPriorityLow,
	};
	osThreadNew(UserTask, &class_instances_argument, &UserTask_attributes);
}

static void UserTask(void *argument) {
	// == retrieving class instances == //
	class_instances* class_instances_pointer = (class_instances*)argument;
	Registers* registers = class_instances_pointer->registers;
	MasterSubscribers* subscribers = class_instances_pointer->subscribers;
	Communication* communication = class_instances_pointer->communication;
	Services* services = class_instances_pointer->services;
	Publishers* publishers = class_instances_pointer->publishers;
	HardwareDelay* hardware_delay = class_instances_pointer->hardware_delay;
	Sensors* sensors = class_instances_pointer->sensors;
	LEDS* leds = class_instances_pointer->leds;

	for(;;) {
		leds->SetLED(LED_USER3, GPIO_PIN_SET);
		osDelay(500);
		leds->SetLED(LED_USER3, GPIO_PIN_RESET);
		osDelay(500);
	}
}
