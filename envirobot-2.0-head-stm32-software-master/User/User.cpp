/*
 * User.cpp
 *
 *  Created on: Jan 30, 2023
 *      Author: bignet
 */

#include "User.hpp"
#include "CPG.hpp"

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

#define MODULE_NUMBER	4
#define PUB_CPG			0x01
//Register map
#define REG_CPG_SETPOINTS		0x0500

CPG cpg;

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

	// === Registers Setup === //
	static int8_t reg_setpoints[MODULE_NUMBER];
	registers->AddRegister<int8_t>(REG_CPG_SETPOINTS);
	registers->SetRegisterAsArray(REG_CPG_SETPOINTS, MODULE_NUMBER);
	registers->AddRegisterPointer<int8_t>(REG_CPG_SETPOINTS, reg_setpoints);

	// === Publisher Setup === //
	publishers->AddPublisher(PUB_CPG);
	publishers->SetPublisherPrescaler(PUB_CPG, 1);
	publishers->LinkToInterface(PUB_CPG, CANFD1);
	publishers->SetPublishAddress(PUB_CPG, CANFD1, ALL);

	publishers->AddTopic(PUB_CPG, REG_CPG_SETPOINTS);
	publishers->ActivateTopic(PUB_CPG, REG_CPG_SETPOINTS);

	publishers->AddTopic(PUB_CPG, REG_TIMEBASE);
	publishers->ActivateTopic(PUB_CPG, REG_TIMEBASE);

	publishers->ActivatePublisher(PUB_CPG);

	// === CPG Setup === //
	cpg.init(MODULE_NUMBER, 0.5, 0, 0.3, 0.5, 1, 50, 10);
	int8_t setpoints[MODULE_NUMBER];
	for(;;) {
		for(uint8_t j=0;j<10;j++) {
			cpg.step(setpoints, 20);
		}
		registers->WriteRegister<int8_t>(REG_CPG_SETPOINTS, setpoints, MODULE_NUMBER);
		publishers->SpinPublisher(PUB_CPG);
		leds->SetLED(LED_USER3, GPIO_PIN_SET);
		osDelay(100);
		leds->SetLED(LED_USER3, GPIO_PIN_RESET);
		osDelay(100);
	}
}
