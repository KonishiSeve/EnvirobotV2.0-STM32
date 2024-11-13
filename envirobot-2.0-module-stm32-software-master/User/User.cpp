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
			LEDS* leds_,
			Controller* controller_,
			Servomotors* servomotors_) {
	registers = registers_;
	subscribers = subscribers_;
	communication = communication_;
	services = services_;
	publishers = publishers_;
	hardware_delay = hardware_delay_;
	sensors = sensors_;
	leds = leds_;
	controller = controller_;
	servomotors = servomotors_;
}

/**
 * @brief Function called just before kernel Init and after Core classes init
 */
void User::Init(void) {
	// Insert USER Code Here
}

/**
 * @brief Function called just before kernel Start and after Core tasks registering
 */
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
			.controller = this->controller,
			.servomotors = this->servomotors
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

// ===== User Part ===== //
#define MODULE_NUMBER			4
#define REG_CPG_SETPOINTS		0x0500
#define SUB_CPG_SETPOINT		0x10

// ==
class SubCPGSetpoints: public Subscriber {
public:
	SubCPGSetpoints(Registers* registers_, LEDS* leds_, Controller* controller_) {
		registers = registers_;
		leds = leds_;
		controller = controller_;
	}
private:
	void ReceiveINT8(SubscriberInput information, const int8_t* data) {
		uint8_t module_id;
		uint16_t length;
		registers->ReadRegister<uint8_t>(REG_COM_ADDRESS, &module_id, &length);
		//check that the module got allocated an ID
		if(module_id != UNKNOWN) {
			leds->SetLED(LED_USER2, GPIO_PIN_SET);
			static int8_t copy;
			copy = *data;
			registers->WriteRegister<int8_t>(REG_CPG_SETPOINTS, &copy);
			controller->MoveTo((float)data[module_id-2]);
		}
	}
	Registers* registers;
	LEDS* leds;
	Controller* controller;
};


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
	Controller* controller = class_instances_pointer->controller;
	Servomotors* servomotors = class_instances_pointer->servomotors;

	// == Registers == //
	static int8_t reg_cpg_setpoints[MODULE_NUMBER];
	registers->AddRegister<int8_t>(REG_CPG_SETPOINTS);
	registers->SetRegisterAsArray(REG_CPG_SETPOINTS, MODULE_NUMBER);
	registers->AddRegisterPointer<int8_t>(REG_CPG_SETPOINTS, reg_cpg_setpoints);

	// == Subscriber == //
	static SubCPGSetpoints sub_setpoint(registers, leds, controller);
	subscribers->AddSubscriber(SUB_CPG_SETPOINT, &sub_setpoint);
	subscribers->SubscribeToRemoteRegister(SUB_CPG_SETPOINT, REG_CPG_SETPOINTS, SubscriberInterface{.interface=CANFD1 , .address=ALL});
	subscribers->ActivateSubscriber(SUB_CPG_SETPOINT);

	// == Motor Controller == //
	controller->SelectInputFilter(POSITION_MODE, OUTPUT_MOTOR_POSITION_FILTER);
	controller->trajectory_generator.SetTrajectoryMode(TRAJECTORY_STEP);

	for(;;) {
		leds->SetLED(LED_USER3, GPIO_PIN_SET);
		//controller->MoveTo(0.0f);
		osDelay(5000);
		leds->SetLED(LED_USER3, GPIO_PIN_RESET);
		//controller->MoveTo(20.0f);
		osDelay(5000);
	}
}
