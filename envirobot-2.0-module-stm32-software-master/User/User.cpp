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
#define MODULE_NUMBER				4

// == Register map == //
#define REG_CPG_SETPOINTS			0x0500
#define REG_CPG_SETPOINT			0x0520	//setpoint for this module only

#define REG_REMOTE_MODE				0x0600

#define REG_ALERT_WATER				0x0700

#define SUB_GENERAL			0x00
#define PUB_ALERT_WATER		0x02


// == Publisher/Subscriber == //
class SubGeneral: public Subscriber {
public:
	SubGeneral(Registers* registers_, LEDS* leds_, Controller* controller_) {
		registers = registers_;
		leds = leds_;
		controller = controller_;
	}
private:
	void ReceiveINT8(SubscriberInput information, const int8_t* data) {
		//received setpoints register
		if(information.register_.address == REG_CPG_SETPOINTS) {
			uint8_t module_address;
			uint16_t length;
			registers->ReadRegister<uint8_t>(REG_COM_ADDRESS, &module_address, &length);
			if(*data == 1 && module_address != UNKNOWN) {
				leds->SetLED(LED_USER2, GPIO_PIN_SET);
				//store the setpoint for this module
				int8_t setpoint = data[module_address-2];
				registers->WriteRegister<int8_t>(REG_CPG_SETPOINT, &setpoint);
				controller->MoveTo((float)setpoint);
			}
		}
	}
	void ReceiveUINT8(SubscriberInput information, const uint8_t* data) {
		//received remote mode register
		if(information.register_.address == REG_REMOTE_MODE) {
			uint8_t module_address;
			uint16_t length;
			registers->ReadRegister<uint8_t>(REG_COM_ADDRESS, &module_address, &length);
			//check that an ID was assigned to the module before enabling the motor
			if(*data == 1 && module_address != UNKNOWN) {
				leds->SetLED(LED_USER1, GPIO_PIN_SET);
				//controller->ActivateBridge();
				//controller->ActivateController();
			}
			else {
				leds->SetLED(LED_USER1, GPIO_PIN_RESET);
				//controller->DeactivateBridge();
				//controller->DeactivateController();
			}
		}
	}
	Registers* registers;
	LEDS* leds;
	Controller* controller;
};

/*
class SubRemoteMode: public Subscriber {
public:
	SubRemoteMode(Registers* registers_, LEDS* leds_, Controller* controller_) {
		registers = registers_;
		leds = leds_;
		controller = controller_;
	}
private:
	void ReceiveINT8(SubscriberInput information, const int8_t* data) {
		leds->SetLED(LED_USER2, GPIO_PIN_SET);
		static uint8_t temp = 0;
		if(*data == 1) {
			leds->SetLED(LED_USER1, GPIO_PIN_SET);
			//controller->ActivateBridge();
			//controller->ActivateController();
			temp = 1;
			registers->WriteRegister<uint8_t>(REG_REMOTE_MODE, &temp, 1);
		}
		else {
			leds->SetLED(LED_USER1, GPIO_PIN_RESET);
			controller->DeactivateBridge();
			controller->DeactivateController();
			temp = 0;
			registers->WriteRegister<uint8_t>(REG_REMOTE_MODE, &temp, 1);
		}
	}
	Registers* registers;
	LEDS* leds;
	Controller* controller;
};
*/


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

	static int8_t reg_cpg_setpoint = 0;
	registers->AddRegister<int8_t>(REG_CPG_SETPOINT);
	registers->SetRegisterAsSingle(REG_CPG_SETPOINT);
	registers->AddRegisterPointer<int8_t>(REG_CPG_SETPOINT, &reg_cpg_setpoint);

	//defines if the remote started the robot
	static uint8_t reg_remote_mode = 0;
	registers->AddRegister<uint8_t>(REG_REMOTE_MODE);
	registers->SetRegisterAsSingle(REG_REMOTE_MODE);
	registers->AddRegisterPointer<uint8_t>(REG_REMOTE_MODE, &reg_remote_mode);

	//set to 1 if there is a water leak detected
	static uint8_t reg_alert_water = 0;
	registers->AddRegister<uint8_t>(REG_ALERT_WATER);
	registers->SetRegisterAsSingle(REG_ALERT_WATER);
	registers->AddRegisterPointer<uint8_t>(REG_ALERT_WATER, &reg_alert_water);

	// == Subscribers == //
	static SubGeneral sub_general(registers, leds, controller);
	subscribers->AddSubscriber(SUB_GENERAL, &sub_general);
	subscribers->SubscribeToRemoteRegister(SUB_GENERAL, REG_CPG_SETPOINTS, SubscriberInterface{.interface=CANFD1 , .address=ALL});
	subscribers->SubscribeToRemoteRegister(SUB_GENERAL, REG_REMOTE_MODE, SubscriberInterface{.interface=CANFD1 , .address=ALL});
	subscribers->ActivateSubscriber(SUB_GENERAL);

	/* [DEL]
	static SubRemoteMode sub_remote_mode(registers, leds, controller);
	subscribers->AddSubscriber(SUB_REMOTE_MODE, &sub_remote_mode);
	subscribers->SubscribeToRemoteRegister(SUB_REMOTE_MODE, REG_REMOTE_MODE, SubscriberInterface{.interface=CANFD1 , .address=ALL});
	subscribers->ActivateSubscriber(SUB_REMOTE_MODE);
	*/

	// == Publishers == //
	publishers->AddPublisher(PUB_ALERT_WATER);
	publishers->SetPublisherPrescaler(PUB_ALERT_WATER, 1);
	publishers->LinkToInterface(PUB_ALERT_WATER, CANFD1);
	publishers->SetPublishAddress(PUB_ALERT_WATER, CANFD1, 0xFF);

	publishers->AddTopic(PUB_ALERT_WATER, REG_ALERT_WATER);
	publishers->ActivateTopic(PUB_ALERT_WATER, REG_ALERT_WATER);
	publishers->ActivatePublisher(PUB_ALERT_WATER);

	//publish the water alert register in the same packet as the sensor readings
	//publishers->AddTopic(PUBLISHER_MOTOR, REG_ALERT_WATER);
	//publishers->ActivateTopic(PUBLISHER_MOTOR, REG_ALERT_WATER);

	// == Motor Controller == //
	controller->SelectInputFilter(POSITION_MODE, OUTPUT_MOTOR_POSITION_FILTER);
	controller->trajectory_generator.SetTrajectoryMode(TRAJECTORY_STEP);

	for(;;) {
		/*
		uint8_t module_id;
		uint16_t length;
		//Check that an ID was assigned to the module
		registers->ReadRegister<uint8_t>(REG_COM_ADDRESS, &module_id, &length);
		if(module_id != UNKNOWN && reg_remote_mode==1) {
			//controller->MoveTo((float)reg_cpg_setpoints[module_id-2]);
		}
		*/
		publishers->SpinPublisher(PUB_ALERT_WATER);
		osDelay(1000);
		/*
		leds->SetLED(LED_USER3, GPIO_PIN_SET);
		//controller->MoveTo(0.0f);
		osDelay(1000);
		leds->SetLED(LED_USER3, GPIO_PIN_RESET);
		//controller->MoveTo(20.0f);
		osDelay(1000);
		*/
	}
}
