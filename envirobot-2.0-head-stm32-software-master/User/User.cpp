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
#define REG_CPG_ENABLED			0x0501
#define REG_CPG_RESET			0x0502
#define REG_CPG_NB_MODULES		0x0503

#define REG_CPG_FREQUENCY		0x0510
#define REG_CPG_DIRECTION		0x0511
#define REG_CPG_AMPLC			0x0512
#define REG_CPG_AMPLH			0x0513
#define REG_CPG_NWAVE			0x0514
#define REG_CPG_COUPLING_STRENGTH	0x0515
#define REG_CPG_A_R				0x0516

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
	//stores the output of the CPG (joint angles in degree), published on CANFD1
	static int8_t reg_cpg_setpoints[MODULE_NUMBER];
	registers->AddRegister<int8_t>(REG_CPG_SETPOINTS);
	registers->SetRegisterAsArray(REG_CPG_SETPOINTS, MODULE_NUMBER);
	registers->AddRegisterPointer<int8_t>(REG_CPG_SETPOINTS, reg_cpg_setpoints);

	//enables/disables the computation of CPG steps (supposed to be accessed by UART of CM4 or Radio PIC)
	static int8_t reg_cpg_enabled = 1;
	registers->AddRegister<int8_t>(REG_CPG_ENABLED);
	registers->SetRegisterAsSingle(REG_CPG_ENABLED);
	registers->AddRegisterPointer<int8_t>(REG_CPG_ENABLED, &reg_cpg_enabled);

	//write only register, to reset the CPG states
	static int8_t reg_cpg_reset;
	registers->AddRegister<int8_t>(REG_CPG_RESET);
	registers->SetRegisterAsSingle(REG_CPG_RESET);
	registers->AddRegisterPointer<int8_t>(REG_CPG_RESET, &reg_cpg_reset);
	registers->SetRegisterPermissions(REG_CPG_RESET, WRITE_PERMISSION);
	registers->AddWriteCallback<int8_t>(REG_CPG_RESET, argument,
		[](void* context , uint16_t register_ID , int8_t* input , uint16_t length) -> bool {
		cpg.reset();
		return true;
	});

	//CPG frequency register
	static float reg_cpg_frequency;
	registers->AddRegister<float>(REG_CPG_FREQUENCY);
	registers->SetRegisterAsSingle(REG_CPG_FREQUENCY);
	registers->AddRegisterPointer<float>(REG_CPG_FREQUENCY, &reg_cpg_frequency);
	registers->SetRegisterPermissions(REG_CPG_FREQUENCY, WRITE_PERMISSION);
	registers->AddWriteCallback<float>(REG_CPG_FREQUENCY, argument,
		[](void* context , uint16_t register_ID , float* input , uint16_t length) -> bool {
		cpg.set_frequency(*input);
		return true;
	});

	//CPG direction register
	static float reg_cpg_direction;
	registers->AddRegister<float>(REG_CPG_DIRECTION);
	registers->SetRegisterAsSingle(REG_CPG_DIRECTION);
	registers->AddRegisterPointer<float>(REG_CPG_DIRECTION, &reg_cpg_direction);
	registers->SetRegisterPermissions(REG_CPG_DIRECTION, WRITE_PERMISSION);
	registers->AddWriteCallback<float>(REG_CPG_DIRECTION, argument,
		[](void* context , uint16_t register_ID , float* input , uint16_t length) -> bool {
		cpg.set_direction(*input);
		return true;
	});

	//CPG amplc register
	static float reg_cpg_amplc;
	registers->AddRegister<float>(REG_CPG_AMPLC);
	registers->SetRegisterAsSingle(REG_CPG_AMPLC);
	registers->AddRegisterPointer<float>(REG_CPG_AMPLC, &reg_cpg_amplc);
	registers->SetRegisterPermissions(REG_CPG_AMPLC, WRITE_PERMISSION);
	registers->AddWriteCallback<float>(REG_CPG_AMPLC, argument,
		[](void* context , uint16_t register_ID , float* input , uint16_t length) -> bool {
		cpg.set_amplc(*input);
		return true;
	});

	//CPG amplh register
	static float reg_cpg_amplh;
	registers->AddRegister<float>(REG_CPG_AMPLH);
	registers->SetRegisterAsSingle(REG_CPG_AMPLH);
	registers->AddRegisterPointer<float>(REG_CPG_AMPLH, &reg_cpg_amplh);
	registers->SetRegisterPermissions(REG_CPG_AMPLH, WRITE_PERMISSION);
	registers->AddWriteCallback<float>(REG_CPG_AMPLH, argument,
		[](void* context , uint16_t register_ID , float* input , uint16_t length) -> bool {
		cpg.set_amplh(*input);
		return true;
	});

	//CPG nwave register
	static float reg_cpg_nwave;
	registers->AddRegister<float>(REG_CPG_NWAVE);
	registers->SetRegisterAsSingle(REG_CPG_NWAVE);
	registers->AddRegisterPointer<float>(REG_CPG_NWAVE, &reg_cpg_nwave);
	registers->SetRegisterPermissions(REG_CPG_NWAVE, WRITE_PERMISSION);
	registers->AddWriteCallback<float>(REG_CPG_NWAVE, argument,
		[](void* context , uint16_t register_ID , float* input , uint16_t length) -> bool {
		cpg.set_nwave(*input);
		return true;
	});

	//CPG coupling strength register
	static float reg_cpg_coupling_strength;
	registers->AddRegister<float>(REG_CPG_COUPLING_STRENGTH);
	registers->SetRegisterAsSingle(REG_CPG_COUPLING_STRENGTH);
	registers->AddRegisterPointer<float>(REG_CPG_COUPLING_STRENGTH, &reg_cpg_coupling_strength);
	registers->SetRegisterPermissions(REG_CPG_COUPLING_STRENGTH, WRITE_PERMISSION);
	registers->AddWriteCallback<float>(REG_CPG_COUPLING_STRENGTH, argument,
		[](void* context , uint16_t register_ID , float* input , uint16_t length) -> bool {
		cpg.set_coupling_strength(*input);
		return true;
	});

	//CPG a_r register
	static float reg_cpg_a_r;
	registers->AddRegister<float>(REG_CPG_A_R);
	registers->SetRegisterAsSingle(REG_CPG_A_R);
	registers->AddRegisterPointer<float>(REG_CPG_A_R, &reg_cpg_a_r);
	registers->SetRegisterPermissions(REG_CPG_A_R, WRITE_PERMISSION);
	registers->AddWriteCallback<float>(REG_CPG_A_R, argument,
		[](void* context , uint16_t register_ID , float* input , uint16_t length) -> bool {
		cpg.set_a_r(*input);
		return true;
	});

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
		if(reg_cpg_enabled) {
			leds->SetLED(LED_USER3, GPIO_PIN_SET);
			for(uint32_t j=0;j<1000;j++) {
				cpg.step(setpoints, 10);
			}
			leds->SetLED(LED_USER3, GPIO_PIN_RESET);
			registers->WriteRegister<int8_t>(REG_CPG_SETPOINTS, setpoints, MODULE_NUMBER);
			publishers->SpinPublisher(PUB_CPG);
		}
		//leds->SetLED(LED_USER3, GPIO_PIN_SET);
		osDelay(50);
		//leds->SetLED(LED_USER3, GPIO_PIN_RESET);
		osDelay(50);
	}
}
