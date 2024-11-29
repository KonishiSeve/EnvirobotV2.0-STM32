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
#define SUB_ALERT		0x02

// == Register map == //
//registers for the radio remote support
//The Radio PIC needs to map these to 0x00 - 0x04 (radio protocol addresses)
#define REG_REMOTE_MODE			0x0400
#define REG_REMOTE_ELT_NB		0x0401
#define REG_REMOTE_SPEED		0x0402
#define REG_REMOTE_FREQUENCY	0x0403
#define REG_REMOTE_DIRECTION	0x0409

//registers for meta parameters of the CPG
#define REG_CPG_SETPOINTS		0x0500
#define REG_CPG_ENABLED			0x0501
#define REG_CPG_RESET			0x0502
#define REG_CPG_NB_MODULES		0x0503

//registers for the CPG model
#define REG_CPG_FREQUENCY		0x0510
#define REG_CPG_DIRECTION		0x0511
#define REG_CPG_AMPLC			0x0512
#define REG_CPG_AMPLH			0x0513
#define REG_CPG_NWAVE			0x0514
#define REG_CPG_COUPLING_STRENGTH	0x0515
#define REG_CPG_A_R				0x0516

//miscellaneous
//set to 1 when any module detects a water leak
#define REG_ALERT_WATER			0x0600

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

	// === Radio Remote Registers Setup === //
	bool debug;
	//the remote needs to read 7 from the 0x00 (side radio address) mode register to start correctly
	static uint8_t reg_remote_mode = 7;
	registers->AddRegister<uint8_t>(REG_REMOTE_MODE);
	registers->SetRegisterAsSingle(REG_REMOTE_MODE);
	registers->AddRegisterPointer<uint8_t>(REG_REMOTE_MODE, &reg_remote_mode);
	registers->SetRegisterPermissions(REG_REMOTE_MODE, READ_PERMISSION);

	//This will display on the remote for 500ms when starting the robot
	static uint8_t reg_remote_elt_nb = MODULE_NUMBER;
	registers->AddRegister<uint8_t>(REG_REMOTE_ELT_NB);
	registers->SetRegisterAsSingle(REG_REMOTE_ELT_NB);
	registers->AddRegisterPointer<uint8_t>(REG_REMOTE_ELT_NB, &reg_remote_elt_nb);
	registers->SetRegisterPermissions(REG_REMOTE_MODE, READ_PERMISSION);

	//the remote tries to write a speed variable to it (controlled by up-down axis of the joystick), we use it to compute a frequency for the CPG
	static uint8_t reg_remote_speed = 0;
	registers->AddRegister<uint8_t>(REG_REMOTE_SPEED);
	registers->SetRegisterAsSingle(REG_REMOTE_SPEED);
	//registers->AddRegisterPointer<uint8_t>(REG_REMOTE_SPEED, &reg_remote_speed);
	registers->SetRegisterPermissions(REG_REMOTE_FREQUENCY, WRITE_PERMISSION);
	registers->AddWriteCallback<uint8_t>(REG_REMOTE_SPEED, argument,
		[](void* context , uint16_t register_ID , uint8_t* input , uint16_t length) -> bool {
		class_instances* class_instances_pointer = (class_instances*)context;
		Registers* registers = class_instances_pointer->registers;

		static float temp;
		temp = (((((float)(*input))/255.0)*2.0)-1.0)/0.75;	//invert the scaling done on the remote and get a value between -1 and 1 (more around -1 to 0.7 because of the bad stick calibration)
		//temp = (temp + 1)/2;	//scaling to have values from 0 to 1 for the CPG frequency
		registers->WriteRegister<float>(REG_CPG_FREQUENCY, &temp);
		return true;
	});

	//the remote write a frequency to this module, only 3 values exist (2 high values for swimming, and 1 low for walking) we switch from one to another by pressing the enter button on the remote
	//we use it turn the CPG on and off
	static uint8_t reg_remote_frequency = 0;
	registers->AddRegister<uint8_t>(REG_REMOTE_FREQUENCY);
	registers->SetRegisterAsSingle(REG_REMOTE_FREQUENCY);
	registers->SetRegisterPermissions(REG_REMOTE_FREQUENCY, WRITE_PERMISSION);
	registers->AddWriteCallback<uint8_t>(REG_REMOTE_FREQUENCY, argument,
		[](void* context , uint16_t register_ID , uint8_t* input , uint16_t length) -> bool {
		class_instances* class_instances_pointer = (class_instances*)context;
		Registers* registers = class_instances_pointer->registers;

		static uint8_t cpg_enabled = 0;
		if(*input > 0 && cpg_enabled==0) {
			cpg_enabled = 1;
			registers->WriteRegister<uint8_t>(REG_CPG_ENABLED, &cpg_enabled);
		}
		else if(cpg_enabled==1){
			cpg_enabled = 0;
			registers->WriteRegister<uint8_t>(REG_CPG_ENABLED, &cpg_enabled);
		}
		return true;
	});

	static uint8_t reg_remote_direction = 0;
	registers->AddRegister<uint8_t>(REG_REMOTE_DIRECTION);
	registers->SetRegisterAsSingle(REG_REMOTE_DIRECTION);
	//registers->AddRegisterPointer<uint8_t>(REG_REMOTE_DIRECTION, &reg_remote_direction);
	registers->SetRegisterPermissions(REG_REMOTE_DIRECTION, WRITE_PERMISSION);
	debug = registers->AddWriteCallback<uint8_t>(REG_REMOTE_DIRECTION, argument,
		[](void* context , uint16_t register_ID , uint8_t* input , uint16_t length) -> bool {
		class_instances* class_instances_pointer = (class_instances*)context;
		Registers* registers = class_instances_pointer->registers;
		LEDS* leds = class_instances_pointer->leds;

		leds->SetLED(LED_USER1, GPIO_PIN_SET);
		static float temp;
		temp = ((((float)(*input))/255.0)*2.0)-1;	//get back the raw stick value
		registers->WriteRegister<float>(REG_CPG_DIRECTION, &temp);
		return true;
	});
	if(!debug) {
		leds->SetLED(LED_USER2, GPIO_PIN_SET);
	}

	// === CPG Registers Setup === //
	//stores the output of the CPG (joint angles in degree), published on CANFD1
	static int8_t reg_cpg_setpoints[MODULE_NUMBER];
	registers->AddRegister<int8_t>(REG_CPG_SETPOINTS);
	registers->SetRegisterAsArray(REG_CPG_SETPOINTS, MODULE_NUMBER);
	registers->AddRegisterPointer<int8_t>(REG_CPG_SETPOINTS, reg_cpg_setpoints);

	//enables/disables the computation of CPG steps (supposed to be accessed by UART of CM4 or Radio PIC)
	static uint8_t reg_cpg_enabled = 0;
	registers->AddRegister<uint8_t>(REG_CPG_ENABLED);
	registers->SetRegisterAsSingle(REG_CPG_ENABLED);
	registers->AddRegisterPointer<uint8_t>(REG_CPG_ENABLED, &reg_cpg_enabled);

	//write only register, to reset the CPG states
	static uint8_t reg_cpg_reset;
	registers->AddRegister<uint8_t>(REG_CPG_RESET);
	registers->SetRegisterAsSingle(REG_CPG_RESET);
	registers->AddRegisterPointer<uint8_t>(REG_CPG_RESET, &reg_cpg_reset);
	registers->SetRegisterPermissions(REG_CPG_RESET, WRITE_PERMISSION);
	registers->AddWriteCallback<uint8_t>(REG_CPG_RESET, argument,
		[](void* context , uint16_t register_ID , uint8_t* input , uint16_t length) -> bool {
		cpg.reset();
		return true;
	});

	//CPG frequency register
	float reg_cpg_frequency = 3;
	registers->AddRegister<float>(REG_CPG_FREQUENCY);
	registers->SetRegisterAsSingle(REG_CPG_FREQUENCY);
	//registers->AddRegisterPointer<float>(REG_CPG_FREQUENCY, &reg_cpg_frequency);
	//registers->SetRegisterPermissions(REG_CPG_FREQUENCY, WRITE_PERMISSION);
	registers->AddWriteCallback<float>(REG_CPG_FREQUENCY, &reg_cpg_frequency,
		[](void* context , uint16_t register_ID , float* input , uint16_t length) -> bool {
		*((float*)(context)) = *input;
		cpg.set_frequency(*input);
		return true;
	});
	registers->AddReadCallback<float>(REG_CPG_FREQUENCY, &reg_cpg_frequency,
		[](void* context, uint16_t regiser_ID, float** output, uint16_t* length) -> bool {
		*output = (float*)(context);
		return true;
	});

	//CPG direction register
	static float reg_cpg_direction = 0;
	registers->AddRegister<float>(REG_CPG_DIRECTION);
	registers->SetRegisterAsSingle(REG_CPG_DIRECTION);
	registers->AddRegisterPointer<float>(REG_CPG_DIRECTION, &reg_cpg_direction);
	registers->SetRegisterPermissions(REG_CPG_DIRECTION, WRITE_PERMISSION);
	registers->AddWriteCallback<float>(REG_CPG_DIRECTION, argument,
		[](void* context , uint16_t register_ID , float* input , uint16_t length) -> bool {
		cpg.set_direction(*input);
		return true;
	});
	registers->AddReadCallback<float>(REG_CPG_DIRECTION, &reg_cpg_direction,
		[](void* context, uint16_t regiser_ID, float** output, uint16_t* length) -> bool {
		*output = (float*)(context);
		return true;
	});

	//CPG amplc register
	static float reg_cpg_amplc = 0.3;
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
	static float reg_cpg_amplh = 0.5;
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
	static float reg_cpg_nwave = 1;
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
	static float reg_cpg_coupling_strength = 50;
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
	static float reg_cpg_a_r = 10;
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
	cpg.init(MODULE_NUMBER, reg_cpg_frequency, reg_cpg_direction, reg_cpg_amplc, reg_cpg_amplc, reg_cpg_nwave, reg_cpg_coupling_strength, reg_cpg_a_r);
	int8_t setpoints[MODULE_NUMBER];

	uint32_t pic_data;
	uint16_t pic_length;

	for(;;) {
		if(reg_cpg_enabled) {
			leds->SetLED(LED_USER3, GPIO_PIN_SET);
			//compute 10 steps with 1ms stepsize
			for(uint32_t j=0;j<10;j++) {
				cpg.step(setpoints, 1);
			}
			registers->WriteRegister<int8_t>(REG_CPG_SETPOINTS, setpoints, MODULE_NUMBER);
			publishers->SpinPublisher(PUB_CPG);
		}
		else {
			leds->SetLED(LED_USER3, GPIO_PIN_RESET);
		}
		osDelay(10);
	}
}
