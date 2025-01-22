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
	//save the class instance pointers in a struct to pass it to the tasks created
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

/* ========== USER CODE ============= */
/* ========== USER CODE ============= */
/* ========== USER CODE ============= */

//number of modules (=number of motors/joints, not including the head or the tail)
#define MODULE_NUMBER	4
//max duration between remote packets before turning off CPG
#define REMOTE_TIMEOUT_MS	200
//main loop period (should be a multiple of the integration stepsize)
#define LOOP_TIME_MS 	10
//discrete integration stepsize for the CPG oscillator states
#define CPG_STEPSIZE_MS	1

// == Register map == //
//registers to deal with the IO of the CPG controller
#define REG_CPG_SETPOINTS		0x0500
#define REG_CPG_ENABLED			0x0501
#define REG_CPG_RESET			0x0502

//registers for the CPG model parameters
#define REG_CPG_FREQUENCY		0x0510
#define REG_CPG_DIRECTION		0x0511
#define REG_CPG_AMPLC			0x0512
#define REG_CPG_AMPLH			0x0513
#define REG_CPG_NWAVE			0x0514
#define REG_CPG_COUPLING_STRENGTH	0x0515
#define REG_CPG_A_R					0x0516

//registers for the joystick position to CPG parameter conversion (when using the remote)
#define REG_CPG_DIRECTION_MAX	0x0520
#define REG_CPG_AMPLC_MAX		0x0521
#define REG_CPG_AMPLH_MAX		0x0522

//registers for the radio remote support
//The Radio PIC needs to map these to 0x00 - 0x04 (legacy addresses)
#define REG_REMOTE_MODE			0x0600
#define REG_REMOTE_ELT_NB		0x0601
#define REG_REMOTE_SPEED		0x0602
#define REG_REMOTE_FREQUENCY	0x0603
#define REG_REMOTE_DIRECTION	0x0604

#define REG_REMOTE_LAST_RX		0x0610

//registers for alerts
#define REG_ALERT_WATER			0x0700
#define REG_ALERT_WATER_RADIO	0x0701

// == Publisher/Subscriber map == //
#define PUB_CPG_SETPOINTS	0x10
#define PUB_REMOTE_MODE		0x11
#define PUB_CPG_PARAM		0x12
#define PUB_CPG_FAST		0x13
#define PUB_CPG_ENABLED		0x14
#define SUB_GENERAL			0x10

class SubGeneral: public Subscriber {
public:
	SubGeneral(Registers* registers_, LEDS* leds_) {
		registers = registers_;
		leds = leds_;
	}
private:
	void ReceiveUINT8(SubscriberInput information, const uint8_t* data) {
		//if the received register is REG_ALERT_WATER
		if(information.register_.address == REG_ALERT_WATER) {
			static uint8_t already_alert = 0;
			//If value is 1, turn water LED on and put the module address in REG_ALERT_WATER_RADIO
			if(*data == 1 && already_alert == 0) {
				already_alert = 1;
				leds->SetLED(LED_USER3, GPIO_PIN_SET);
				uint32_t value = (0x65<<24) | (0x43<<16) | (0x21<<8) | (information.interface.address&0xFF);
				registers->WriteRegister<uint32_t>(REG_ALERT_WATER_RADIO, &value, 1);
			}
		}
	}
	Registers* registers;
	LEDS* leds;
};

// CPG class instance
CPG cpg;

static void UserTask(void *argument) {
	// == retrieving class instance pointers from argument == //
	class_instances* class_instances_pointer = (class_instances*)argument;
	Registers* registers = class_instances_pointer->registers;
	MasterSubscribers* subscribers = class_instances_pointer->subscribers;
	Communication* communication = class_instances_pointer->communication;
	Services* services = class_instances_pointer->services;
	Publishers* publishers = class_instances_pointer->publishers;
	HardwareDelay* hardware_delay = class_instances_pointer->hardware_delay;
	Sensors* sensors = class_instances_pointer->sensors;
	LEDS* leds = class_instances_pointer->leds;


	// === CPG Registers Setup === //
	// = stores the output of the CPG (joint angles in degree), published on CANFD1
	static int8_t reg_cpg_setpoints[MODULE_NUMBER];
	registers->AddRegister<int8_t>(REG_CPG_SETPOINTS);
	registers->SetRegisterAsArray(REG_CPG_SETPOINTS, MODULE_NUMBER);
	registers->AddRegisterPointer<int8_t>(REG_CPG_SETPOINTS, reg_cpg_setpoints);

	// = enables/disables the computation of CPG steps
	static uint8_t reg_cpg_enabled = 0;
	registers->AddRegister<uint8_t>(REG_CPG_ENABLED);
	registers->SetRegisterAsSingle(REG_CPG_ENABLED);
	registers->AddWriteCallback<uint8_t>(REG_CPG_ENABLED, argument,
		[](void* context , uint16_t register_ID , uint8_t* input , uint16_t length) -> bool {
		//context and variables
		class_instances* class_instances_pointer = (class_instances*)context;
		Publishers* publishers = class_instances_pointer->publishers;
		//notify the CM4 of the change
		publishers->SpinPublisher(PUB_CPG_ENABLED);
		return true;
	});
	registers->AddRegisterPointer<uint8_t>(REG_CPG_ENABLED, &reg_cpg_enabled);

	// = write only register, to reset the CPG states
	static uint8_t reg_cpg_reset = 1;
	registers->AddRegister<uint8_t>(REG_CPG_RESET);
	registers->SetRegisterAsSingle(REG_CPG_RESET);
	registers->AddWriteCallback<uint8_t>(REG_CPG_RESET, argument,
		[](void* context , uint16_t register_ID , uint8_t* input , uint16_t length) -> bool {
		//context and variables
		class_instances* class_instances_pointer = (class_instances*)context;
		Publishers* publishers = class_instances_pointer->publishers;
		//change the CPG model parameter
		cpg.reset();
		//notify the CM4
		publishers->ActivateTopic(PUB_CPG_PARAM, REG_CPG_RESET);
		publishers->SpinPublisher(PUB_CPG_PARAM);
		publishers->DeactivateTopic(PUB_CPG_PARAM, REG_CPG_RESET);
		return true;
	});
	registers->AddRegisterPointer<uint8_t>(REG_CPG_RESET, &reg_cpg_reset);

	// = CPG frequency register
	static float reg_cpg_frequency = 1;
	registers->AddRegister<float>(REG_CPG_FREQUENCY);
	registers->SetRegisterAsSingle(REG_CPG_FREQUENCY);
	registers->AddWriteCallback<float>(REG_CPG_FREQUENCY, argument,
		[](void* context , uint16_t register_ID , float* input , uint16_t length) -> bool {
		//context and variables
		class_instances* class_instances_pointer = (class_instances*)context;
		Publishers* publishers = class_instances_pointer->publishers;
		//change the CPG model parameter
		cpg.set_frequency(*input);
		//notify the CM4 of the change
		publishers->ActivateTopic(PUB_CPG_PARAM, REG_CPG_FREQUENCY);
		publishers->SpinPublisher(PUB_CPG_PARAM);
		publishers->DeactivateTopic(PUB_CPG_PARAM, REG_CPG_FREQUENCY);
		return true;
	});
	registers->AddRegisterPointer<float>(REG_CPG_FREQUENCY, &reg_cpg_frequency);

	// = CPG direction register
	static float reg_cpg_direction = 0;
	registers->AddRegister<float>(REG_CPG_DIRECTION);
	registers->SetRegisterAsSingle(REG_CPG_DIRECTION);
	registers->AddWriteCallback<float>(REG_CPG_DIRECTION, argument,
		[](void* context , uint16_t register_ID , float* input , uint16_t length) -> bool {
		//context and variables
		class_instances* class_instances_pointer = (class_instances*)context;
		Registers* registers = class_instances_pointer->registers;
		Publishers* publishers = class_instances_pointer->publishers;
		static uint8_t cpg_enabled;
		static uint16_t len;
		//change the CPG model parameter
		cpg.set_direction(*input);
		//only notify the CM4 if the parameter change did not come from the radio remote (too fast to enable/disable topics at each update)
		registers->ReadRegister<uint8_t>(REG_CPG_ENABLED, &cpg_enabled, &len);
		if(cpg_enabled != 1) {
			//notify the CM4 of the change
			publishers->ActivateTopic(PUB_CPG_PARAM, REG_CPG_DIRECTION);
			publishers->SpinPublisher(PUB_CPG_PARAM);
			publishers->DeactivateTopic(PUB_CPG_PARAM, REG_CPG_DIRECTION);
		}
		return true;
	});
	registers->AddRegisterPointer<float>(REG_CPG_DIRECTION, &reg_cpg_direction);

	// = CPG amplc register
	static float reg_cpg_amplc = 0.2;
	registers->AddRegister<float>(REG_CPG_AMPLC);
	registers->SetRegisterAsSingle(REG_CPG_AMPLC);
	registers->AddWriteCallback<float>(REG_CPG_AMPLC, argument,
		[](void* context , uint16_t register_ID , float* input , uint16_t length) -> bool {
		//context and variables
		class_instances* class_instances_pointer = (class_instances*)context;
		Registers* registers = class_instances_pointer->registers;
		Publishers* publishers = class_instances_pointer->publishers;
		static uint8_t cpg_enabled;
		static uint16_t len;
		//change the CPG model parameter
		cpg.set_amplc(*input);
		//only notify the CM4 if the parameter change did not come from the radio remote (frequency too high to enable/disable topics)
		registers->ReadRegister<uint8_t>(REG_CPG_ENABLED, &cpg_enabled, &len);
		if(cpg_enabled != 1) {
			//notify the CM4 of the change
			publishers->ActivateTopic(PUB_CPG_PARAM, REG_CPG_AMPLC);
			publishers->SpinPublisher(PUB_CPG_PARAM);
			publishers->DeactivateTopic(PUB_CPG_PARAM, REG_CPG_AMPLC);
		}
		return true;
	});
	registers->AddRegisterPointer<float>(REG_CPG_AMPLC, &reg_cpg_amplc);

	// = CPG amplh register
	static float reg_cpg_amplh = 0.2;
	registers->AddRegister<float>(REG_CPG_AMPLH);
	registers->SetRegisterAsSingle(REG_CPG_AMPLH);
	registers->AddWriteCallback<float>(REG_CPG_AMPLH, argument,
		[](void* context , uint16_t register_ID , float* input , uint16_t length) -> bool {
		//context and variables
		class_instances* class_instances_pointer = (class_instances*)context;
		Registers* registers = class_instances_pointer->registers;
		Publishers* publishers = class_instances_pointer->publishers;
		static uint8_t cpg_enabled;
		static uint16_t len;
		//change the CPG model parameter
		cpg.set_amplh(*input);
		//only notify the CM4 if the parameter change did not came from the radio remote (frequency too high to enable/disable topics)
		registers->ReadRegister<uint8_t>(REG_CPG_ENABLED, &cpg_enabled, &len);
		if(cpg_enabled != 1) {
			//notify the CM4 of the change
			publishers->ActivateTopic(PUB_CPG_PARAM, REG_CPG_AMPLH);
			publishers->SpinPublisher(PUB_CPG_PARAM);
			publishers->DeactivateTopic(PUB_CPG_PARAM, REG_CPG_AMPLH);
		}
		return true;
	});
	registers->AddRegisterPointer<float>(REG_CPG_AMPLH, &reg_cpg_amplh);

	// = CPG nwave register
	static float reg_cpg_nwave = 1;
	registers->AddRegister<float>(REG_CPG_NWAVE);
	registers->SetRegisterAsSingle(REG_CPG_NWAVE);
	registers->AddWriteCallback<float>(REG_CPG_NWAVE, argument,
		[](void* context , uint16_t register_ID , float* input , uint16_t length) -> bool {
		//context and variables
		class_instances* class_instances_pointer = (class_instances*)context;
		Publishers* publishers = class_instances_pointer->publishers;
		//change the CPG model parameter
		cpg.set_nwave(*input);
		//notify the CM4 of the change
		publishers->ActivateTopic(PUB_CPG_PARAM, REG_CPG_NWAVE);
		publishers->SpinPublisher(PUB_CPG_PARAM);
		publishers->DeactivateTopic(PUB_CPG_PARAM, REG_CPG_NWAVE);
		return true;
	});
	registers->AddRegisterPointer<float>(REG_CPG_NWAVE, &reg_cpg_nwave);

	// = CPG coupling strength register
	static float reg_cpg_coupling_strength = 50;
	registers->AddRegister<float>(REG_CPG_COUPLING_STRENGTH);
	registers->SetRegisterAsSingle(REG_CPG_COUPLING_STRENGTH);
	registers->AddWriteCallback<float>(REG_CPG_COUPLING_STRENGTH, argument,
		[](void* context , uint16_t register_ID , float* input , uint16_t length) -> bool {
		//context and variables
		class_instances* class_instances_pointer = (class_instances*)context;
		Publishers* publishers = class_instances_pointer->publishers;
		//change the CPG model parameter
		cpg.set_coupling_strength(*input);
		//notify the CM4 of the change
		publishers->ActivateTopic(PUB_CPG_PARAM, REG_CPG_COUPLING_STRENGTH);
		publishers->SpinPublisher(PUB_CPG_PARAM);
		publishers->DeactivateTopic(PUB_CPG_PARAM, REG_CPG_COUPLING_STRENGTH);
		return true;
	});
	registers->AddRegisterPointer<float>(REG_CPG_COUPLING_STRENGTH, &reg_cpg_coupling_strength);

	// = CPG a_r register
	static float reg_cpg_a_r = 10;
	registers->AddRegister<float>(REG_CPG_A_R);
	registers->SetRegisterAsSingle(REG_CPG_A_R);
	registers->AddWriteCallback<float>(REG_CPG_A_R, argument,
		[](void* context , uint16_t register_ID , float* input , uint16_t length) -> bool {
		//context and variables
		class_instances* class_instances_pointer = (class_instances*)context;
		Publishers* publishers = class_instances_pointer->publishers;
		//change the CPG model parameter
		cpg.set_a_r(*input);
		//notify the CM4 of the change
		publishers->ActivateTopic(PUB_CPG_PARAM, REG_CPG_A_R);
		publishers->SpinPublisher(PUB_CPG_PARAM);
		publishers->DeactivateTopic(PUB_CPG_PARAM, REG_CPG_A_R);
		return true;
	});
	registers->AddRegisterPointer<float>(REG_CPG_A_R, &reg_cpg_a_r);

	// = CPG MAX direction
	static float reg_cpg_direction_max = 1.0;
	registers->AddRegister<float>(REG_CPG_DIRECTION_MAX);
	registers->SetRegisterAsSingle(REG_CPG_DIRECTION_MAX);
	registers->AddRegisterPointer<float>(REG_CPG_DIRECTION_MAX, &reg_cpg_direction_max);

	// = CPG MAX amplc
	static float reg_cpg_amplc_max = 0.2;
	registers->AddRegister<float>(REG_CPG_AMPLC_MAX);
	registers->SetRegisterAsSingle(REG_CPG_AMPLC_MAX);
	registers->AddRegisterPointer<float>(REG_CPG_AMPLC_MAX, &reg_cpg_amplc_max);

	// = CPG MAX amplh
	static float reg_cpg_amplh_max = 0.2;
	registers->AddRegister<float>(REG_CPG_AMPLH_MAX);
	registers->SetRegisterAsSingle(REG_CPG_AMPLH_MAX);
	registers->AddRegisterPointer<float>(REG_CPG_AMPLH_MAX, &reg_cpg_amplh_max);

	// === Radio Remote Registers Setup === //
	//The remote writes 1 to it to start the robot and 0 to stop it (for the "ENVIROBOT" profile)
	static uint8_t reg_remote_mode = 0;
	registers->AddRegister<uint8_t>(REG_REMOTE_MODE);
	registers->SetRegisterAsSingle(REG_REMOTE_MODE);
	registers->AddWriteCallback<uint8_t>(REG_REMOTE_MODE, argument,
		[](void* context , uint16_t register_ID , uint8_t* input , uint16_t length) -> bool {
		//context and variables
		class_instances* class_instances_pointer = (class_instances*)context;
		Publishers* publishers = class_instances_pointer->publishers;
		//notify the CM4 of the change
		publishers->ActivateTopic(PUB_CPG_PARAM, REG_REMOTE_MODE);
		publishers->SpinPublisher(PUB_CPG_PARAM);
		publishers->DeactivateTopic(PUB_CPG_PARAM, REG_REMOTE_MODE);
		return true;
	});
	registers->AddRegisterPointer<uint8_t>(REG_REMOTE_MODE, &reg_remote_mode);

	//Not used on the "ENVIROBOT" profile
	static uint8_t reg_remote_elt_nb = MODULE_NUMBER;
	registers->AddRegister<uint8_t>(REG_REMOTE_ELT_NB);
	registers->SetRegisterAsSingle(REG_REMOTE_ELT_NB);
	registers->AddRegisterPointer<uint8_t>(REG_REMOTE_ELT_NB, &reg_remote_elt_nb);

	//The remote writes a "speed" variable to this register, we use it to retrieve the up-down axis value of the joystick
	registers->AddRegister<uint8_t>(REG_REMOTE_SPEED);
	registers->SetRegisterAsSingle(REG_REMOTE_SPEED);
	registers->SetRegisterPermissions(REG_REMOTE_SPEED, WRITE_PERMISSION);
	registers->AddWriteCallback<uint8_t>(REG_REMOTE_SPEED, argument,
		[](void* context , uint16_t register_ID , uint8_t* input , uint16_t length) -> bool {
		//context and variables
		class_instances* class_instances_pointer = (class_instances*)context;
		Registers* registers = class_instances_pointer->registers;
		Publishers* publishers = class_instances_pointer->publishers;
		static uint32_t timestamp;
		static uint8_t cpg_enabled;
		static float cpg_ampl_max;
		static float cpg_ampl_new;
		static float cpg_ampl;
		static uint16_t len;
		//save the timestamp
		registers->ReadRegister<uint32_t>(REG_TIMEBASE, &timestamp, &len);
		registers->WriteRegister<uint32_t>(REG_REMOTE_LAST_RX, &timestamp, len);
		//enable the CPG (if not already)
		registers->ReadRegister<uint8_t>(REG_CPG_ENABLED, &cpg_enabled, &len);
		if(cpg_enabled == 0) {
			cpg_enabled = 1;
			registers->WriteRegister<uint8_t>(REG_CPG_ENABLED, &cpg_enabled, 1);	//enable CPG
		}
		//Compute the CPG amplitudes and update them if needed
		registers->ReadRegister<float>(REG_CPG_AMPLC_MAX, &cpg_ampl_max, &len);
		cpg_ampl_new = *input/45.0;					//get raw stick values [0.0 to 1.0]
		cpg_ampl_new = cpg_ampl_new*cpg_ampl_max;	//convert the stick value to a CPG amplitude value
		registers->ReadRegister<float>(REG_CPG_AMPLC, &cpg_ampl, &len);
		if(cpg_ampl != cpg_ampl_new) {	//check if it is a new value
			//update the amplc value
			registers->WriteRegister<float>(REG_CPG_AMPLC, &cpg_ampl_new);
			//update the amplh value
			registers->ReadRegister<float>(REG_CPG_AMPLH_MAX, &cpg_ampl_max, &len);
			cpg_ampl_new = *input/45.0;
			cpg_ampl_new = cpg_ampl_new*cpg_ampl_max;
			registers->WriteRegister<float>(REG_CPG_AMPLH, &cpg_ampl_new);
			//notify the CM4
			publishers->SpinPublisher(PUB_CPG_FAST);
		}
		return true;
	});


	//The remote writes a "frequency" variable to this register, only 2 values exist (0 or 10 depending on the speed)
	//This is used as a dummy register since the remote writes to it but the values have no use to us
	uint8_t reg_remote_frequency;
	registers->AddRegister<uint8_t>(REG_REMOTE_FREQUENCY);
	registers->SetRegisterAsSingle(REG_REMOTE_FREQUENCY);
	registers->AddRegisterPointer<uint8_t>(REG_REMOTE_FREQUENCY, &reg_remote_frequency);
	registers->SetRegisterPermissions(REG_REMOTE_FREQUENCY, WRITE_PERMISSION);

	//The remote writes a "direction" variable to this register, we use it to retrieve the left-right axis value of the joystick
	registers->AddRegister<int8_t>(REG_REMOTE_DIRECTION);
	registers->SetRegisterAsSingle(REG_REMOTE_DIRECTION);
	registers->SetRegisterPermissions(REG_REMOTE_DIRECTION, WRITE_PERMISSION);
	registers->AddWriteCallback<int8_t>(REG_REMOTE_DIRECTION, argument,
		[](void* context , uint16_t register_ID , int8_t* input , uint16_t length) -> bool {
		//context and variables
		class_instances* class_instances_pointer = (class_instances*)context;
		Registers* registers = class_instances_pointer->registers;
		Publishers* publishers = class_instances_pointer->publishers;
		static float cpg_direction;
		static float cpg_direction_max;
		static float cpg_direction_new;
		static uint32_t timestamp;
		static uint8_t cpg_enabled;
		static uint16_t len;
		//save the timestamp
		registers->ReadRegister<uint32_t>(REG_TIMEBASE, &timestamp, &len);
		registers->WriteRegister<uint32_t>(REG_REMOTE_LAST_RX, &timestamp, len);
		//enable the CPG (if not already)
		registers->ReadRegister<uint8_t>(REG_CPG_ENABLED, &cpg_enabled, &len);
		if(cpg_enabled == 0) {
			cpg_enabled = 1;
			registers->WriteRegister<uint8_t>(REG_CPG_ENABLED, &cpg_enabled);
		}
		//retrieve the CPG direction value and update it
		registers->ReadRegister<float>(REG_CPG_DIRECTION_MAX, &cpg_direction_max, &len);
		cpg_direction_new = ((float)(*input))/100.0;				//get raw stick value [-1.0 to 1.0]
		cpg_direction_new = cpg_direction_new*cpg_direction_max;	//convert the stick value to a CPG direction value
		registers->ReadRegister<float>(REG_CPG_DIRECTION, &cpg_direction, &len);
		if(cpg_direction != cpg_direction_new) {
			registers->WriteRegister<float>(REG_CPG_DIRECTION, &cpg_direction_new);
			publishers->SpinPublisher(PUB_CPG_FAST);
		}
		return true;
	});

	//stores the last time (from the timebase register) a direction or speed was received from the remote
	//used to turn the CPG on and off by switching the gait mode (walking/swimming) with remote
	static uint32_t reg_remote_last_rx = 0;
	registers->AddRegister<uint32_t>(REG_REMOTE_LAST_RX);
	registers->SetRegisterAsSingle(REG_REMOTE_LAST_RX);
	registers->AddRegisterPointer<uint32_t>(REG_REMOTE_LAST_RX, &reg_remote_last_rx);

	// === Alert Registers Setup === //
	//Contains the number of the module where there is a water leak (or 0 if no leak)
	static uint8_t reg_alert_water = 0;
	registers->AddRegister<uint8_t>(REG_ALERT_WATER);
	registers->SetRegisterAsSingle(REG_ALERT_WATER);
	registers->AddRegisterPointer<uint8_t>(REG_ALERT_WATER, &reg_alert_water);

	//contains the same information as the reg_alert_water register but the 3 MSB keep a constant value
	//content: (0x654321ZZ) ZZ being the reg_alert_water register content
	//allows the Radio PC client to read it while the remote is running and be sure that the received packet is the correct one
	static uint32_t reg_alert_radio = (0x65)<<24 | (0x43)<<16 | (0x21)<<8;
	registers->AddRegister<uint32_t>(REG_ALERT_WATER_RADIO);
	registers->SetRegisterAsSingle(REG_ALERT_WATER_RADIO);
	registers->AddRegisterPointer<uint32_t>(REG_ALERT_WATER_RADIO, &reg_alert_radio);


	// === Publisher Setup === //
	// = setpoints publisher
	publishers->AddPublisher(PUB_CPG_SETPOINTS);
	publishers->SetPublisherPrescaler(PUB_CPG_SETPOINTS, 1); //publish at ~100Hz
	publishers->LinkToInterface(PUB_CPG_SETPOINTS, CANFD1);
	publishers->SetPublishAddress(PUB_CPG_SETPOINTS, CANFD1, ALL);
	publishers->AddTopic(PUB_CPG_SETPOINTS, REG_CPG_SETPOINTS);
	publishers->ActivateTopic(PUB_CPG_SETPOINTS, REG_CPG_SETPOINTS);
	publishers->ActivatePublisher(PUB_CPG_SETPOINTS);

	// = Mode publisher: publish the state of the robot (if the remote started it or not)
	publishers->AddPublisher(PUB_REMOTE_MODE);
	publishers->SetPublisherPrescaler(PUB_REMOTE_MODE, 10); //publish at ~10Hz
	publishers->LinkToInterface(PUB_REMOTE_MODE, CANFD1);
	publishers->SetPublishAddress(PUB_REMOTE_MODE, CANFD1, ALL);
	publishers->AddTopic(PUB_REMOTE_MODE, REG_REMOTE_MODE);
	publishers->ActivateTopic(PUB_REMOTE_MODE, REG_REMOTE_MODE);
	publishers->ActivatePublisher(PUB_REMOTE_MODE);

	// = Robot state and CPG parameters publisher
	publishers->AddPublisher(PUB_CPG_PARAM);
	publishers->SetPublisherPrescaler(PUB_CPG_PARAM, 1); //publish when there is a change
	publishers->LinkToInterface(PUB_CPG_PARAM, UART_CM4);
	publishers->SetPublishAddress(PUB_CPG_PARAM, UART_CM4, ALL);
	publishers->AddTopic(PUB_CPG_PARAM, REG_REMOTE_MODE);
	publishers->AddTopic(PUB_CPG_PARAM, REG_CPG_RESET);
	publishers->AddTopic(PUB_CPG_PARAM, REG_CPG_FREQUENCY);
	publishers->AddTopic(PUB_CPG_PARAM, REG_CPG_DIRECTION);
	publishers->AddTopic(PUB_CPG_PARAM, REG_CPG_AMPLC);
	publishers->AddTopic(PUB_CPG_PARAM, REG_CPG_AMPLH);
	publishers->AddTopic(PUB_CPG_PARAM, REG_CPG_NWAVE);
	publishers->AddTopic(PUB_CPG_PARAM, REG_CPG_COUPLING_STRENGTH);
	publishers->AddTopic(PUB_CPG_PARAM, REG_CPG_A_R);
	publishers->ActivatePublisher(PUB_CPG_PARAM);

	// = Publisher to update the CPG enabled state on the CM4
	publishers->AddPublisher(PUB_CPG_ENABLED);
	publishers->SetPublisherPrescaler(PUB_CPG_ENABLED, 1); //publish when there is a change
	publishers->LinkToInterface(PUB_CPG_ENABLED, UART_CM4);
	publishers->SetPublishAddress(PUB_CPG_ENABLED, UART_CM4, ALL);
	publishers->AddTopic(PUB_CPG_ENABLED, REG_CPG_ENABLED);
	publishers->ActivatePublisher(PUB_CPG_ENABLED);

	// = Remote updates
	publishers->AddPublisher(PUB_CPG_FAST);
	publishers->SetPublisherPrescaler(PUB_CPG_FAST, 1); //publish when there is a change
	publishers->LinkToInterface(PUB_CPG_FAST, UART_CM4);
	publishers->SetPublishAddress(PUB_CPG_FAST, UART_CM4, ALL);
	publishers->AddTopic(PUB_CPG_FAST, REG_CPG_DIRECTION);
	publishers->ActivateTopic(PUB_CPG_FAST, REG_CPG_DIRECTION);
	publishers->AddTopic(PUB_CPG_FAST, REG_CPG_AMPLC);
	publishers->ActivateTopic(PUB_CPG_FAST, REG_CPG_AMPLC);
	publishers->AddTopic(PUB_CPG_FAST, REG_CPG_AMPLH);
	publishers->ActivateTopic(PUB_CPG_FAST, REG_CPG_AMPLH);
	publishers->ActivatePublisher(PUB_CPG_FAST);

	// === Subscribers Setup === //
	static SubGeneral sub_general(registers, leds);
	subscribers->AddSubscriber(SUB_GENERAL, &sub_general);
	subscribers->SubscribeToRemoteRegister(SUB_GENERAL, REG_ALERT_WATER, SubscriberInterface{.interface=CANFD1 , .address=ALL});
	subscribers->ActivateSubscriber(SUB_GENERAL);

	// === CPG Setup === //
	cpg.init(MODULE_NUMBER, reg_cpg_frequency, reg_cpg_direction, reg_cpg_amplc, reg_cpg_amplc, reg_cpg_nwave, reg_cpg_coupling_strength, reg_cpg_a_r);
	int8_t setpoints[MODULE_NUMBER];

	//variables used in loop
	uint8_t cpg_enabled = 0;
	uint8_t remote_mode = 0;
	uint8_t remote_mode_last = 0;
	uint16_t len;

	//to have a precise loop time
	uint32_t time_now;
	registers->ReadRegister<uint32_t>(REG_TIMEBASE, &time_now, &len);
	uint32_t time_next = time_now + LOOP_TIME_MS;

	for(;;) {
		//publish the remote mode registers
		publishers->SpinPublisher(PUB_REMOTE_MODE);	//1/10 prescaler

		//robot is started from the remote
		registers->ReadRegister<uint8_t>(REG_REMOTE_MODE, &remote_mode, &len);
		if(remote_mode == 1 && remote_mode_last == 0) {
			leds->SetLED(LED_USER1, GPIO_PIN_SET);
			remote_mode_last = 1;
			cpg.reset();
		}
		else if(remote_mode != 1 && remote_mode_last == 1) {
			leds->SetLED(LED_USER1, GPIO_PIN_RESET);
			remote_mode_last = 0;
			cpg_enabled = 0;
			registers->WriteRegister<uint8_t>(REG_CPG_ENABLED, &cpg_enabled);
		}

		//retrieve timestamp
		registers->ReadRegister<uint32_t>(REG_TIMEBASE, &time_now, &len);

		//compute CPG steps if CPG is enabled
		registers->ReadRegister<uint8_t>(REG_CPG_ENABLED, &cpg_enabled, &len);
		if(cpg_enabled > 0) {
			//disable the CPG if the remote stopped it
			uint32_t remote_last_rx;
			registers->ReadRegister<uint32_t>(REG_REMOTE_LAST_RX, &remote_last_rx, &len);
			if((time_now-remote_last_rx > REMOTE_TIMEOUT_MS) && (cpg_enabled == 1)) {
				cpg_enabled = 0;
				registers->WriteRegister<uint8_t>(REG_CPG_ENABLED, &cpg_enabled);
			}
			else {
				leds->SetLED(LED_USER2, GPIO_PIN_SET);
				//compute 10 cpg steps with 1ms stepsize
				for(uint32_t j=0;j<(LOOP_TIME_MS/CPG_STEPSIZE_MS);j++) {
					cpg.step(setpoints, CPG_STEPSIZE_MS);
				}
				//publish the setpoints
				registers->WriteRegister<int8_t>(REG_CPG_SETPOINTS, setpoints, MODULE_NUMBER);
				publishers->SpinPublisher(PUB_CPG_SETPOINTS);
			}
		}
		else {
			leds->SetLED(LED_USER2, GPIO_PIN_RESET);
		}

		//Sleep to get desired loop time
		registers->ReadRegister<uint32_t>(REG_TIMEBASE, &time_now, &len);
		if(time_now >= time_next) {
			osDelay(1);
		}
		else {
			osDelay(time_next-time_now);
		}
		time_next += LOOP_TIME_MS;
	}
}
