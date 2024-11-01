/*
 * Controller.cpp
 *
 *  Created on: Nov 23, 2022
 *      Author: bignet
 */

#include <Controller/Controller.hpp>

/**
 * @brief class Constructor. Defines the used hardware interfaces
 *
 * @param input timer: pointer to the encoder timer used to read the motor position
 * @param input pulse_: pointer to the pulse register of the timer used to control the motor.
 * @param input EN_: GPIO to activate the H-bridge
 * @param input SHIFTER_: GPIO to activate the voltage shifter that interfaces the STM32 with the H-bridge
 * @param input IN1_: GPIO to control the first column of the H-bridge. Defines the current sign
 * @param input IN2_: GPIO to control the second column of the H-bridge. Defines the current sign
 * @param input ADC_: pointer to the ADC used to read the H-bridge current feedback
 */
Controller::Controller(TIM_TypeDef* timer, uint32_t* pulse_, GPIO EN_, GPIO SHIFTER_, GPIO IN1_, GPIO IN2_, ADC_HandleTypeDef* ADC_){
	encoder = timer;
	pulse = pulse_;
	EN = EN_;
	SHIFTER = SHIFTER_;
	IN1 = IN1_;
	IN2 = IN2_;
	ADC = ADC_;
}

/**
 * @brief Initialize the class with references to other classes. Set default values.
 *
 * @param input registers_: the Registers instance
 * @param input publishers_: the Publishers instance
 * @param input leds_: the LEDS instance
 */
void Controller::Init(Registers* registers_, Publishers* publishers_, LEDS* leds_){
	registers = registers_;
	publishers = publishers_;
	leds = leds_;

	ControlSemaphore = osSemaphoreNew(1,1,NULL);
	osSemaphoreRelease(ControlSemaphore);

	ResetController();

	// Set default values
	SetControllerPeriodMs(DEFAULT_CONTROLLER_PERIOD);
	SelectControllerMode(DEFAULT_CONTROLLER_MODE);
	integration_type = DEFAULT_ERROR_INTEGRATION_MODE;
	derivation_type = DEFAULT_ERROR_DERIVATION_MODE;
	encoder_offset = 0;
	encoder_security_threshold = DEFAULT_ENCODER_SECURITY_THRESHOLD;
	current_conversion = DEFAULT_CURRENT_CONVERSION_FACTOR;
	motor_current_compensation = DEFAULT_CURRENT_COMPENSATION_FACTOR;

	// Setup trajectory generation
	trajectory_generator.SetTrajectoryMode(DEFAULT_MOTOR_TRAJECTORY);
	trajectory_generator.SetTrajectoryCyclicValue(DEFAULT_MOTOR_CYCLIC);
	trajectory_generator.SetTrajectoryMaxSpeed(DEFAULT_MOTOR_MAX_SPEED);
	trajectory_generator.SetTrajectoryMaxAcceleration(DEFAULT_MOTOR_ACCEL_SPEED);
	trajectory_generator.SetTrajectoryFrequency(DEFAULT_MOTOR_FREQUENCY);
	trajectory_generator.SetTrajectoryPhase(DEFAULT_MOTOR_PHASE);

	// Set params values
	params.resize(5);
	params[MOTOR_REDUCTION_RATIO] = DEFAULT_REDUCTION_RATIO; 					// Motor Reduction ratio
	params[MOTOR_ELECTRIC_RESISTOR] = DEFAULT_EQUIVALENT_ELECTRIC_RESISTANCE; 	// Equivalent Motor Resistor
	params[MOTOR_SPEED_CONSTANT] = DEFAULT_EQUIVALENT_SPEED_CONSTANT; 			// Equivalent Motor Speed Constant
	params[MOTOR_TORQUE_CONSTANT_INVERSE] = DEFAULT_TORQUE_CONSTANT_INVERSE; 	// Inverse of Motor Torque Constant
	params[MOTOR_ENCODER_TICK_NUMBER] = DEFAULT_ENCODER_TICK_NUMBER; 			// Motor Encoder Tick Number

	// Position control configuration
	positionConfiguration.track = &encoder_position;
	positionConfiguration.selected_filter = DEFAULT_POSITION_FILTER;
	positionConfiguration.input_filters.resize(2);
	positionConfiguration.input_filters[OUTPUT_MOTOR_POSITION_FILTER] = ([](float input, std::vector<float> parameters) -> float {return input * parameters[MOTOR_REDUCTION_RATIO] * parameters[MOTOR_ENCODER_TICK_NUMBER] / 90;}); // motor position input after reductor in °
	positionConfiguration.input_filters[INPUT_MOTOR_POSITION_FILTER] = ([](float input, std::vector<float> parameters) -> float {return input * parameters[MOTOR_ENCODER_TICK_NUMBER] / 90;}); // motor position input in °
	positionConfiguration.K = DEFAULT_POSITION_K;
	positionConfiguration.Ti_inv = DEFAULT_POSITION_Ti_inv;
	positionConfiguration.Td = DEFAULT_POSITION_Td;
	positionConfiguration.Isaturation = DEFAULT_POSITION_Isat / (positionConfiguration.Ti_inv * positionConfiguration.K);
	positionConfiguration.P_active = DEFAULT_POSITION_P_ACTIVE;
	positionConfiguration.I_active = DEFAULT_POSITION_I_ACTIVE;
	positionConfiguration.D_active = DEFAULT_POSITION_D_ACTIVE;
	positionConfiguration.S_active = DEFAULT_POSITION_S_ACTIVE;
	positionConfiguration.direction_criteria = &PWM;
	positionConfiguration.PWMAssignement = ([](float val) -> uint32_t {return (uint32_t) (abs(val));});
	positionConfiguration.model_active = false;

	// Torque control configuration
	torqueConfiguration.track = &motor_current;
	torqueConfiguration.selected_filter = DEFAULT_TORQUE_FILTER;
	torqueConfiguration.input_filters.resize(3);
	torqueConfiguration.input_filters[CURRENT_FILTER] = ([](float input, std::vector<float> parameters) -> float {return input;}); // motor current input in A
	torqueConfiguration.input_filters[TORQUE_FILTER] = ([](float input, std::vector<float> parameters) -> float {return input * parameters[MOTOR_TORQUE_CONSTANT_INVERSE];}); // motor torque before reductor input in mNm
	torqueConfiguration.input_filters[TORQUE_REDUCTOR_FILTER] = ([](float input, std::vector<float> parameters) -> float {return input * parameters[MOTOR_TORQUE_CONSTANT_INVERSE] / parameters[MOTOR_REDUCTION_RATIO];}); // motor torque after reductor input in mNm
	torqueConfiguration.K = DEFAULT_TORQUE_K;
	torqueConfiguration.Ti_inv = DEFAULT_TORQUE_Ti_inv;
	torqueConfiguration.Td = DEFAULT_TORQUE_Td;
	torqueConfiguration.Isaturation = DEFAULT_TORQUE_Isat / (torqueConfiguration.Ti_inv * torqueConfiguration.K); //24000
	torqueConfiguration.P_active = DEFAULT_TORQUE_P_ACTIVE;
	torqueConfiguration.I_active = DEFAULT_TORQUE_I_ACTIVE;
	torqueConfiguration.D_active = DEFAULT_TORQUE_D_ACTIVE;
	torqueConfiguration.S_active = DEFAULT_TORQUE_S_ACTIVE;
	torqueConfiguration.direction_criteria = &setpoint;
	torqueConfiguration.PWMAssignement = ([](float val) -> uint32_t {if (val > 0) return (uint32_t) (abs(val)); else return 0;});
	torqueConfiguration.model_active = DEFAULT_TORQUE_MODEL_ACTIVE;
	torqueConfiguration.model_variables.resize(2);
	torqueConfiguration.model_variables[0] = &setpoint;
	torqueConfiguration.model_variables[1] = &encoder_speed;
	torqueConfiguration.model = ([](std::vector<float*> variables, std::vector<float> parameters) -> float {return (parameters[MOTOR_ELECTRIC_RESISTOR] * *(variables[0]) + parameters[MOTOR_SPEED_CONSTANT] * *(variables[1]));});

	// PWM control configuration
	PWMConfiguration.track = &PWM;
	PWMConfiguration.P_active = false;
	PWMConfiguration.I_active = false;
	PWMConfiguration.D_active = false;
	PWMConfiguration.S_active = false;
	PWMConfiguration.direction_criteria = &PWM;
	PWMConfiguration.PWMAssignement = ([](float val) -> uint32_t {return (uint32_t) (abs(val));});
	PWMConfiguration.model_active = true;
	torqueConfiguration.model_variables.resize(1);
	PWMConfiguration.model_variables[0] = &setpoint;
	PWMConfiguration.model = ([](std::vector<float*> variables, std::vector<float> parameters) -> float {return (*(variables[0]));});
}

/**
 * @brief Add class related registers
 */
void Controller::AddRegisters(void) {
	// Register to reset the controller
	registers->AddRegister<uint8_t>(REG_CONTROLLER_RESET);
	registers->SetRegisterAsSingle(REG_CONTROLLER_RESET);
	registers->AddWriteCallback<uint8_t>(REG_CONTROLLER_RESET, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Controller* self = (Controller*) context;
			self->ResetController();
			return true;
		}
	);

	// Register to access the H-bridge state
	registers->AddRegister<uint8_t>(REG_BRIDGE_STATE);
	registers->SetRegisterAsSingle(REG_BRIDGE_STATE);
	registers->AddRegisterSemaphore(REG_BRIDGE_STATE, &ControlSemaphore);
	registers->AddReadCallback<uint8_t>(REG_BRIDGE_STATE, (void*) this,
		[](void* context, uint16_t register_id, uint8_t** output, uint16_t* length) -> bool {
			Controller* self = (Controller*) context;
			**output = (uint8_t) self->bridge_active;
			return true;
		}
	);
	registers->AddWriteCallback<uint8_t>(REG_BRIDGE_STATE, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Controller* self = (Controller*) context;
			if (*input > 0)
				self->ActivateBridge();
			else
				self->DeactivateBridge();
			return true;
		}
	);

	// Register to access the controller state
	registers->AddRegister<uint8_t>(REG_CONTROLLER_STATE);
	registers->SetRegisterAsSingle(REG_CONTROLLER_STATE);
	registers->AddReadCallback<uint8_t>(REG_BRIDGE_STATE, (void*) this,
			[](void* context, uint16_t register_id, uint8_t** output, uint16_t* length) -> bool {
			Controller* self = (Controller*) context;
			osSemaphoreAcquire(self->ControlSemaphore, osWaitForever);
			**output = (uint8_t) self->controller_active;
			osSemaphoreRelease(self->ControlSemaphore);
			return true;
		}
	);
	registers->AddWriteCallback<uint8_t>(REG_CONTROLLER_STATE, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Controller* self = (Controller*) context;
			if (*input > 0)
				self->ActivateController();
			else
				self->DeactivateController();
			return true;
		}
	);

	// Register to access the controller period
	registers->AddRegister<uint32_t>(REG_CONTROLLER_PERIOD);
	registers->SetRegisterAsSingle(REG_CONTROLLER_PERIOD);
	registers->AddReadCallback<uint32_t>(REG_CONTROLLER_PERIOD, (void*) this,
		[](void* context, uint16_t register_id, uint32_t** output, uint16_t* length) -> bool {
			Controller* self = (Controller*) context;
			osSemaphoreAcquire(self->ControlSemaphore, osWaitForever);
			**output = self->period_ms;
			osSemaphoreRelease(self->ControlSemaphore);
			return true;
		}
	);
	registers->AddWriteCallback<uint8_t>(REG_CONTROLLER_PERIOD, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Controller* self = (Controller*) context;
			self->SetControllerPeriodMs(*input);
			return true;
		}
	);

	// Register to access the controller mode
	registers->AddRegister<uint8_t>(REG_CONTROLLER_MODE);
	registers->SetRegisterAsSingle(REG_CONTROLLER_MODE);
	registers->AddReadCallback<uint8_t>(REG_CONTROLLER_MODE, (void*) this,
		[](void* context, uint16_t register_id, uint8_t** output, uint16_t* length) -> bool {
			Controller* self = (Controller*) context;
			osSemaphoreAcquire(self->ControlSemaphore, osWaitForever);
			**output = self->mode;
			osSemaphoreRelease(self->ControlSemaphore);
			return true;
		}
	);
	registers->AddWriteCallback<uint8_t>(REG_CONTROLLER_MODE, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Controller* self = (Controller*) context;
			self->SelectControllerMode(*input);
			return true;
		}
	);

	// Register to access the input filter of the current selected mode
	registers->AddRegister<uint8_t>(REG_CONTROLLER_INPUT_FILTER);
	registers->SetRegisterAsSingle(REG_CONTROLLER_INPUT_FILTER);
	registers->AddRegisterSemaphore(REG_CONTROLLER_INPUT_FILTER, &ControlSemaphore);
	registers->AddReadCallback<uint8_t>(REG_CONTROLLER_INPUT_FILTER, (void*) this,
		[](void* context, uint16_t register_id, uint8_t** output, uint16_t* length) -> bool {
			Controller* self = (Controller*) context;
			**output = self->configurations[self->mode]->selected_filter;
			return true;
		}
	);
	registers->AddWriteCallback<uint8_t>(REG_CONTROLLER_INPUT_FILTER, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Controller* self = (Controller*) context;
			self->configurations[self->mode]->selected_filter = *input;
			return true;
		}
	);

	// Register to set a configuration parameter
	registers->AddRegister<float>(REG_CONTROLLER_SET_CONFIGURATION);
	registers->SetRegisterAsArray(REG_CONTROLLER_SET_CONFIGURATION, 2);
	registers->AddRegisterSemaphore(REG_CONTROLLER_SET_CONFIGURATION, &ControlSemaphore);
	registers->AddWriteCallback<float>(REG_CONTROLLER_SET_CONFIGURATION, (void*) this,
		[](void* context, uint16_t register_id, float* input, uint16_t length) -> bool {
			Controller* self = (Controller*) context;
			if (length != 2) return false;
			switch ((uint8_t) input[0]) {
			case CONTROLLER_CONFIGURATION_K:
				self->configurations[self->mode]->K = input[1];
				break;
			case CONTROLLER_CONFIGURATION_Ti:
				self->configurations[self->mode]->Ti_inv = 1 / input[1];
				break;
			case CONTROLLER_CONFIGURATION_Td:
				self->configurations[self->mode]->Td = input[1];
				break;
			case CONTROLLER_CONFIGURATION_ISAT:
				self->configurations[self->mode]->Isaturation = input[1];
				break;
			case CONTROLLER_CONFIGURATION_P_ACTIVE:
				self->configurations[self->mode]->P_active = (bool) input[1];
				break;
			case CONTROLLER_CONFIGURATION_I_ACTIVE:
				self->configurations[self->mode]->I_active = (bool) input[1];
				break;
			case CONTROLLER_CONFIGURATION_D_ACTIVE:
				self->configurations[self->mode]->D_active = (bool) input[1];
				break;
			case CONTROLLER_CONFIGURATION_S_ACTIVE:
				self->configurations[self->mode]->S_active = (bool) input[1];
				break;
			case CONTROLLER_CONFIGURATION_MODEL_ACTIVE:
				self->configurations[self->mode]->model_active = (bool) input[1];
				break;
			default:
				return false;
				break;
			}
			return true;
		}
	);

	// Register to read configuration parameters
	registers->AddRegister<float>(REG_CONTROLLER_GET_CONFIGURATION);
	registers->SetRegisterAsArray(REG_CONTROLLER_GET_CONFIGURATION, 9);
	registers->AddRegisterSemaphore(REG_CONTROLLER_GET_CONFIGURATION, &ControlSemaphore);
	registers->AddReadCallback<float>(REG_CONTROLLER_GET_CONFIGURATION, (void*) this,
		[](void* context, uint16_t register_id, float** output, uint16_t* length) -> bool {
			Controller* self = (Controller*) context;
			*output[CONTROLLER_CONFIGURATION_K] = self->configurations[self->mode]->K;
			*output[CONTROLLER_CONFIGURATION_Ti] = 1 / self->configurations[self->mode]->Ti_inv;
			*output[CONTROLLER_CONFIGURATION_Td] = self->configurations[self->mode]->Td;
			*output[CONTROLLER_CONFIGURATION_ISAT] = self->configurations[self->mode]->Isaturation;
			*output[CONTROLLER_CONFIGURATION_P_ACTIVE] = self->configurations[self->mode]->P_active;
			*output[CONTROLLER_CONFIGURATION_I_ACTIVE] = self->configurations[self->mode]->I_active;
			*output[CONTROLLER_CONFIGURATION_D_ACTIVE] = self->configurations[self->mode]->D_active;
			*output[CONTROLLER_CONFIGURATION_S_ACTIVE] = self->configurations[self->mode]->S_active;
			*output[CONTROLLER_CONFIGURATION_MODEL_ACTIVE] = self->configurations[self->mode]->model_active;
			return true;
		}
	);

	// Register to access the controller integration mode
	registers->AddRegister<uint8_t>(REG_CONTROLLER_INTEGRATION_MODE);
	registers->SetRegisterAsSingle(REG_CONTROLLER_INTEGRATION_MODE);
	registers->AddRegisterSemaphore(REG_CONTROLLER_INTEGRATION_MODE, &ControlSemaphore);
	registers->AddRegisterPointer<uint8_t>(REG_CONTROLLER_INTEGRATION_MODE, &integration_type);

	// Register to access the controller derivation mode
	registers->AddRegister<uint8_t>(REG_CONTROLLER_DERIVATION_MODE);
	registers->SetRegisterAsSingle(REG_CONTROLLER_DERIVATION_MODE);
	registers->AddRegisterSemaphore(REG_CONTROLLER_DERIVATION_MODE, &ControlSemaphore);
	registers->AddRegisterPointer<uint8_t>(REG_CONTROLLER_DERIVATION_MODE, &derivation_type);

	// Register to access the encoder security threshold that stops the controller. A 0 value deactivates the capability
	registers->AddRegister<uint32_t>(REG_MOTOR_ENCODER_THRESHOLD_SECURITY);
	registers->SetRegisterAsSingle(REG_MOTOR_ENCODER_THRESHOLD_SECURITY);
	registers->AddRegisterSemaphore(REG_MOTOR_ENCODER_THRESHOLD_SECURITY, &ControlSemaphore);
	registers->AddRegisterPointer<uint32_t>(REG_MOTOR_ENCODER_THRESHOLD_SECURITY, &encoder_security_threshold);

	// Register to read encoder position
	registers->AddRegister<float>(REG_MOTOR_ENCODER);
	registers->SetRegisterAsSingle(REG_MOTOR_ENCODER);
	registers->AddRegisterSemaphore(REG_MOTOR_ENCODER, &ControlSemaphore);
	registers->AddRegisterPointer<float>(REG_MOTOR_ENCODER, &encoder_position);
	registers->SetRegisterPermissions(REG_MOTOR_ENCODER, READ_PERMISSION);

	// Register to access the current compensation factor to be applied to the H-bridge current feedback
	registers->AddRegister<float>(REG_MOTOR_CURRENT_COMPENSATION);
	registers->SetRegisterAsSingle(REG_MOTOR_CURRENT_COMPENSATION);
	registers->AddRegisterSemaphore(REG_MOTOR_CURRENT_COMPENSATION, &ControlSemaphore);
	registers->AddRegisterPointer<float>(REG_MOTOR_CURRENT_COMPENSATION, &motor_current_compensation);

	// Register to read measured H-bridge current
	registers->AddRegister<float>(REG_MOTOR_HBRIDGE_CURRENT);
	registers->SetRegisterAsSingle(REG_MOTOR_HBRIDGE_CURRENT);
	registers->AddRegisterSemaphore(REG_MOTOR_HBRIDGE_CURRENT, &ControlSemaphore);
	registers->AddRegisterPointer<float>(REG_MOTOR_HBRIDGE_CURRENT, &motor_current);
	registers->SetRegisterPermissions(REG_MOTOR_HBRIDGE_CURRENT, READ_PERMISSION);

	// Register to read measured H-bridge voltage
	registers->AddRegister<float>(REG_MOTOR_HBRIDGE_VOLTAGE);
	registers->SetRegisterAsSingle(REG_MOTOR_HBRIDGE_VOLTAGE);
	registers->AddRegisterSemaphore(REG_MOTOR_HBRIDGE_VOLTAGE, &ControlSemaphore);
	registers->AddRegisterPointer<float>(REG_MOTOR_HBRIDGE_VOLTAGE, &motor_voltage);
	registers->SetRegisterPermissions(REG_MOTOR_HBRIDGE_VOLTAGE, READ_PERMISSION);

	// Register to read the computed motor power
	registers->AddRegister<float>(REG_MOTOR_HBRIDGE_POWER);
	registers->SetRegisterAsSingle(REG_MOTOR_HBRIDGE_POWER);
	registers->AddRegisterSemaphore(REG_MOTOR_HBRIDGE_POWER, &ControlSemaphore);
	registers->AddRegisterPointer<float>(REG_MOTOR_HBRIDGE_POWER, &motor_power);
	registers->SetRegisterPermissions(REG_MOTOR_HBRIDGE_POWER, READ_PERMISSION);

	// Register to read the target setpoint
	registers->AddRegister<float>(REG_MOTOR_SETPOINT);
	registers->SetRegisterAsSingle(REG_MOTOR_SETPOINT);
	registers->AddRegisterSemaphore(REG_MOTOR_SETPOINT, &ControlSemaphore);
	registers->AddRegisterPointer<float>(REG_MOTOR_SETPOINT, &setpoint);
	registers->SetRegisterPermissions(REG_MOTOR_SETPOINT, READ_PERMISSION);

	// Register to start a motion from a start value to a target value. They should be coherent with the selected mode.
	registers->AddRegister<float>(REG_MOTOR_MOVEFROMTO);
	registers->SetRegisterAsArray(REG_MOTOR_MOVEFROMTO, 2);
	registers->AddWriteCallback<float>(REG_MOTOR_MOVEFROMTO, (void*) this,
		[](void* context, uint16_t register_ID, float* input, uint16_t length) -> bool {
			Controller* self = (Controller*) context;
			if (length != 2) return false;
			self->MoveFromTo(input[0], input[1]);
			return true;
		}
	);

	// Register to start a motion from the current controller state to a target value. It should be coherent with the selected mode.
	registers->AddRegister<float>(REG_MOTOR_MOVETO);
	registers->SetRegisterAsSingle(REG_MOTOR_MOVETO);
	registers->AddWriteCallback<float>(REG_MOTOR_MOVETO, (void*) this,
		[](void* context, uint16_t register_ID, float* input, uint16_t length) -> bool {
			Controller* self = (Controller*) context;
			self->MoveTo(*input);
			return true;
		}
	);

	// Register to reset the encoder position
	registers->AddRegister<uint8_t>(REG_MOTOR_SETZERO);
	registers->SetRegisterAsSingle(REG_MOTOR_SETZERO);
	registers->AddWriteCallback<uint8_t>(REG_MOTOR_SETZERO, (void*) this,
		[](void* context, uint16_t register_ID, uint8_t* input, uint16_t length) -> bool {
			Controller* self = (Controller*) context;
			self->SetEncoderZero();
			return true;
		}
	);

	// Register to set a controller element in params
	registers->AddRegister<float>(REG_MOTOR_SET_PARAMETER);
	registers->SetRegisterAsArray(REG_MOTOR_SET_PARAMETER, 2);
	registers->AddRegisterSemaphore(REG_MOTOR_SET_PARAMETER, &ControlSemaphore);
	registers->AddWriteCallback<float>(REG_MOTOR_SET_PARAMETER, (void*) this,
		[](void* context, uint16_t register_id, float* input, uint16_t length) -> bool {
			Controller* self = (Controller*) context;
			if (length != 2) return false;
			if ((uint8_t) input[0] >= self->params.size()) return false;

			self->params[(uint8_t) input[0]] = input[1];
			return true;
		}
	);

	// Register to read all params from the controller
	registers->AddRegister<float>(REG_MOTOR_GET_PARAMETERS);
	registers->SetRegisterAsVector(REG_MOTOR_GET_PARAMETERS);
	registers->AddRegisterSemaphore(REG_MOTOR_GET_PARAMETERS, &ControlSemaphore);
	registers->AddVectorRegisterPointer<float>(REG_MOTOR_GET_PARAMETERS, &params);
	registers->SetRegisterPermissions(REG_MOTOR_GET_PARAMETERS, READ_PERMISSION);

	// Register to access the trajectory mode
	registers->AddRegister<uint8_t>(REG_MOTOR_TRAJECTORY_MODE);
	registers->SetRegisterAsSingle(REG_MOTOR_TRAJECTORY_MODE);
	registers->AddRegisterSemaphore(REG_MOTOR_TRAJECTORY_MODE, &ControlSemaphore);
	registers->AddRegisterPointer<uint8_t>(REG_MOTOR_TRAJECTORY_MODE, &(trajectory_generator.mode));

	// Register to access the trajectory cyclic value
	registers->AddRegister<uint8_t>(REG_MOTOR_TRAJECTORY_CYCLIC);
	registers->SetRegisterAsSingle(REG_MOTOR_TRAJECTORY_CYCLIC);
	registers->AddRegisterSemaphore(REG_MOTOR_TRAJECTORY_CYCLIC, &ControlSemaphore);
	registers->AddRegisterPointer<uint8_t>(REG_MOTOR_TRAJECTORY_CYCLIC, &(trajectory_generator.cyclic));

	// Register to plan a trajectory via the trajectory generator
	registers->AddRegister<float>(REG_MOTOR_PLAN_TRAJECTORY);
	registers->SetRegisterAsArray(REG_MOTOR_PLAN_TRAJECTORY, 2);
	registers->AddRegisterSemaphore(REG_MOTOR_PLAN_TRAJECTORY, &ControlSemaphore);
	registers->AddWriteCallback<float>(REG_MOTOR_PLAN_TRAJECTORY, (void*) this,
		[](void* context, uint16_t register_id, float* input, uint16_t length) -> bool {
			Controller* self = (Controller*) context;
			if (length != 2) return false;
			self->trajectory_generator.PlanTrajectory(input[0], input[1]);
			return true;
		}
	);

	// Register to start a planned trajectory via the trajectory generator
	registers->AddRegister<uint8_t>(REG_MOTOR_START_TRAJECTORY);
	registers->SetRegisterAsSingle(REG_MOTOR_START_TRAJECTORY);
	registers->AddRegisterSemaphore(REG_MOTOR_START_TRAJECTORY, &ControlSemaphore);
	registers->AddWriteCallback<float>(REG_MOTOR_START_TRAJECTORY, (void*) this,
		[](void* context, uint16_t register_id, float* input, uint16_t length) -> bool {
			Controller* self = (Controller*) context;
			self->trajectory_generator.StartTrajectory();
			return true;
		}
	);

	// Register to access the maximum speed tolerated by the trajectory generator
	registers->AddRegister<float>(REG_MOTOR_MAX_SPEED);
	registers->SetRegisterAsSingle(REG_MOTOR_MAX_SPEED);
	registers->AddRegisterSemaphore(REG_MOTOR_MAX_SPEED, &ControlSemaphore);
	registers->AddRegisterPointer<float>(REG_MOTOR_MAX_SPEED, &(trajectory_generator.speed_max));

	// Register to access the maximum acceleration tolerated by the trajectory generator
	registers->AddRegister<float>(REG_MOTOR_MAX_ACCELERATION);
	registers->SetRegisterAsSingle(REG_MOTOR_MAX_ACCELERATION);
	registers->AddRegisterSemaphore(REG_MOTOR_MAX_ACCELERATION, &ControlSemaphore);
	registers->AddRegisterPointer<float>(REG_MOTOR_MAX_ACCELERATION, &(trajectory_generator.accel_max));

	// Register to access the sinus frequency saved by the trajectory generator
	registers->AddRegister<float>(REG_MOTOR_SIN_FREQUENCY);
	registers->SetRegisterAsSingle(REG_MOTOR_SIN_FREQUENCY);
	registers->AddRegisterSemaphore(REG_MOTOR_SIN_FREQUENCY, &ControlSemaphore);
	registers->AddRegisterPointer<float>(REG_MOTOR_SIN_FREQUENCY, &(trajectory_generator.frequency));

	// Register to access the sinus phase saved by the trajectory generator
	registers->AddRegister<float>(REG_MOTOR_SIN_PHASE);
	registers->SetRegisterAsSingle(REG_MOTOR_SIN_PHASE);
	registers->AddRegisterSemaphore(REG_MOTOR_SIN_PHASE, &ControlSemaphore);
	registers->AddRegisterPointer<float>(REG_MOTOR_SIN_PHASE, &(trajectory_generator.phase));
}

/**
 * @brief Setup the controller publisher
 *
 * @param input interface_ID: the interface ID to publish on
 */
void Controller::SetupPublisher(uint8_t interface_ID) {
	// Add publisher
	publishers->AddPublisher(PUBLISHER_CONTROLLER);

	// Add an interface to the publisher
	publishers->LinkToInterface(PUBLISHER_CONTROLLER, interface_ID);

	// Setup publisher params
	publishers->SetPublisherPrescaler(PUBLISHER_CONTROLLER, PUBLISHER_CONTROLLER_PRESCALER);
	publishers->SetPublishAddress(PUBLISHER_CONTROLLER, interface_ID, PUBLISHER_CONTROLLER_ADDRESS);

	// Add topics
	publishers->AddTopic(PUBLISHER_CONTROLLER, REG_TIMEBASE);
	publishers->AddTopic(PUBLISHER_CONTROLLER, REG_MOTOR_ENCODER);
	publishers->AddTopic(PUBLISHER_CONTROLLER, REG_MOTOR_HBRIDGE_VOLTAGE);
	publishers->AddTopic(PUBLISHER_CONTROLLER, REG_MOTOR_HBRIDGE_CURRENT);
	publishers->AddTopic(PUBLISHER_CONTROLLER, REG_MOTOR_HBRIDGE_POWER);
	publishers->AddTopic(PUBLISHER_CONTROLLER, REG_MOTOR_SETPOINT);

	// Activate topics
#ifdef PUBLISH_CONTROLLER_TIMEBASE
	publishers->ActivateTopic(PUBLISHER_CONTROLLER, REG_TIMEBASE);
#endif
#ifdef PUBLISH_CONTROLLER_ENCODER
	publishers->ActivateTopic(PUBLISHER_CONTROLLER, REG_MOTOR_ENCODER);
#endif
#ifdef PUBLISH_CONTROLLER_VOLTAGE
	publishers->ActivateTopic(PUBLISHER_CONTROLLER, REG_MOTOR_HBRIDGE_VOLTAGE);
#endif
#ifdef PUBLISH_CONTROLLER_CURRENT
	publishers->ActivateTopic(PUBLISHER_CONTROLLER, REG_MOTOR_HBRIDGE_CURRENT);
#endif
#ifdef PUBLISH_CONTROLLER_POWER
	publishers->ActivateTopic(PUBLISHER_CONTROLLER, REG_MOTOR_HBRIDGE_POWER);
#endif
#ifdef PUBLISH_CONTROLLER_SETPOINT
	publishers->ActivateTopic(PUBLISHER_CONTROLLER, REG_MOTOR_SETPOINT);
#endif

#ifdef PUBLISHER_CONTROLLER_ACTIVE
	// Activate publisher
	publishers->ActivatePublisher(PUBLISHER_CONTROLLER);
#endif
}

/**
 * @brief Activate the H-bridge and voltage shifter
 */
void Controller::ActivateBridge(void) {
	bridge_active = true;
	HAL_GPIO_WritePin(EN.port, EN.pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SHIFTER.port, SHIFTER.pin, GPIO_PIN_SET);
}

/**
 * @brief Deactivate the H-bridge and voltage shifter
 */
void Controller::DeactivateBridge(void) {
	bridge_active = false;
	HAL_GPIO_WritePin(EN.port, EN.pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SHIFTER.port, SHIFTER.pin, GPIO_PIN_RESET);
}

/**
 * @brief Activate the controller. Blinks the controller LED
 */
void Controller::ActivateController(void) {
	osSemaphoreAcquire(ControlSemaphore, osWaitForever);
#if defined(USE_LEDS) && defined(USE_SMD_LEDs)
	leds->Blink(LED_CONTROLLER);
#endif

	controller_active = true;
	osSemaphoreRelease(ControlSemaphore);
}

/**
 * @brief Deactivate the controller. Stop the controller LED
 */
void Controller::DeactivateController(void) {
	osSemaphoreAcquire(ControlSemaphore, osWaitForever);
#if defined(USE_LEDS) && defined(USE_SMD_LEDs)
	leds->ClearLED(LED_CONTROLLER);
#endif

	controller_active = false;
	osSemaphoreRelease(ControlSemaphore);
}

/**
 * @brief Reset controller and reset controller variables
 */
void Controller::ResetController(void) {
	osSemaphoreAcquire(ControlSemaphore, osWaitForever);
	PWM = 0;
	error_history[0] = 0;
	error_history[1] = 0;
	error_history[2] = 0;
	error_integral = 0;
	osSemaphoreRelease(ControlSemaphore);
}

/**
 * @brief Set the controller period
 *
 * @param input period: new controller period
 */
void Controller::SetControllerPeriodMs(uint32_t period) {
	osSemaphoreAcquire(ControlSemaphore, osWaitForever);
	period_ms = period; // ms
	period_s = period_ms / 1000.0f;
	period_s_half = period_s / 2.0f;
	period_s_inv = 1.0f / period_s;
	period_s_double_inv = period_s_inv / 2.0f;
	osSemaphoreRelease(ControlSemaphore);
}

/**
 * @brief Set the controller mode. Setup the controller LED.
 *
 * @param input mode_: new controller mode
 */
void Controller::SelectControllerMode(uint8_t mode_) {
	osSemaphoreAcquire(ControlSemaphore, osWaitForever);
	mode = mode_;

	switch (mode_) {
	case POSITION_MODE:
		leds->ConfigureTimings(LED_CONTROLLER, std::vector<uint8_t>{1}, 10);
		break;
	case TORQUE_MODE:
		leds->ConfigureTimings(LED_CONTROLLER, std::vector<uint8_t>{1,2,3}, 10);
		break;
	case PWM_MODE:
		leds->ConfigureTimings(LED_CONTROLLER, std::vector<uint8_t>{1,2,3,4,5}, 10);
		break;
	default:
		leds->ConfigureTimings(LED_CONTROLLER, std::vector<uint8_t>{5}, 10);
		break;
	}

	osSemaphoreRelease(ControlSemaphore);
}

/**
 * @brief Select the filter of the given configuration mode
 *
 * @param input mode: a controller configuration mode
 * @param input filter: filter ID to select for the selected mode
 */
void Controller::SelectInputFilter(uint8_t mode, uint8_t filter) {
	osSemaphoreAcquire(ControlSemaphore, osWaitForever);
	configurations[mode]->selected_filter = filter;
	osSemaphoreRelease(ControlSemaphore);
}

/**
 * @brief Set the controller integration mode
 *
 * @param input mode: new controller integration mode
 */
void Controller::SetIntegrationMode(uint8_t mode) {
	osSemaphoreAcquire(ControlSemaphore, osWaitForever);
	integration_type = mode;
	osSemaphoreRelease(ControlSemaphore);
}

/**
 * @brief Set the controller derivation mode
 *
 * @param input mode: new controller derivation mode
 */
void Controller::SetDerivationMode(uint8_t mode) {
	osSemaphoreAcquire(ControlSemaphore, osWaitForever);
	derivation_type = mode;
	osSemaphoreRelease(ControlSemaphore);
}

/**
 * @brief Start a motion from start to target
 *
 * @param input start: start value with respected to the selected mode and filter
 * @param input target: target value with respected to the selected mode and filter
 */
void Controller::MoveFromTo(float start, float target) {
	osSemaphoreAcquire(ControlSemaphore, osWaitForever);
	float filtered_start = start;
	float filtered_target = target;

	// Translate values via filter
	ControllerConfiguration* configuration = configurations[mode];
	uint8_t selected_filter = configuration->selected_filter;
	if (configuration->input_filters.size() > 0 && selected_filter < configuration->input_filters.size()) {
		filtered_start = configuration->input_filters[selected_filter](start, params);
		filtered_target = configuration->input_filters[selected_filter](target, params);
	}

	// Start trajectory
	trajectory_generator.StartTrajectory(filtered_start, filtered_target);
	osSemaphoreRelease(ControlSemaphore);
}

/**
 * @brief Start a motion from the current controller state to target
 *
 * @param input target: target value with respected to the selected mode and filter
 */
void Controller::MoveTo(float target) {
	osSemaphoreAcquire(ControlSemaphore, osWaitForever);
	float filtered_target = target;

	// Translate values via filter
	ControllerConfiguration* configuration = configurations[mode];
	uint8_t selected_filter = configuration->selected_filter;
	if (configuration->input_filters.size() > 0 && selected_filter < configuration->input_filters.size()) {
		filtered_target = configuration->input_filters[selected_filter](target, params);
	}

	// Start trajectory
	trajectory_generator.StartTrajectory(*(configuration->track), filtered_target);
	osSemaphoreRelease(ControlSemaphore);
}

/**
 * @brief Function called repeatedly to run the controller
 */
void Controller::Control(void){
	osSemaphoreAcquire(ControlSemaphore, osWaitForever);

	// Get motor position
	encoder_position = ((int32_t) (encoder->CNT)) - encoder_offset;

	// Check encoder safety guard
	if (encoder_security_threshold > 0 && (uint32_t) abs((int32_t) encoder_position) > encoder_security_threshold) {
		controller_active = false;
		DeactivateBridge();
		ControllerFaultLEDS(leds);
	}

	// Get motor current from H-bridge
	MeasureCurrent();

	// Get motor instantaneous power
	uint16_t length;
	bool success = registers->ReadRegister<float>(REG_MOTOR_VOLTAGE, &motor_voltage, &length);
	if (success)
		motor_power = motor_current * motor_voltage;

	if (controller_active) {
		encoder_history[2] = encoder_history[1];
		encoder_history[1] = encoder_history[0];
		encoder_history[0] = encoder_position;
		encoder_speed = Derivation(encoder_history, DEFAULT_SPEED_DERIVATION_MODE);

		// Trajectory Generation
		setpoint = trajectory_generator.GenerateSetPoint();

		// Control
		float error = setpoint - *((*configurations[mode]).track);

		error_history[2] = error_history[1];
		error_history[1] = error_history[0];
		error_history[0] = error;

		PWM = 0;
		if ((*configurations[mode]).P_active) 		PWM += error;
		if ((*configurations[mode]).I_active) 		PWM += (*configurations[mode]).Ti_inv * Integration(&error_integral, error_history, (*configurations[mode]).S_active, (*configurations[mode]).Isaturation, integration_type);
		if ((*configurations[mode]).D_active) 		PWM += (*configurations[mode]).Td * Derivation(error_history, derivation_type);
		PWM = (*configurations[mode]).K * PWM;

		// Additional model if any
		if ((*configurations[mode]).model_active) 	PWM += (*configurations[mode]).model((*configurations[mode]).model_variables, params);

		HAL_GPIO_WritePin(IN1.port, IN1.pin, (GPIO_PinState) (*((*configurations[mode]).direction_criteria) > 0));
		HAL_GPIO_WritePin(IN2.port, IN2.pin, (GPIO_PinState) (*((*configurations[mode]).direction_criteria) < 0));
		*pulse = (*configurations[mode]).PWMAssignement(PWM);
	}

	osSemaphoreRelease(ControlSemaphore);

	publishers->SpinPublisher(PUBLISHER_CONTROLLER);

	osDelay(period_ms);
}

/**
 * @brief Reset the encoder value
 */
void Controller::SetEncoderZero(void) {
	osSemaphoreAcquire(ControlSemaphore, osWaitForever);
	encoder_offset = (int32_t) (encoder->CNT);
	osSemaphoreRelease(ControlSemaphore);
}

/**
 * @brief Derive the input value based on the selected derivation mode
 *
 * @param input values: the value to derive with previous numbers in an array
 * @param input derivation_mode: mode that defines the derivation computation
 * @return the derivated value
 */
float Controller::Derivation(float values[], uint8_t derivation_mode){
	if (derivation_mode == DER_EULER_1T) {
		return (values[0] - values[1]) * period_s_inv;
	} else if (derivation_mode == DER_EULER_2T) {
		return (values[0] - values[2]) * period_s_double_inv;
	}
	return 0;
}

/**
 * @brief Integrate the input value based on the selected integration mode. Accumulates the input integral
 *
 * @param input/output integral: the current integral value
 * @param input values: the value to integrate with previous numbers in an array
 * @param input saturation_active: whether the integration is saturated as of a threshold
 * @param input saturation: saturation threshold to consider if saturation_active true
 * @param input integration_mode: mode that defines the integration computation
 * @return the new integral value (also accessible via the integral param)
 */
float Controller::Integration(float* integral, float values[], bool saturation_active, float saturation, uint8_t integration_mode){
	if (saturation_active)
		if (((values[0] > 0) && (*integral > saturation)) || ((values[0] < 0) && (*integral < -saturation)))
			return *integral;

	if (integration_mode == INT_RECT) {
		*integral += values[0] * period_s;
	} else if (integration_mode == INT_TRAP) {
		*integral += (values[0] + values[1]) * period_s_half;
	}
	return *integral;
}

/**
 * @brief Measure H-bridge current
*/
void Controller::MeasureCurrent(void) {
	HAL_ADC_Start(ADC);
	if (HAL_ADC_PollForConversion(ADC, 1) == HAL_OK) {
		motor_current = HAL_ADC_GetValue(ADC) * current_conversion * motor_current_compensation;
		if (HAL_GPIO_ReadPin(IN2.port, IN2.pin) == GPIO_PIN_SET)
			motor_current = -motor_current;
	}
	HAL_ADC_Stop(ADC);
}

/**
 * @brief Get the integration mode
 *
 * @return the integration mode
*/
uint8_t Controller::GetIntegrationMode(void) {
	osSemaphoreAcquire(ControlSemaphore, osWaitForever);
	uint8_t mode = integration_type;
	osSemaphoreRelease(ControlSemaphore);
	return mode;
}

/**
 * @brief Get the derivation mode
 *
 * @return the derivation mode
*/
uint8_t Controller::GetDerivationMode(void) {
	osSemaphoreAcquire(ControlSemaphore, osWaitForever);
	uint8_t mode = derivation_type;
	osSemaphoreRelease(ControlSemaphore);
	return mode;
}

/**
 * @brief Get the encoder value
 *
 * @return the encoder value
*/
int32_t Controller::GetEncoder(void) {
	return (int32_t) encoder_position;
}

/**
 * @brief Get the current
 *
 * @return the current
*/
float Controller::GetCurrent(void) {
	return motor_current;
}

/**
 * @brief Get the setpoint
 *
 * @return the setpoint
*/
int32_t Controller::GetSetPoint(void) {
	return setpoint;
}
