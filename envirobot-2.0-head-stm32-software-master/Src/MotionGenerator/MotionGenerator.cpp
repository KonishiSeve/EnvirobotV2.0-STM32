/*
 * MotionGenerator.cpp
 *
 *  Created on: 2 févr. 2023
 *      Author: bignet
 */

#include <MotionGenerator/MotionGenerator.hpp>

/**
 * Class constructor
 */
MotionGenerator::MotionGenerator() {

}

/**
 * @brief Initialize the class with references to other classes. Set default values.
 *
 * @param input registers_: the Registers instance
 * @param input publishers_: the Publishers instance
 * @param input services_: the Services instance
 * @param input leds_: the LEDS instance
 */
void MotionGenerator::Init(Registers* registers_, Publishers* publishers_, Services* services_, LEDS* leds_) {
	registers = registers_;
	publishers = publishers_;
	services = services_;
	leds = leds_;

	GeneratorSemaphore = osSemaphoreNew(1,1,NULL);
	osSemaphoreRelease(GeneratorSemaphore);

	// Define parameters
	MoveToRegister = Register{.address=REG_MOTOR_MOVETO, .type=FLOAT_TYPE, .isArray=false, .length=1};
	GeneratorInterface = ServiceInterface{.interface=GENERATOR_INTERFACE};

	// Set default values
	SetGeneratorPeriod(DEFAULT_GENERATOR_PERIOD);
	SetNumberOfModules(DEFAULT_GENERATOR_NB_NODULES);
	ResetGeneratorOffsets();
	SetModuleLength(DEFAULT_MODULE_LENGTH);
	SetGeneratorAmplitude(DEFAULT_GENERATOR_AMPLITUDE);
	SetGeneratorFrequency(DEFAULT_GENERATOR_FREQUENCY);
	SetGeneratorWavelengthInverse(1 / DEFAULT_GENERATOR_WAVELENGTH);
	SetGeneratorPhase(DEFAULT_GENERATOR_PHASE);

	request_module_configuration = false;
	request_module_controller_activation = false;
	request_module_controller_deactivation = false;
}

/**
 * @brief Add class related registers
 */
void MotionGenerator::AddRegisters(void) {
	// Register to configure modules
	registers->AddRegister<uint8_t>(REG_GEN_CONFIGURE_MODULES);
	registers->SetRegisterAsSingle(REG_GEN_CONFIGURE_MODULES);
	registers->AddRegisterSemaphore(REG_GEN_GENERATOR_STATUS, &GeneratorSemaphore);
	registers->AddWriteCallback<uint8_t>(REG_GEN_CONFIGURE_MODULES, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			MotionGenerator* self = (MotionGenerator*) context;
			self->request_module_configuration = true;
			return true;
		}
	);

	// Register to set module controller status
	registers->AddRegister<uint8_t>(REG_GEN_MODULES_CONTROLLER_STATUS);
	registers->SetRegisterAsSingle(REG_GEN_MODULES_CONTROLLER_STATUS);
	registers->AddRegisterSemaphore(REG_GEN_GENERATOR_STATUS, &GeneratorSemaphore);
	registers->AddWriteCallback<uint8_t>(REG_GEN_MODULES_CONTROLLER_STATUS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			MotionGenerator* self = (MotionGenerator*) context;
			if (*input > 0) {
				self->request_module_controller_activation = true;
			} else {
				self->request_module_controller_deactivation = true;
			}
			return true;
		}
	);

	// Register to access generator status
	registers->AddRegister<uint8_t>(REG_GEN_GENERATOR_STATUS);
	registers->SetRegisterAsSingle(REG_GEN_GENERATOR_STATUS);
	registers->AddRegisterSemaphore(REG_GEN_GENERATOR_STATUS, &GeneratorSemaphore);
	registers->AddReadCallback<uint8_t>(REG_GEN_GENERATOR_STATUS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t** output, uint16_t* length) -> bool {
			MotionGenerator* self = (MotionGenerator*) context;
			osSemaphoreAcquire(self->GeneratorSemaphore, osWaitForever);
			*length = 1;
			**output = self->active;
			osSemaphoreRelease(self->GeneratorSemaphore);
			return true;
		}
	);
	registers->AddWriteCallback<uint8_t>(REG_GEN_GENERATOR_STATUS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			MotionGenerator* self = (MotionGenerator*) context;
			if (*input > 0)
				self->ActivateGenerator();
			else
				self->DeactivateGenerator();
			return true;
		}
	);

	// Register to get setpoints
	registers->AddRegister<float>(REG_GEN_SETPOINTS);
	registers->SetRegisterAsVector(REG_GEN_SETPOINTS);
	registers->AddRegisterSemaphore(REG_GEN_SETPOINTS, &GeneratorSemaphore);
	registers->AddVectorRegisterPointer<float>(REG_GEN_SETPOINTS, &setpoints);
	registers->SetRegisterPermissions(REG_GEN_SETPOINTS, READ_PERMISSION);

	// Register to access generator period
	registers->AddRegister<uint32_t>(REG_GEN_PERIOD);
	registers->SetRegisterAsSingle(REG_GEN_PERIOD);
	registers->AddRegisterSemaphore(REG_GEN_PERIOD, &GeneratorSemaphore);
	registers->AddRegisterPointer<uint32_t>(REG_GEN_PERIOD, &period_ms);

	// Register to access the number of modules
	registers->AddRegister<uint8_t>(REG_GEN_NB_MODULES);
	registers->SetRegisterAsSingle(REG_GEN_NB_MODULES);
	registers->AddReadCallback<uint8_t>(REG_GEN_NB_MODULES, (void*) this,
		[](void* context, uint16_t register_id, uint8_t** output, uint16_t* length) -> bool {
			MotionGenerator* self = (MotionGenerator*) context;
			osSemaphoreAcquire(self->GeneratorSemaphore, osWaitForever);
			*length = 1;
			**output = self->number_of_modules;
			osSemaphoreRelease(self->GeneratorSemaphore);
			return true;
		}
	);
	registers->AddWriteCallback<uint8_t>(REG_GEN_NB_MODULES, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			MotionGenerator* self = (MotionGenerator*) context;
			return self->SetNumberOfModules(*input);
		}
	);

	// Register to set the generator offset for a specific module
	registers->AddRegister<float>(REG_GEN_GENERATOR_OFFSET);
	registers->SetRegisterAsArray(REG_GEN_GENERATOR_OFFSET, 2);
	registers->AddWriteCallback<float>(REG_GEN_GENERATOR_OFFSET, (void*) this,
		[](void* context, uint16_t register_id, float* input, uint16_t length) -> bool {
			MotionGenerator* self = (MotionGenerator*) context;
			if (length != 2) return false;
			return self->SetGeneratorOffset((uint8_t) input[0], input[1]);
		}
	);

	// Register to reset all generator offsets to 0
	registers->AddRegister<uint8_t>(REG_GEN_RESET_GENERATOR_OFFSETS);
	registers->SetRegisterAsSingle(REG_GEN_RESET_GENERATOR_OFFSETS);
	registers->AddWriteCallback<uint8_t>(REG_GEN_RESET_GENERATOR_OFFSETS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			MotionGenerator* self = (MotionGenerator*) context;
			std::fill(self->generator_offsets.begin(), self->generator_offsets.end(), 0);
			return true;
		}
	);

	// Register to set the position offset for a specific module
	registers->AddRegister<float>(REG_GEN_POSITION_OFFSET);
	registers->SetRegisterAsArray(REG_GEN_POSITION_OFFSET, 2);
	registers->AddWriteCallback<float>(REG_GEN_POSITION_OFFSET, (void*) this,
		[](void* context, uint16_t register_id, float* input, uint16_t length) -> bool {
			MotionGenerator* self = (MotionGenerator*) context;
			if (length != 2) return false;
			return self->SetPositionOffset((uint8_t) input[0], input[1]);
		}
	);

	// Register to reset all position offsets
	registers->AddRegister<uint8_t>(REG_GEN_RESET_POSITION_OFFSETS);
	registers->SetRegisterAsSingle(REG_GEN_RESET_POSITION_OFFSETS);
	registers->AddWriteCallback<uint8_t>(REG_GEN_RESET_POSITION_OFFSETS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			MotionGenerator* self = (MotionGenerator*) context;
			std::fill(self->position_offsets.begin(), self->position_offsets.end(), 0);
			return true;
		}
	);

	// Register to access the module length
	registers->AddRegister<float>(REG_GEN_MODULE_LENGTH);
	registers->SetRegisterAsSingle(REG_GEN_MODULE_LENGTH);
	registers->AddRegisterSemaphore(REG_GEN_MODULE_LENGTH, &GeneratorSemaphore);
	registers->AddRegisterPointer<float>(REG_GEN_MODULE_LENGTH, &module_length);

	// Register to access the waveform amplitude
	registers->AddRegister<float>(REG_GEN_AMPLITUDE);
	registers->SetRegisterAsSingle(REG_GEN_AMPLITUDE);
	registers->AddRegisterSemaphore(REG_GEN_AMPLITUDE, &GeneratorSemaphore);
	registers->AddRegisterPointer<float>(REG_GEN_AMPLITUDE, &amplitude);

	// Register to access the waveform frequency
	registers->AddRegister<float>(REG_GEN_FREQUENCY);
	registers->SetRegisterAsSingle(REG_GEN_FREQUENCY);
	registers->AddRegisterSemaphore(REG_GEN_FREQUENCY, &GeneratorSemaphore);
	registers->AddRegisterPointer<float>(REG_GEN_FREQUENCY, &frequency);

	// Register to access the wavelength inverse
	registers->AddRegister<float>(REG_GEN_WAVELENGTH_INVERSE);
	registers->SetRegisterAsSingle(REG_GEN_WAVELENGTH_INVERSE);
	registers->AddRegisterSemaphore(REG_GEN_WAVELENGTH_INVERSE, &GeneratorSemaphore);
	registers->AddRegisterPointer<float>(REG_GEN_WAVELENGTH_INVERSE, &wavelength_inverse);

	// Register to access the waveform phase
	registers->AddRegister<float>(REG_GEN_PHASE);
	registers->SetRegisterAsSingle(REG_GEN_PHASE);
	registers->AddRegisterSemaphore(REG_GEN_PHASE, &GeneratorSemaphore);
	registers->AddRegisterPointer<float>(REG_GEN_PHASE, &phase);
}

/**
 * @brief Setup the generator publisher
 *
 * @param input interface_ID: the interface ID to publish on
 */
void MotionGenerator::SetupPublisher(void) {
	// Add publisher
	publishers->AddPublisher(PUBLISHER_MOTION_GENERATOR);

	// Add an interface to the publisher
	publishers->LinkToInterface(PUBLISHER_MOTION_GENERATOR, PUBLISHER_MOTION_GENERATOR_INTERFACE);

	// Setup publisher params
	publishers->SetPublisherPrescaler(PUBLISHER_MOTION_GENERATOR, PUBLISHER_MOTION_GENERATOR_PRESCALER);
	publishers->SetPublishAddress(PUBLISHER_MOTION_GENERATOR, PUBLISHER_MOTION_GENERATOR_INTERFACE, PUBLISHER_MOTION_GENERATOR_ADDRESS);

	// Add topics
	publishers->AddTopic(PUBLISHER_MOTION_GENERATOR, REG_TIMEBASE);
	publishers->AddTopic(PUBLISHER_MOTION_GENERATOR, REG_GEN_SETPOINTS);

	// Activate topics
#ifdef PUBLISH_MOTION_GENERATOR_TIMEBASE
	publishers->ActivateTopic(PUBLISHER_MOTION_GENERATOR, REG_TIMEBASE);
#endif
#ifdef PUBLISH_MOTION_GENERATOR_SETPOINTS
	publishers->ActivateTopic(PUBLISHER_MOTION_GENERATOR, REG_GEN_SETPOINTS);
#endif

#ifdef PUBLISHER_MOTION_GENERATOR_ACTIVE
	// Activate publisher
	publishers->ActivatePublisher(PUBLISHER_MOTION_GENERATOR);
#endif
}

/**
 * @brief Configure module controllers (reset controller, position control, output position filter, step trajectory, not cyclic)
 */
bool MotionGenerator::ConfigureModules(void) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	uint8_t status;
	for (uint8_t index = 0; index < number_of_modules; index++) {
		GeneratorInterface.address = STM32_HEAD + 1 + index;

		// Reset controller
		if (GENERATOR_CONFIGURATON_DELAY > 0) osDelay(GENERATOR_CONFIGURATON_DELAY);
		uint8_t reset = (uint8_t) true;
		status = services->WriteRemoteRegister<uint8_t>(Register{.address=REG_CONTROLLER_RESET, .type=UINT8_TYPE, .isArray=false, .length=1}, GeneratorInterface, &reset);
		if (status != OK) {osSemaphoreRelease(GeneratorSemaphore); return false;}

		// Set in position control
		if (GENERATOR_CONFIGURATON_DELAY > 0) osDelay(GENERATOR_CONFIGURATON_DELAY);
		uint8_t mode = POSITION_MODE;
		status = services->WriteRemoteRegister<uint8_t>(Register{.address=REG_CONTROLLER_MODE, .type=UINT8_TYPE, .isArray=false, .length=1}, GeneratorInterface, &mode);
		if (status != OK) {osSemaphoreRelease(GeneratorSemaphore); return false;}

		// Select the output motor position filter
		if (GENERATOR_CONFIGURATON_DELAY > 0) osDelay(GENERATOR_CONFIGURATON_DELAY);
		uint8_t filter = OUTPUT_MOTOR_POSITION_FILTER;
		status = services->WriteRemoteRegister<uint8_t>(Register{.address=REG_CONTROLLER_INPUT_FILTER, .type=UINT8_TYPE, .isArray=false, .length=1}, GeneratorInterface, &filter);
		if (status != OK) {osSemaphoreRelease(GeneratorSemaphore); return false;}

		// Select a step trajectory
		if (GENERATOR_CONFIGURATON_DELAY > 0) osDelay(GENERATOR_CONFIGURATON_DELAY);
		uint8_t trajectory = TRAJECTORY_STEP;
		status = services->WriteRemoteRegister<uint8_t>(Register{.address=REG_MOTOR_TRAJECTORY_MODE, .type=UINT8_TYPE, .isArray=false, .length=1}, GeneratorInterface, &trajectory);
		if (status != OK) {osSemaphoreRelease(GeneratorSemaphore); return false;}

		// Set trajectory as not cyclic
		if (GENERATOR_CONFIGURATON_DELAY > 0) osDelay(GENERATOR_CONFIGURATON_DELAY);
		uint8_t cyclic = (uint8_t) false;
		status = services->WriteRemoteRegister<uint8_t>(Register{.address=REG_MOTOR_TRAJECTORY_CYCLIC, .type=UINT8_TYPE, .isArray=false, .length=1}, GeneratorInterface, &cyclic);
		if (status != OK) {osSemaphoreRelease(GeneratorSemaphore); return false;}
	}
	osSemaphoreRelease(GeneratorSemaphore);
	return true;
}

/**
 * @brief Activate H-bridges and controllers
 */
bool MotionGenerator::ActivateModuleControllers(void) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	uint8_t status;
	for (uint8_t index = 0; index < number_of_modules; index++) {
		GeneratorInterface.address = STM32_HEAD + 1 + index;

		// Activate H-bridge
		if (GENERATOR_CONFIGURATON_DELAY > 0) osDelay(GENERATOR_CONFIGURATON_DELAY);
		uint8_t bridge_state = (uint8_t) true;
		status = services->WriteRemoteRegister<uint8_t>(Register{.address=REG_BRIDGE_STATE, .type=UINT8_TYPE, .isArray=false, .length=1}, GeneratorInterface, &bridge_state);
		if (status != OK) {osSemaphoreRelease(GeneratorSemaphore); return false;}

		// Activate controller
		if (GENERATOR_CONFIGURATON_DELAY > 0) osDelay(GENERATOR_CONFIGURATON_DELAY);
		uint8_t controller_state = (uint8_t) true;
		status = services->WriteRemoteRegister<uint8_t>(Register{.address=REG_CONTROLLER_STATE, .type=UINT8_TYPE, .isArray=false, .length=1}, GeneratorInterface, &controller_state);
		if (status != OK) {osSemaphoreRelease(GeneratorSemaphore); return false;}
	}
	osSemaphoreRelease(GeneratorSemaphore);
	return true;
}

/**
 * @brief Activate H-bridges and controllers
 */
bool MotionGenerator::DeactivateModuleControllers(void) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	for (uint8_t index = 0; index < number_of_modules; index++) {
		GeneratorInterface.address = STM32_HEAD + 1 + index;

		// Deactivate H-bridge
		if (GENERATOR_CONFIGURATON_DELAY > 0) osDelay(GENERATOR_CONFIGURATON_DELAY);
		uint8_t bridge_state = (uint8_t) false;
		services->WriteRemoteRegister<uint8_t>(Register{.address=REG_BRIDGE_STATE, .type=UINT8_TYPE, .isArray=false, .length=1}, GeneratorInterface, &bridge_state);

		// Deactivate controller
		if (GENERATOR_CONFIGURATON_DELAY > 0) osDelay(GENERATOR_CONFIGURATON_DELAY);
		uint8_t controller_state = (uint8_t) false;
		services->WriteRemoteRegister<uint8_t>(Register{.address=REG_CONTROLLER_STATE, .type=UINT8_TYPE, .isArray=false, .length=1}, GeneratorInterface, &controller_state);
	}
	osSemaphoreRelease(GeneratorSemaphore);
	return true;
}

/**
 * @brief Function to compute setpoints via MotionGenerator and send targets to modules
 */
void MotionGenerator::Spin(void) {
	// Check if module configuration requested from register access
	if (request_module_configuration) {
		request_module_configuration = false;
		ConfigureModules();
	}
	// Check if module controller activate requested from register access
	if (request_module_controller_activation) {
		request_module_controller_activation = false;
		ActivateModuleControllers();
	}
	// Check if module controller deactivate requested from register access
	if (request_module_controller_deactivation) {
		request_module_controller_deactivation = false;
		DeactivateModuleControllers();
	}

	// Motion generator
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	if (active) {
		uint32_t time = HAL_GetTick();
		float x = 0;
		for (uint8_t index = 0; index < number_of_modules; index++) {
			// Compute module position
			x += module_length + position_offsets[index];

			// Compute setpoint
			float setpoint = amplitude * sin(2.0f * M_PI * frequency * (time - start_time) * 0.001f + 2.0f * M_PI * wavelength_inverse * x + phase);

			// Add offset
			setpoint += generator_offsets[index];

			// Store the setpoint
			setpoints[index] = setpoint;

			// Send the setpoint to the respective module
			GeneratorInterface.address = STM32_HEAD + 1 + index;
			services->WriteRemoteRegister<float>(MoveToRegister, GeneratorInterface, &setpoint);
		}
	}
	osSemaphoreRelease(GeneratorSemaphore);

	publishers->SpinPublisher(PUBLISHER_MOTION_GENERATOR);

	osDelay(period_ms);
}

/**
 * @brief Set the generator status
 *
 * @param input status: generator status
 * @return whether successful
 */
bool MotionGenerator::SetGeneratorStatus(bool status) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);

#if defined(USE_LEDS) && defined(USE_SMD_LEDs)
	if (status) {
		leds->ConfigureTimings(LED_ACTIVITY, std::vector<uint8_t>{1}, 10);
		leds->Blink(LED_ACTIVITY);
	} else {
		leds->ClearLED(LED_ACTIVITY);
	}
#endif

	active = status;
	osSemaphoreRelease(GeneratorSemaphore);
	return true;
}

/**
 * @brief Activate the generator
 *
 * @return whether successful
 */
bool MotionGenerator::ActivateGenerator(void) {
	start_time = HAL_GetTick();
	return SetGeneratorStatus(true);
}

/**
 * @brief Deactivate the generator
 *
 * @return whether successful
 */
bool MotionGenerator::DeactivateGenerator(void) {
	return SetGeneratorStatus(false);
}

/**
 * @brief Set the generator period
 *
 * @param input period_ms_: generator period in ms
 * @return whether successful
 */
bool MotionGenerator::SetGeneratorPeriod(uint32_t period_ms_) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	period_ms = period_ms_;
	osSemaphoreRelease(GeneratorSemaphore);
	return true;
}

/**
 * @brief Set the generator number of modules in the robot
 *
 * @param input nb_modules: number of modules in the robot. Careful, if too high, the generator will block as the service to set the setpoint won't return
 * @return whether successful
 */
bool MotionGenerator::SetNumberOfModules(uint8_t nb_modules) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	number_of_modules = nb_modules;

	// Resize the generator offsets
	uint8_t size = generator_offsets.size();
	generator_offsets.resize(nb_modules);
	std::fill(generator_offsets.begin() + size, generator_offsets.end(), 0);

	// Resize the position offsets
	size = position_offsets.size();
	position_offsets.resize(nb_modules);
	std::fill(position_offsets.begin() + size, position_offsets.end(), 0);

	osSemaphoreRelease(GeneratorSemaphore);
	return true;
}

/**
 * @brief Set the generator offset for a given module
 *
 * @param input index: module index with 0 = module directly attached to head
 * @param input offset: the generator offset
 * @return whether successful
 */
bool MotionGenerator::SetGeneratorOffset(uint8_t index, float offset) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	if (index >= generator_offsets.size()) {osSemaphoreRelease(GeneratorSemaphore); return false;}

	generator_offsets[index] = offset;
	osSemaphoreRelease(GeneratorSemaphore);
	return true;
}

/**
 * @brief Reset all generator offsets to 0
 *
 * @return whether successful
 */
bool MotionGenerator::ResetGeneratorOffsets(void) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	for (float &offset : generator_offsets) {
		offset = 0;
	}
	osSemaphoreRelease(GeneratorSemaphore);
	return true;
}

/**
 * @brief Set the position offset for a given module
 *
 * @param input index: module index with 0 = module directly attached to head
 * @param input offset: the position offset
 * @return whether successful
 */
bool MotionGenerator::SetPositionOffset(uint8_t index, float offset) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	if (index >= position_offsets.size()) {osSemaphoreRelease(GeneratorSemaphore); return false;}

	position_offsets[index] = offset;
	osSemaphoreRelease(GeneratorSemaphore);
	return true;
}

/**
 * @brief Set the module length
 *
 * @param input length: the module length
 * @return whether successful
 */
bool MotionGenerator::SetModuleLength(float length) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	module_length = length;
	osSemaphoreRelease(GeneratorSemaphore);
	return true;
}

/**
 * @brief Set the generator amplitude
 *
 * @param input amplitude_: waveform amplitude A with Asin(...)
 * @return whether successful
 */
bool MotionGenerator::SetGeneratorAmplitude(float amplitude_) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	amplitude = amplitude_;
	osSemaphoreRelease(GeneratorSemaphore);
	return true;
}

/**
 * @brief Set the generator frequency
 *
 * @param input frequency_: waveform frequency
 * @return whether successful
 */
bool MotionGenerator::SetGeneratorFrequency(float frequency_) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	frequency = frequency_;
	osSemaphoreRelease(GeneratorSemaphore);
	return true;
}

/**
 * @brief Set the generator wavelength inverse
 *
 * @param input wavelength_inverse_: waveform wavelength inverse 1/l with Asin(.. + 2*pi*x/l)
 * @return whether successful
 */
bool MotionGenerator::SetGeneratorWavelengthInverse(float wavelength_inverse_) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	wavelength_inverse = wavelength_inverse_;
	osSemaphoreRelease(GeneratorSemaphore);
	return true;
}

/**
 * @brief Set the generator phase
 *
 * @param input wavelength_inverse_: waveform phase
 * @return whether successful
 */
bool MotionGenerator::SetGeneratorPhase(float phase_) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	phase = phase_;
	osSemaphoreRelease(GeneratorSemaphore);
	return true;
}

/**
 * @brief Get the generator period
 *
 * @param output success: whether successful
 * @return the generator period
 */
uint32_t MotionGenerator::GetGeneratorPeriod(bool* success) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	uint32_t period_ms_ = period_ms;
	osSemaphoreRelease(GeneratorSemaphore);
	*success = true;
	return period_ms_;
}

/**
 * @brief Get the number of modules configured in the generator
 *
 * @param output success: whether successful
 * @return the number of modules
 */
uint8_t MotionGenerator::GetNumberOfModules(bool* success) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	uint8_t nb_modules = number_of_modules;
	osSemaphoreRelease(GeneratorSemaphore);
	*success = true;
	return nb_modules;
}

/**
 * @brief Get the generator offset for a given module
 *
 * @param input index: module index with 0 = module directly attached to head
 * @param output success: whether successful
 * @return the generator offset of the input module
 */
float MotionGenerator::GetGeneratorOffset(uint8_t index, bool* success) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	if (index >= generator_offsets.size()) {osSemaphoreRelease(GeneratorSemaphore); *success = false; return 0;}
	float offset = generator_offsets[index];
	osSemaphoreRelease(GeneratorSemaphore);
	*success = true;
	return offset;
}

/**
 * @brief Get the position offset for a given module
 *
 * @param input index: module index with 0 = module directly attached to head
 * @param output success: whether successful
 * @return the position offset of the input module
 */
float MotionGenerator::GetPositionOffset(uint8_t index, bool* success) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	if (index >= position_offsets.size()) {osSemaphoreRelease(GeneratorSemaphore); *success = false; return 0;}
	float offset = position_offsets[index];
	osSemaphoreRelease(GeneratorSemaphore);
	*success = true;
	return offset;
}

/**
 * @brief Get the module length
 *
 * @param output success: whether successful
 * @return the module length
 */
float MotionGenerator::GetModuleLength(bool* success) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	float length = module_length;
	osSemaphoreRelease(GeneratorSemaphore);
	*success = true;
	return length;
}

/**
 * @brief Get the waveform amplitude
 *
 * @param output success: whether successful
 * @return the waveform amplitude
 */
float MotionGenerator::GetGeneratorAmplitude(bool* success) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	float amplitude_ = amplitude;
	osSemaphoreRelease(GeneratorSemaphore);
	*success = true;
	return amplitude_;
}

/**
 * @brief Get the waveform frequency
 *
 * @param output success: whether successful
 * @return the waveform frequency
 */
float MotionGenerator::GetGeneratorFrequency(bool* success) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	float frequency_ = frequency;
	osSemaphoreRelease(GeneratorSemaphore);
	*success = true;
	return frequency_;
}

/**
 * @brief Get the waveform wavelength inverse
 *
 * @param output success: whether successful
 * @return the waveform wavelength inverse
 */
float MotionGenerator::GetGeneratorWavelengthInverse(bool* success) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	float wavelength_inverse_ = wavelength_inverse;
	osSemaphoreRelease(GeneratorSemaphore);
	*success = true;
	return wavelength_inverse_;
}

/**
 * @brief Get the waveform phase
 *
 * @param output success: whether successful
 * @return the waveform phase
 */
float MotionGenerator::GetGeneratorPhase(bool* success) {
	osSemaphoreAcquire(GeneratorSemaphore, osWaitForever);
	float phase_ = phase;
	osSemaphoreRelease(GeneratorSemaphore);
	*success = true;
	return phase_;
}
