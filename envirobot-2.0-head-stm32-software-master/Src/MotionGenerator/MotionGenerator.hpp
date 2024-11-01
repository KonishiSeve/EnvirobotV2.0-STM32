/*
 * MotionGenerator.hpp
 *
 *  Created on: 2 févr. 2023
 *      Author: bignet
 */

#pragma once

#include <vector>
#include <stm32h750xx.h>
#include <stm32h7xx_hal.h>
#include "cmsis_os.h"
#include <math.h>

#include "Registers/Registers.hpp"
#include "Publishers/Publishers.hpp"
#include "Services/Services.hpp"
#include "LEDS/LEDS.hpp"

#include "Configuration.h"
#include "Configurations/LEDsConfiguration.h"
#include "Configurations/MotionGeneratorConfiguration.h"
#include "RegisterMaps/RegisterMapMotionGenerator.h"
#include "RegisterMaps/RegisterMapController.h"

// MotionGenerator class to generate a sinusoidal motion of the robot. It controls all modules in the chain
class MotionGenerator {
public:
	MotionGenerator();
	void Init(Registers* registers, Publishers* publishers, Services* services, LEDS* leds);
	void AddRegisters(void);
	void SetupPublisher(void);

	bool ConfigureModules(void); 					// configure module controllers
	bool ActivateModuleControllers(void); 			// activate controllers and bridges of every modules
	bool DeactivateModuleControllers(void); 		// deactivate controllers and bridges of every modules

	void Spin(void);

	bool SetGeneratorStatus(bool status);
	bool ActivateGenerator(void);
	bool DeactivateGenerator(void);
	bool SetGeneratorPeriod(uint32_t period_ms);
	bool SetNumberOfModules(uint8_t nb_modules);
	bool SetGeneratorOffset(uint8_t index, float offset);
	bool ResetGeneratorOffsets(void);
	bool SetPositionOffset(uint8_t index, float offset);
	bool SetModuleLength(float length);
	bool SetGeneratorAmplitude(float amplitude);
	bool SetGeneratorFrequency(float frequency);
	bool SetGeneratorWavelengthInverse(float wavelength_inverse);
	bool SetGeneratorPhase(float phase);

	uint32_t GetGeneratorPeriod(bool* success);
	uint8_t GetNumberOfModules(bool* success);
	float GetGeneratorOffset(uint8_t index, bool* success);
	float GetPositionOffset(uint8_t index, bool* success);
	float GetModuleLength(bool* success);
	float GetGeneratorAmplitude(bool* success);
	float GetGeneratorFrequency(bool* success);
	float GetGeneratorWavelengthInverse(bool* success);
	float GetGeneratorPhase(bool* success);

private:
	Registers* registers;
	Publishers* publishers;
	Services* services;
	LEDS* leds;

	// OS
	osSemaphoreId_t GeneratorSemaphore;

	bool active;
	uint32_t period_ms;
	uint8_t number_of_modules;
	float module_length;
	std::vector<float> generator_offsets;
	std::vector<float> position_offsets;		// with respect to the previous module motor shaft position
	float amplitude;
	float frequency;
	float wavelength_inverse;
	float phase;

	uint32_t start_time;
	std::vector<float> setpoints;

	Register MoveToRegister;
	ServiceInterface GeneratorInterface;

	bool request_module_configuration;
	bool request_module_controller_activation;
	bool request_module_controller_deactivation;
};
