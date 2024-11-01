/*
 * Controller.hpp
 *
 *  Created on: Nov 23, 2022
 *      Author: bignet
 */

#pragma once

#include <stm32h750xx.h>
#include <stm32h7xx_hal.h>
#include "cmsis_os.h"

#include "GPIO/GPIO.hpp"
#include "Trajectory/TrajectoryGenerator.hpp"
#include "Registers/Registers.hpp"
#include "Publishers/Publishers.hpp"
#include "LEDS/LEDS.hpp"

#include "Configuration.h"
#include "RegisterMaps/RegisterMapController.h"
#include "RegisterMaps/RegisterMapSensors.h"
#include "Configurations/ControlConfiguration.h"

#include "PlatformLEDs/PlatformLEDs.hpp"

// Controller Configuration Struct
struct ControllerConfiguration {
	float* track; 														// pointer to regulate
	uint8_t selected_filter; 											// selected filter ID
	std::vector<float (*)(float, std::vector<float>)> input_filters; 	// vector of filters to be applied to inputs of MoveFromTo and MoveTo functions
	float K;	 														// Proportional gain
	float Ti_inv; 														// Inverse of Ti with Ki=K/Ti
	float Td;															// Td with Kd=K*Td
	float Isaturation; 													// Integral saturation
	bool P_active; 														// Proportional controller active
	bool I_active; 														// Integral controller active
	bool D_active; 														// Derivation controller active
	bool S_active; 														// Integral saturation active
	float* direction_criteria; 											// Pointer whose sign is used to define the H-bridge orientation via IN1, IN2
	uint32_t (*PWMAssignement)(float); 									// Function used to set the controller PWM from the controller output
	bool model_active; 													// Whether additional model is active or not
	float (*model)(std::vector<float*>, std::vector<float>); 			// Function to add a custom model to the controller output
	std::vector<float*> model_variables; 								// Vector of variables used by the custom model
};

// Controller class used to control the motor
class Controller {
public:
	Controller(TIM_TypeDef* encoder_timer, uint32_t* pulse_, GPIO EN, GPIO SHIFTER, GPIO IN1, GPIO IN2, ADC_HandleTypeDef* adc);
	void Init(Registers* registers, Publishers* publishers, LEDS* leds);
	void AddRegisters(void);
	void SetupPublisher(uint8_t interface_ID);

	void ActivateBridge(void);
	void DeactivateBridge(void);
	void ActivateController(void);
	void DeactivateController(void);
	void ResetController(void);
	void SetControllerPeriodMs(uint32_t period);

	void SelectControllerMode(uint8_t mode);
	void SelectInputFilter(uint8_t mode, uint8_t filter);
	void SetIntegrationMode(uint8_t mode);
	void SetDerivationMode(uint8_t mode);

	void MoveFromTo(float start, float target);
	void MoveTo(float target);
	void Control(void);

	void SetEncoderZero(void);

	uint8_t GetIntegrationMode(void);
	uint8_t GetDerivationMode(void);
	int32_t GetEncoder(void);
	float GetCurrent(void);
	int32_t GetSetPoint(void);

	TrajectoryGenerator trajectory_generator;
private:
	float Derivation(float values[], uint8_t mode);
	float Integration(float* integral, float values[], bool saturation_active, float saturation, uint8_t mode);
	void MeasureCurrent(void);

	// OS
	osSemaphoreId_t ControlSemaphore;

	Registers* registers;
	Publishers* publishers;
	LEDS* leds;

	// Interfaces
	TIM_TypeDef* encoder;
	uint32_t* pulse;
	ADC_HandleTypeDef* ADC;
	GPIO EN;
	GPIO SHIFTER;
	GPIO IN1;
	GPIO IN2;

	// Configuration parameters
	bool bridge_active;
	bool controller_active;
	uint8_t mode; // POSITION_MODE, TORQUE_MODE
	uint32_t period_ms;
	struct ControllerConfiguration positionConfiguration;
	struct ControllerConfiguration torqueConfiguration;
	struct ControllerConfiguration PWMConfiguration;
	ControllerConfiguration* configurations[3] = {&positionConfiguration, &torqueConfiguration, &PWMConfiguration};
	std::vector<float> params;
	uint8_t integration_type;
	uint8_t derivation_type;

	// Variables
	float setpoint;
	float PWM;
	int32_t encoder_offset;
	uint32_t encoder_security_threshold; // if 0, not activated
	float encoder_position;
	float encoder_history[3];
	float encoder_speed;
	float error_history[3];
	float motor_voltage;
	float motor_current;
	float motor_current_compensation;
	float motor_power;
	float error_integral;

	// Precomputed variables
	float period_s;
	float period_s_half;
	float period_s_inv;
	float period_s_double_inv;
	float current_conversion;
};
