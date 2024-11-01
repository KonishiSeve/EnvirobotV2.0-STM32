/**
 * This file defines platform specific LED functions
 */

#pragma once

#include <stm32h750xx.h>
#include <stm32h7xx_hal.h>
#include "cmsis_os.h"

#include "Registers/Registers.hpp"
#include "Sensors/Sensors.hpp"
#include "Publishers/Publishers.hpp"

#include "RegisterMaps/RegisterMapSensors.h"
#include "Configurations/SensorsConfiguration.h"
#include "Hardware/MS5803.h"
#include "Hardware/PCA9633.h"
#include "Hardware/LTC2947.h"
#include "Hardware/DS2778.h"

class PlatformSensors {
public:
	PlatformSensors();
	void Init(Sensors* sensors, Registers* registers, Publishers* publishers, LEDS* leds);

	// ADC functions
	float ReadVoltage3_3V(uint32_t timeout = HAL_MAX_DELAY);
	float ReadVoltage5V(uint32_t timeout = HAL_MAX_DELAY);
	uint8_t GetADCResolution(ADC_HandleTypeDef* ADC);

	// Add sensors
	void AddVoltageSensor(ADC_HandleTypeDef* ADC3_3V, ADC_HandleTypeDef* ADC5V);

	// Activate sensors
	void ActivateVoltageSensor(void);

	// Setup sensor publisher
	void SetupVoltageSensorPublisher(void);

private:
	Sensors* sensors;
	Registers* registers;
	Publishers* publishers;
	LEDS* leds;

	// Voltage sensor variables
	ADC_HandleTypeDef* ADC3_3V;
	ADC_HandleTypeDef* ADC5V;

	float Vref_int;
	uint8_t voltage3_3V_resolution;
	uint8_t voltage5V_resolution;
	float voltage3_3V_conversion_ratio;
	float voltage5V_conversion_ratio;
};
