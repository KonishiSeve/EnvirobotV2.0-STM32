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

	// Add registers
	void AddPressureSensorsRegisters(void);
	void AddBatterySensorRegisters(void);
	void AddMotorSensorRegisters(void);

	// Add sensors
	void AddPressureSensors(void);
	void AddBatterySensor(void);
	void AddMotorSensor(void);
	void AddVoltageSensor(ADC_HandleTypeDef* ADC3_3V, ADC_HandleTypeDef* ADC5V);

	// Activate sensors
	void ActivatePressureSensors(void);
	void ActivateBatterySensor(void);
	void ActivateMotorSensor(void);
	void ActivateVoltageSensor(void);

	// Configure Sensors
	void ConfigurePressureSensors(void);
	void ConfigureMotorSensor(void);

	// Setup sensor publisher
	void SetupPressureSensorsPublisher(void);
	void SetupBatterySensorPublisher(void);
	void SetupMotorSensorPublisher(void);
	void SetupVoltageSensorPublisher(void);

private:
	Sensors* sensors;
	Registers* registers;
	Publishers* publishers;
	LEDS* leds;

	// Pressure sensor variables
	uint16_t CONFIG_SENS_1;
	uint16_t CONFIG_OFF_1;
	uint16_t CONFIG_TCS_1;
	uint16_t CONFIG_TCO_1;
	uint16_t CONFIG_TREF_1;
	uint16_t CONFIG_TEMPSENS_1;

	uint16_t CONFIG_SENS_2;
	uint16_t CONFIG_OFF_2;
	uint16_t CONFIG_TCS_2;
	uint16_t CONFIG_TCO_2;
	uint16_t CONFIG_TREF_2;
	uint16_t CONFIG_TEMPSENS_2;

	int32_t pressure_sensor_T1;
	int32_t pressure_sensor_P1;
	int32_t pressure_sensor_T2;
	int32_t pressure_sensor_P2;

	// Battery sensor variables
	float voltage_low_threshold;
	float Vcell1;
	float Vcell2;
	float VBAT;
	float IBAT;
	float TBAT;

	// Motor sensor variables
	float Vmotor;
	float Imotor;
	float Pmotor;
	double Emotor;
	float Emotor_float;
	float Tmotor;

	// Voltage sensor variables
	ADC_HandleTypeDef* ADC3_3V;
	ADC_HandleTypeDef* ADC5V;

	float Vref_int;
	uint8_t voltage3_3V_resolution;
	uint8_t voltage5V_resolution;
	float voltage3_3V_conversion_ratio;
	float voltage5V_conversion_ratio;
};
