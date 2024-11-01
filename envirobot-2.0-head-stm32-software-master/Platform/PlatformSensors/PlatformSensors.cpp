#include "PlatformSensors.hpp"

/**
 * @brief Class constructor
 */
PlatformSensors::PlatformSensors() {

}

/**
 * @brief Initialize the class with references to other classes. Set default values.
 *
 * @param input registers_: the Registers instance
 * @param input publishers_: the Publishers instance
 * @param input leds_: the LEDS instance
 */
void PlatformSensors::Init(Sensors* sensors_, Registers* registers_, Publishers* publishers_, LEDS* leds_) {
	sensors = sensors_;
	registers = registers_;
	publishers = publishers_;
	leds = leds_;
}

/**
 * @brief Read the 3.3V voltage level from ADC and internal reference
 *
 * @param input timeout: time left to measure the voltage level with the ADC
 * @return the 3.3V voltage level in V
 */
float PlatformSensors::ReadVoltage3_3V(uint32_t timeout) {
	// Measure voltage
	HAL_ADC_Start(ADC3_3V);
	HAL_ADC_PollForConversion(ADC3_3V, timeout);
	uint16_t ADC_conversion = HAL_ADC_GetValue(ADC3_3V);

	// Convert ADC value
	float voltage = voltage3_3V_conversion_ratio / ADC_conversion;

	// Return measure
	HAL_ADC_Stop(ADC3_3V);
	return voltage;
}

/**
 * @brief Read the 5V voltage level from ADC
 *
 * @param input timeout: time left to measure the voltage level with the ADC
 * @return the 5V voltage level in V
 */
float PlatformSensors::ReadVoltage5V(uint32_t timeout) {
	bool success;
	float voltage_reference;
	uint16_t length;

	// Get 3.3V voltage reference
	success = registers->ReadRegister<float>(REG_3_3V, &voltage_reference, &length, false);
	if (!success || voltage_reference == 0)
		voltage_reference = 3.3f;

	// Measure voltage
	HAL_ADC_Start(ADC5V);
	HAL_ADC_PollForConversion(ADC5V, timeout);
	uint16_t ADC_conversion = HAL_ADC_GetValue(ADC5V);

	// Convert ADC value
	float voltage = voltage5V_conversion_ratio * voltage_reference * ADC_conversion;

	// Return measure
	HAL_ADC_Stop(ADC5V);
	return voltage;
}

/**
 * @brief Get ADC resolution in bytes from hardware ADC handle
 *
 * @param input ADC: the hardware ADC handle
 * @return the ADC resolution in bytes
 */
uint8_t PlatformSensors::GetADCResolution(ADC_HandleTypeDef* ADC) {
	uint32_t resolution_configuration = ADC->Instance->CFGR & ADC_CFGR_RES_Msk;
	if (resolution_configuration == LL_ADC_RESOLUTION_8B) 		return 8;
	if (resolution_configuration == LL_ADC_RESOLUTION_10B) 		return 10;
	if (resolution_configuration == LL_ADC_RESOLUTION_12B) 		return 12;
	if (resolution_configuration == LL_ADC_RESOLUTION_12B_OPT) 	return 12;
	if (resolution_configuration == LL_ADC_RESOLUTION_14B) 		return 14;
	if (resolution_configuration == LL_ADC_RESOLUTION_14B_OPT) 	return 14;
	if (resolution_configuration == LL_ADC_RESOLUTION_16B) 		return 16;
	return 0;
}

/**
 * @brief Add voltage sensor and setup reading sequence
 */
void PlatformSensors::AddVoltageSensor(ADC_HandleTypeDef* ADC3_3V_, ADC_HandleTypeDef* ADC5V_) {
	ADC3_3V = ADC3_3V_;
	ADC5V = ADC5V_;

	// Setup voltage constants
	Vref_int = (float) VREFINT_CAL_VREF / 1000.0f;
	voltage3_3V_resolution = GetADCResolution(ADC3_3V);
	voltage5V_resolution = GetADCResolution(ADC5V);
	voltage3_3V_conversion_ratio = Vref_int * (*VREFINT_CAL >> (16 - voltage3_3V_resolution));
	voltage5V_conversion_ratio = DIVISION_STAGE_5V / (pow(2, voltage5V_resolution) - 1);

	// Add sensor group
	sensors->AddSensorGroup(INTERNAL, VOLTAGES, true);

	// Set group prescaler
	sensors->SetSensorGroupPrescaler(VOLTAGES, VOLTAGES_PRESCALER);

	// Add sensor
	sensors->AddSensor(VOLTAGES, SENSOR_ADC, 0);

	// Add reading of the 3.3V
	sensors->AddSensorData<float>(SENSOR_ADC, REG_3_3V, true);
	sensors->AddCodeSequence<float>(SENSOR_ADC, REG_3_3V, (void*) this,
		[](void* context) -> float {
			PlatformSensors* self = (PlatformSensors*) context;
			return self->ReadVoltage3_3V();
		}
	);

	// Add reading of the 5V
	sensors->AddSensorData<float>(SENSOR_ADC, REG_5V, true);
	sensors->AddCodeSequence<float>(SENSOR_ADC, REG_5V, (void*) this,
		[](void* context) -> float {
			PlatformSensors* self = (PlatformSensors*) context;
			return self->ReadVoltage5V();
		}
	);
}

/**
 * @brief Activate voltage sensor
 */
void PlatformSensors::ActivateVoltageSensor(void) {
	// Activate sensor data
#ifdef MEAS_3_3V_ACTIVE
	sensors->ActivateSensorData(SENSOR_ADC, REG_3_3V);
#endif
#ifdef MEAS_5V_ACTIVE
	sensors->ActivateSensorData(SENSOR_ADC, REG_5V);
#endif

	// Activate sensor
#ifdef SENSOR_ADC_ACTIVE
	sensors->ActivateSensor(SENSOR_ADC);
#endif

#ifdef VOLTAGES_ACTIVE
	// Activate group
	sensors->ActivateSensorGroup(VOLTAGES);
#endif
}

/**
 * @brief Setup voltage sensor publisher
 *
 * @param input interface_ID: interface to publish data on
 */
void PlatformSensors::SetupVoltageSensorPublisher(void) {
	// Link interface to publisher
	publishers->LinkToInterface(VOLTAGES, PUBLISHER_VOLTAGES_INTERFACE);

	// Setup publisher
	publishers->SetPublisherPrescaler(VOLTAGES, PUBLISHER_VOLTAGES_PRESCALER);
	publishers->SetPublishAddress(VOLTAGES, PUBLISHER_VOLTAGES_INTERFACE, PUBLISHER_VOLTAGES_ADDRESS);

	// Activate topics
#ifdef PUBLISH_VOLTAGES_TIMEBASE
	publishers->ActivateTopic(VOLTAGES, REG_TIMEBASE);
#endif
#ifdef PUBLISH_VOLTAGES_3_3V
	publishers->ActivateTopic(VOLTAGES, REG_3_3V);
#endif
#ifdef PUBLISH_VOLTAGES_5V
	publishers->ActivateTopic(VOLTAGES, REG_5V);
#endif

#ifdef PUBLISHER_VOLTAGES_ACTIVE
	// Activate publisher
	publishers->ActivatePublisher(VOLTAGES);
#endif
}
