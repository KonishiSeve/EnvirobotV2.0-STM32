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
 * @brief Add pressure sensor registers
 */
void PlatformSensors::AddPressureSensorsRegisters(void) {
	// Register of the converted temperature of the first pressure sensor
	pressure_sensor_T1 = 0;
	registers->AddRegister<int32_t>(REG_PRESSURE_SENSOR_T1);
	registers->SetRegisterAsSingle(REG_PRESSURE_SENSOR_T1);
	registers->AddRegisterSemaphore(REG_PRESSURE_SENSOR_T1, &(sensors->SensorsSemaphore));
	registers->AddRegisterPointer<int32_t>(REG_PRESSURE_SENSOR_T1, &pressure_sensor_T1);
	registers->SetRegisterPermissions(REG_PRESSURE_SENSOR_T1, READ_PERMISSION);

	// Register of the converted pressure of the first pressure sensor
	pressure_sensor_P1 = 0;
	registers->AddRegister<int32_t>(REG_PRESSURE_SENSOR_P1);
	registers->SetRegisterAsSingle(REG_PRESSURE_SENSOR_P1);
	registers->AddRegisterSemaphore(REG_PRESSURE_SENSOR_P1, &(sensors->SensorsSemaphore));
	registers->AddRegisterPointer<int32_t>(REG_PRESSURE_SENSOR_P1, &pressure_sensor_P1);
	registers->SetRegisterPermissions(REG_PRESSURE_SENSOR_P1, READ_PERMISSION);

	// Register of the converted temperature of the second pressure sensor
	pressure_sensor_T2 = 0;
	registers->AddRegister<int32_t>(REG_PRESSURE_SENSOR_T2);
	registers->SetRegisterAsSingle(REG_PRESSURE_SENSOR_T2);
	registers->AddRegisterSemaphore(REG_PRESSURE_SENSOR_T2, &(sensors->SensorsSemaphore));
	registers->AddRegisterPointer<int32_t>(REG_PRESSURE_SENSOR_T2, &pressure_sensor_T2);
	registers->SetRegisterPermissions(REG_PRESSURE_SENSOR_T2, READ_PERMISSION);

	// Register of the converted pressure of the second pressure sensor
	pressure_sensor_P2 = 0;
	registers->AddRegister<int32_t>(REG_PRESSURE_SENSOR_P2);
	registers->SetRegisterAsSingle(REG_PRESSURE_SENSOR_P2);
	registers->AddRegisterSemaphore(REG_PRESSURE_SENSOR_P2, &(sensors->SensorsSemaphore));
	registers->AddRegisterPointer<int32_t>(REG_PRESSURE_SENSOR_P2, &pressure_sensor_P2);
	registers->SetRegisterPermissions(REG_PRESSURE_SENSOR_P2, READ_PERMISSION);
}

/**
 * @brief Add pressure sensors and setup reading sequences
 */
void PlatformSensors::AddPressureSensors(void) {
	// Add sensor group
	sensors->AddSensorGroup(I2C1_ID, PRESSURE_SENSORS, true);

	// Set group prescaler
	sensors->SetSensorGroupPrescaler(PRESSURE_SENSORS, PRESSURE_SENSORS_PRESCALER);

	// Add pressure sensors
	sensors->AddSensor(PRESSURE_SENSORS, PRESSURE_SENSOR_1, MS5803_ADDRESS_1);
	sensors->AddSensor(PRESSURE_SENSORS, PRESSURE_SENSOR_2, MS5803_ADDRESS_2);

	// Add reading of raw pressure from the first pressure sensor
	sensors->AddSensorData<uint32_t>(PRESSURE_SENSOR_1, REG_PRESSURE_SENSOR_P1_RAW, true);
	sensors->AddWriteSequence(PRESSURE_SENSOR_1, REG_PRESSURE_SENSOR_P1_RAW, MS5803_OSR_4096_P);
	sensors->AddDelaySequence(PRESSURE_SENSOR_1, REG_PRESSURE_SENSOR_P1_RAW, MS5803_OSR_4096_DELAY);
	sensors->AddWriteSequence(PRESSURE_SENSOR_1, REG_PRESSURE_SENSOR_P1_RAW, MS5803_ADC_READ);
	sensors->AddReadSequence(PRESSURE_SENSOR_1, REG_PRESSURE_SENSOR_P1_RAW, MS5803_DATA_SIZE);

	// Add reading of raw temperature from the first pressure sensor
	sensors->AddSensorData<uint32_t>(PRESSURE_SENSOR_1, REG_PRESSURE_SENSOR_T1_RAW, true);
	sensors->AddWriteSequence(PRESSURE_SENSOR_1, REG_PRESSURE_SENSOR_T1_RAW, MS5803_OSR_4096_T);
	sensors->AddDelaySequence(PRESSURE_SENSOR_1, REG_PRESSURE_SENSOR_T1_RAW, MS5803_OSR_4096_DELAY);
	sensors->AddWriteSequence(PRESSURE_SENSOR_1, REG_PRESSURE_SENSOR_T1_RAW, MS5803_ADC_READ);
	sensors->AddReadSequence(PRESSURE_SENSOR_1, REG_PRESSURE_SENSOR_T1_RAW, MS5803_DATA_SIZE);

	// Add reading of raw pressure from the second pressure sensor
	sensors->AddSensorData<uint32_t>(PRESSURE_SENSOR_2, REG_PRESSURE_SENSOR_P2_RAW, true);
	sensors->AddWriteSequence(PRESSURE_SENSOR_2, REG_PRESSURE_SENSOR_P2_RAW, MS5803_OSR_4096_P);
	sensors->AddDelaySequence(PRESSURE_SENSOR_2, REG_PRESSURE_SENSOR_P2_RAW, MS5803_OSR_4096_DELAY);
	sensors->AddWriteSequence(PRESSURE_SENSOR_2, REG_PRESSURE_SENSOR_P2_RAW, MS5803_ADC_READ);
	sensors->AddReadSequence(PRESSURE_SENSOR_2, REG_PRESSURE_SENSOR_P2_RAW, MS5803_DATA_SIZE);

	// Add reading of raw temperature from the second pressure sensor
	sensors->AddSensorData<uint32_t>(PRESSURE_SENSOR_2, REG_PRESSURE_SENSOR_T2_RAW, true);
	sensors->AddWriteSequence(PRESSURE_SENSOR_2, REG_PRESSURE_SENSOR_T2_RAW, MS5803_OSR_4096_T);
	sensors->AddDelaySequence(PRESSURE_SENSOR_2, REG_PRESSURE_SENSOR_T2_RAW, MS5803_OSR_4096_DELAY);
	sensors->AddWriteSequence(PRESSURE_SENSOR_2, REG_PRESSURE_SENSOR_T2_RAW, MS5803_ADC_READ);
	sensors->AddReadSequence(PRESSURE_SENSOR_2, REG_PRESSURE_SENSOR_T2_RAW, MS5803_DATA_SIZE);

	// Add sensor callbacks
	sensors->AddSensorCallback(PRESSURE_SENSOR_1, (void*) this,
		  [](void* context) -> void {
			  PlatformSensors* self = (PlatformSensors*) context;
			  bool success;
			  uint16_t length;

			  if (!self->sensors->IsSensorDataActive(PRESSURE_SENSOR_1, REG_PRESSURE_SENSOR_T1_RAW)) return;

			  // Convert the raw temperature if active
			  uint32_t T_raw;
			  success = self->registers->ReadRegister<uint32_t>(REG_PRESSURE_SENSOR_T1_RAW, &T_raw, &length, false);
			  if (!success) return;

			  int32_t dT = T_raw - ((uint32_t) self->CONFIG_TREF_1 << 8);
			  self->pressure_sensor_T1 = ((int32_t) (((int64_t) dT * (int64_t) self->CONFIG_TEMPSENS_1) >> 23)) + 2000;

			  if (!self->sensors->IsSensorDataActive(PRESSURE_SENSOR_1, REG_PRESSURE_SENSOR_P1_RAW)) return;

			  // Convert the raw pressure if active and temperature available
			  uint32_t P_raw;
			  success = self->registers->ReadRegister<uint32_t>(REG_PRESSURE_SENSOR_P1_RAW, &P_raw, &length, false);
			  if (!success) return;

			  int64_t OFF = ((int64_t) self->CONFIG_OFF_1 << 16) + (((int64_t) self->CONFIG_TCO_1 * (int64_t) dT) >> 7);
			  int64_t SENS = ((int64_t) self->CONFIG_SENS_1 << 15) + (((int64_t) self->CONFIG_TCS_1 * (int64_t) dT) >> 8);
			  self->pressure_sensor_P1 = (int32_t) (((((int64_t) P_raw * SENS) >> 21) - OFF) >> 15);
		  }
	);

	sensors->AddSensorCallback(PRESSURE_SENSOR_2, (void*) this,
		  [](void* context) -> void {
			  PlatformSensors* self = (PlatformSensors*) context;
			  bool success;
			  uint16_t length;

			  if (!self->sensors->IsSensorDataActive(PRESSURE_SENSOR_2, REG_PRESSURE_SENSOR_T2_RAW)) return;

			  // Convert the raw temperature if active
			  uint32_t T_raw;
			  success = self->registers->ReadRegister<uint32_t>(REG_PRESSURE_SENSOR_T2_RAW, &T_raw, &length, false);
			  if (!success) return;

			  int32_t dT = T_raw - ((uint32_t) self->CONFIG_TREF_2 << 8);
			  self->pressure_sensor_T2 = ((int32_t) (((int64_t) dT * (int64_t) self->CONFIG_TEMPSENS_2) >> 23)) + 2000;

			  if (!self->sensors->IsSensorDataActive(PRESSURE_SENSOR_2, REG_PRESSURE_SENSOR_P2_RAW)) return;

			  // Convert the raw pressure if active and temperature available
			  uint32_t P_raw;
			  success = self->registers->ReadRegister<uint32_t>(REG_PRESSURE_SENSOR_P2_RAW, &P_raw, &length, false);
			  if (!success) return;

			  int64_t OFF = ((int64_t) self->CONFIG_OFF_2 << 16) + (((int64_t) self->CONFIG_TCO_2 * (int64_t) dT) >> 7);
			  int64_t SENS = ((int64_t) self->CONFIG_SENS_2 << 15) + (((int64_t) self->CONFIG_TCS_2 * (int64_t) dT) >> 8);
			  self->pressure_sensor_P2 = (int32_t) (((((int64_t) P_raw * SENS) >> 21) - OFF) >> 15); // in bar
		  }
	);
}

/**
 * @brief Activate pressure sensors
 */
void PlatformSensors::ActivatePressureSensors(void) {
	// Activate sensor data
#ifdef MEAS_P1_RAW
	sensors->ActivateSensorData(PRESSURE_SENSOR_1, REG_PRESSURE_SENSOR_P1_RAW);
#endif
#ifdef MEAS_T1_RAW
	sensors->ActivateSensorData(PRESSURE_SENSOR_1, REG_PRESSURE_SENSOR_T1_RAW);
#endif
#ifdef MEAS_P2_RAW
	sensors->ActivateSensorData(PRESSURE_SENSOR_2, REG_PRESSURE_SENSOR_P2_RAW);
#endif
#ifdef MEAS_T2_RAW
	sensors->ActivateSensorData(PRESSURE_SENSOR_2, REG_PRESSURE_SENSOR_T2_RAW);
#endif

	// Activate sensors
#ifdef PRESSURE_SENSOR_1_ACTIVE
	sensors->ActivateSensor(PRESSURE_SENSOR_1);
#endif
#ifdef PRESSURE_SENSOR_2_ACTIVE
	sensors->ActivateSensor(PRESSURE_SENSOR_2);
#endif

#ifdef PRESSURE_SENSORS_ACTIVE
	// Activate group
	sensors->ActivateSensorGroup(PRESSURE_SENSORS);
#endif
}

/**
 * @brief Setup pressure sensor publisher
 *
 * @param input interface_ID: interface to publish data on
 */
void PlatformSensors::SetupPressureSensorsPublisher(void) {
	// Link publisher to interface
	publishers->LinkToInterface(PRESSURE_SENSORS, PUBLISHER_PRESSURES_INTERFACE);

	// Setup publisher
	publishers->SetPublisherPrescaler(PRESSURE_SENSORS, PUBLISHER_PRESSURES_PRESCALER);
	publishers->SetPublishAddress(PRESSURE_SENSORS, PUBLISHER_PRESSURES_INTERFACE, PUBLISHER_PRESSURES_ADDRESS);

	// Activate topics
#ifdef PUBLISH_PRESSURES_TIMEBASE
	publishers->ActivateTopic(PRESSURE_SENSORS, REG_TIMEBASE);
#endif
#ifdef PUBLISH_PRESSURES_P1_RAW
	publishers->ActivateTopic(PRESSURE_SENSORS, REG_PRESSURE_SENSOR_P1_RAW);
#endif
#ifdef PUBLISH_PRESSURES_P2_RAW
	publishers->ActivateTopic(PRESSURE_SENSORS, REG_PRESSURE_SENSOR_P2_RAW);
#endif
#ifdef PUBLISH_PRESSURES_T1_RAW
	publishers->ActivateTopic(PRESSURE_SENSORS, REG_PRESSURE_SENSOR_T1_RAW);
#endif
#ifdef PUBLISH_PRESSURES_T2_RAW
	publishers->ActivateTopic(PRESSURE_SENSORS, REG_PRESSURE_SENSOR_T2_RAW);
#endif
#ifdef PUBLISH_PRESSURES_P1
	publishers->ActivateTopic(PRESSURE_SENSORS, REG_PRESSURE_SENSOR_P1);
#endif
#ifdef PUBLISH_PRESSURES_P2
	publishers->ActivateTopic(PRESSURE_SENSORS, REG_PRESSURE_SENSOR_P2);
#endif
#ifdef PUBLISH_PRESSURES_T1
	publishers->ActivateTopic(PRESSURE_SENSORS, REG_PRESSURE_SENSOR_T1);
#endif
#ifdef PUBLISH_PRESSURES_T2
	publishers->ActivateTopic(PRESSURE_SENSORS, REG_PRESSURE_SENSOR_T2);
#endif

#ifdef PUBLISHER_PRESSURES_ACTIVE
	// Activate publisher
	publishers->ActivatePublisher(PRESSURE_SENSORS);
#endif
}

/**
 * @brief Configuration of pressure sensors
 */
void PlatformSensors::ConfigurePressureSensors(void) {
	// Reset sensors
	sensors->ConfigureSensor(PRESSURE_SENSOR_1, MS5803_RESET);
	sensors->ConfigureSensor(PRESSURE_SENSOR_2, MS5803_RESET);

	HAL_Delay(10);

	// Read configuration parameters
	HAL_StatusTypeDef status;
	sensors->ConfigureSensor(PRESSURE_SENSOR_1, MS5803_PROM_READ | MS5803_PROM_C1_SENS);
	CONFIG_SENS_1 = sensors->ReadSensor<uint16_t>(PRESSURE_SENSOR_1, &status);
	sensors->ConfigureSensor(PRESSURE_SENSOR_1, MS5803_PROM_READ | MS5803_PROM_C2_OFF);
	CONFIG_OFF_1 = sensors->ReadSensor<uint16_t>(PRESSURE_SENSOR_1, &status);
	sensors->ConfigureSensor(PRESSURE_SENSOR_1, MS5803_PROM_READ | MS5803_PROM_C3_TCS);
	CONFIG_TCS_1 = sensors->ReadSensor<uint16_t>(PRESSURE_SENSOR_1, &status);
	sensors->ConfigureSensor(PRESSURE_SENSOR_1, MS5803_PROM_READ | MS5803_PROM_C4_TCO);
	CONFIG_TCO_1 = sensors->ReadSensor<uint16_t>(PRESSURE_SENSOR_1, &status);
	sensors->ConfigureSensor(PRESSURE_SENSOR_1, MS5803_PROM_READ | MS5803_PROM_C5_TREF);
	CONFIG_TREF_1 = sensors->ReadSensor<uint16_t>(PRESSURE_SENSOR_1, &status);
	sensors->ConfigureSensor(PRESSURE_SENSOR_1, MS5803_PROM_READ | MS5803_PROM_C6_TEMPSENS);
	CONFIG_TEMPSENS_1 = sensors->ReadSensor<uint16_t>(PRESSURE_SENSOR_1, &status);

	sensors->ConfigureSensor(PRESSURE_SENSOR_2, MS5803_PROM_READ | MS5803_PROM_C1_SENS);
	CONFIG_SENS_2 = sensors->ReadSensor<uint16_t>(PRESSURE_SENSOR_2, &status);
	sensors->ConfigureSensor(PRESSURE_SENSOR_2, MS5803_PROM_READ | MS5803_PROM_C2_OFF);
	CONFIG_OFF_2 = sensors->ReadSensor<uint16_t>(PRESSURE_SENSOR_2, &status);
	sensors->ConfigureSensor(PRESSURE_SENSOR_2, MS5803_PROM_READ | MS5803_PROM_C3_TCS);
	CONFIG_TCS_2 = sensors->ReadSensor<uint16_t>(PRESSURE_SENSOR_2, &status);
	sensors->ConfigureSensor(PRESSURE_SENSOR_2, MS5803_PROM_READ | MS5803_PROM_C4_TCO);
	CONFIG_TCO_2 = sensors->ReadSensor<uint16_t>(PRESSURE_SENSOR_2, &status);
	sensors->ConfigureSensor(PRESSURE_SENSOR_2, MS5803_PROM_READ | MS5803_PROM_C5_TREF);
	CONFIG_TREF_2 = sensors->ReadSensor<uint16_t>(PRESSURE_SENSOR_2, &status);
	sensors->ConfigureSensor(PRESSURE_SENSOR_2, MS5803_PROM_READ | MS5803_PROM_C6_TEMPSENS);
	CONFIG_TEMPSENS_2 = sensors->ReadSensor<uint16_t>(PRESSURE_SENSOR_2, &status);
}

/**
 * @brief Add battery sensor registers
 */
void PlatformSensors::AddBatterySensorRegisters(void) {
	// Register of the fist cell voltage
	Vcell1 = 0;
	registers->AddRegister<float>(REG_VCELL1);
	registers->SetRegisterAsSingle(REG_VCELL1);
	registers->AddRegisterSemaphore(REG_VCELL1, &(sensors->SensorsSemaphore));
	registers->AddRegisterPointer<float>(REG_VCELL1, &Vcell1);
	registers->SetRegisterPermissions(REG_VCELL1, READ_PERMISSION);

	// Register of the second cell voltage
	Vcell2 = 0;
	registers->AddRegister<float>(REG_VCELL2);
	registers->SetRegisterAsSingle(REG_VCELL2);
	registers->AddRegisterSemaphore(REG_VCELL2, &(sensors->SensorsSemaphore));
	registers->AddRegisterPointer<float>(REG_VCELL2, &Vcell2);
	registers->SetRegisterPermissions(REG_VCELL2, READ_PERMISSION);

	// Register of the battery voltage
	VBAT = 0;
	registers->AddRegister<float>(REG_VBAT);
	registers->SetRegisterAsSingle(REG_VBAT);
	registers->AddRegisterSemaphore(REG_VBAT, &(sensors->SensorsSemaphore));
	registers->AddRegisterPointer<float>(REG_VBAT, &VBAT);
	registers->SetRegisterPermissions(REG_VBAT, READ_PERMISSION);

	// Register of the battery current
	IBAT = 0;
	registers->AddRegister<float>(REG_IBAT);
	registers->SetRegisterAsSingle(REG_IBAT);
	registers->AddRegisterSemaphore(REG_IBAT, &(sensors->SensorsSemaphore));
	registers->AddRegisterPointer<float>(REG_IBAT, &IBAT);
	registers->SetRegisterPermissions(REG_IBAT, READ_PERMISSION);

	// Register of the battery protection unit temperature
	TBAT = 0;
	registers->AddRegister<float>(REG_TBAT);
	registers->SetRegisterAsSingle(REG_TBAT);
	registers->AddRegisterSemaphore(REG_TBAT, &(sensors->SensorsSemaphore));
	registers->AddRegisterPointer<float>(REG_TBAT, &TBAT);
	registers->SetRegisterPermissions(REG_TBAT, READ_PERMISSION);
}

/**
 * @brief Add battery sensor and setup reading sequences
 */
void PlatformSensors::AddBatterySensor(void) {
	// Add sensor group
	sensors->AddSensorGroup(I2C2_ID, BATTERY, true);

	// Set group prescaler
	sensors->SetSensorGroupPrescaler(BATTERY, BATTERY_PRESCALER);

	// Add sensor
	sensors->AddSensor(BATTERY, BATTERY_PROTECTION_DS2778, DS2778_ADDRESS);

	// Add reading of first cell raw voltage
	sensors->AddSensorData<uint16_t>(BATTERY_PROTECTION_DS2778, REG_CELL1_RAW, true);
	sensors->AddMemoryReadSequence(BATTERY_PROTECTION_DS2778, REG_CELL1_RAW, DS2778_VOLTAGE_V1_MSB, 2);
	sensors->AddCodeSequence<uint16_t>(BATTERY_PROTECTION_DS2778, REG_CELL1_RAW, (void*) registers,
		  [](void* context) -> uint16_t {
			  Registers* self = (Registers*) context;
			  uint16_t length;
			  uint16_t cell1_raw;
			  self->ReadRegister<uint16_t>(REG_CELL1_RAW, &cell1_raw, &length, false);
			  return (cell1_raw >> 5);
		  }
	);

    // Add reading of second cell raw voltage
	sensors->AddSensorData<uint16_t>(BATTERY_PROTECTION_DS2778, REG_CELL2_RAW, true);
	sensors->AddMemoryReadSequence(BATTERY_PROTECTION_DS2778, REG_CELL2_RAW, DS2778_VOLTAGE_V2_MSB, 2);
	sensors->AddCodeSequence<uint16_t>(BATTERY_PROTECTION_DS2778, REG_CELL2_RAW, (void*) registers,
		  [](void* context) -> uint16_t {
			  Registers* self = (Registers*) context;
			  uint16_t cell2_raw;
			  uint16_t length;
			  self->ReadRegister<uint16_t>(REG_CELL2_RAW, &cell2_raw, &length, false);
			  return (cell2_raw >> 5);
		  }
	);

    // Add reading of battery raw current
	sensors->AddSensorData<uint16_t>(BATTERY_PROTECTION_DS2778, REG_IBAT_RAW, true);
	sensors->AddMemoryReadSequence(BATTERY_PROTECTION_DS2778, REG_IBAT_RAW, DS2778_CURRENT_MSB, 2);

    // Add reading of battery protection unit raw temperature
	sensors->AddSensorData<uint16_t>(BATTERY_PROTECTION_DS2778, REG_TBAT_RAW, true);
	sensors->AddMemoryReadSequence(BATTERY_PROTECTION_DS2778, REG_TBAT_RAW, DS2778_TEMPERATURE_MSB, 2);
	sensors->AddCodeSequence<uint16_t>(BATTERY_PROTECTION_DS2778, REG_TBAT_RAW, (void*) registers,
		  [](void* context) -> uint16_t {
			  Registers* self = (Registers*) context;
			  uint16_t TBAT_raw;
			  uint16_t length;
			  self->ReadRegister<uint16_t>(REG_TBAT_RAW, &TBAT_raw, &length, false);
			  return (TBAT_raw >> 5);
		  }
	);

	// Add sensor callback
	sensors->AddSensorCallback(BATTERY_PROTECTION_DS2778, (void*) this,
		  [](void* context) -> void {
			  PlatformSensors* self = (PlatformSensors*) context;
			  bool success;
			  uint16_t length;

			  // Convert the first cell voltage
			  if (self->sensors->IsSensorDataActive(BATTERY_PROTECTION_DS2778, REG_CELL1_RAW)) {
				  uint16_t cell1_raw;
				  success = self->registers->ReadRegister<uint16_t>(REG_CELL1_RAW, &cell1_raw, &length, false); // LSB UNITS: 4.883mV
				  if (success)
					 self->Vcell1 = cell1_raw * 0.004883f;
			  }

			  // Convert the second cell voltage
			  if (self->sensors->IsSensorDataActive(BATTERY_PROTECTION_DS2778, REG_CELL2_RAW)) {
				  uint16_t cell2_raw;
				  success = self->registers->ReadRegister<uint16_t>(REG_CELL2_RAW, &cell2_raw, &length, false); // LSB UNITS: 4.883mV
				  if (success)
					 self->Vcell2 = cell2_raw * 0.004883f;
			  }

			  // Induce battery voltage
			  if (self->sensors->IsSensorDataActive(BATTERY_PROTECTION_DS2778, REG_CELL1_RAW) && self->sensors->IsSensorDataActive(BATTERY_PROTECTION_DS2778, REG_CELL2_RAW)) {
				  self->VBAT = self->Vcell1 + self->Vcell2;
				  if (self->VBAT < self->voltage_low_threshold) // low voltage detection
					  VoltageLowLEDS(self->leds);
			  }

			  // Convert battery current
			  if (self->sensors->IsSensorDataActive(BATTERY_PROTECTION_DS2778, REG_IBAT_RAW)) {
				  uint16_t IBAT_raw;
				  success = self->registers->ReadRegister<uint16_t>(REG_IBAT_RAW, &IBAT_raw, &length, false); // LSB UNITS: 1.5625μV/RSNS, RSNS=5mR so LSB=0.3125mA
				  if (success) {
					  self->IBAT = (int16_t) IBAT_raw * 0.0003125f;
				  }
			  }

			  // Convert battery protection unit temperature
			  if (self->sensors->IsSensorDataActive(BATTERY_PROTECTION_DS2778, REG_TBAT_RAW)) {
				  uint16_t TBAT_raw;
				  success = self->registers->ReadRegister<uint16_t>(REG_TBAT_RAW, &TBAT_raw, &length, false); // LSB UNITS: 0.125°C
				  if (success)
					 self->TBAT = TBAT_raw * 0.125;
			  }
		  }
    );
}

/**
 * @brief Activate battery sensor
 */
void PlatformSensors::ActivateBatterySensor(void) {
	// Activate sensor data
#ifdef MEAS_CELL1_RAW
	sensors->ActivateSensorData(BATTERY_PROTECTION_DS2778, REG_CELL1_RAW);
#endif
#ifdef MEAS_CELL2_RAW
	sensors->ActivateSensorData(BATTERY_PROTECTION_DS2778, REG_CELL2_RAW);
#endif
#ifdef MEAS_IBAT_RAW
	sensors->ActivateSensorData(BATTERY_PROTECTION_DS2778, REG_IBAT_RAW);
#endif
#ifdef MEAS_TBAT_RAW
	sensors->ActivateSensorData(BATTERY_PROTECTION_DS2778, REG_TBAT_RAW);
#endif

	// Activate sensor
#ifdef BATTERY_PROTECTION_DS2778_ACTIVE
	sensors->ActivateSensor(BATTERY_PROTECTION_DS2778);
#endif

#ifdef BATTERY_ACTIVE
	// Activate group
	sensors->ActivateSensorGroup(BATTERY);
#endif
}

/**
 * @brief Setup battery sensor publisher
 *
 * @param input interface_ID: interface to publish data on
 */
void PlatformSensors::SetupBatterySensorPublisher(void) {
	// Link interface to publisher
	publishers->LinkToInterface(BATTERY, PUBLISHER_BATTERY_INTERFACE);

	// Setup publisher
	publishers->SetPublisherPrescaler(BATTERY, PUBLISHER_BATTERY_PRESCALER);
	publishers->SetPublishAddress(BATTERY, PUBLISHER_BATTERY_INTERFACE, PUBLISHER_BATTERY_ADDRESS);

	// Add topics
	publishers->AddTopic(BATTERY, REG_VCELL1);
	publishers->AddTopic(BATTERY, REG_VCELL2);
	publishers->AddTopic(BATTERY, REG_VBAT);
	publishers->AddTopic(BATTERY, REG_IBAT);
	publishers->AddTopic(BATTERY, REG_TBAT);

	// Activate topics
#ifdef PUBLISH_BATTERY_TIMEBASE
	publishers->ActivateTopic(BATTERY, REG_TIMEBASE);
#endif
#ifdef PUBLISH_BATTERY_CELL1_RAW
	publishers->ActivateTopic(BATTERY, REG_CELL1_RAW);
#endif
#ifdef PUBLISH_BATTERY_CELL2_RAW
	publishers->ActivateTopic(BATTERY, REG_CELL2_RAW);
#endif
#ifdef PUBLISH_BATTERY_IBAT_RAW
	publishers->ActivateTopic(BATTERY, REG_IBAT_RAW);
#endif
#ifdef PUBLISH_BATTERY_VCELL1
	publishers->ActivateTopic(BATTERY, REG_VCELL1);
#endif
#ifdef PUBLISH_BATTERY_VCELL2
	publishers->ActivateTopic(BATTERY, REG_VCELL2);
#endif
#ifdef PUBLISH_BATTERY_VBAT
	publishers->ActivateTopic(BATTERY, REG_VBAT);
#endif
#ifdef PUBLISH_BATTERY_IBAT
	publishers->ActivateTopic(BATTERY, REG_IBAT);
#endif
#ifdef PUBLISH_BATTERY_TBAT
	publishers->ActivateTopic(BATTERY, REG_TBAT);
#endif

#ifdef PUBLISHER_BATTERY_ACTIVE
	// Activate publisher
	publishers->ActivatePublisher(BATTERY);
#endif
}

/**
 * @brief Add motor sensor registers
 */
void PlatformSensors::AddMotorSensorRegisters(void) {
	// Register of the motor voltage
	Vmotor = 0;
	registers->AddRegister<float>(REG_MOTOR_VOLTAGE);
	registers->SetRegisterAsSingle(REG_MOTOR_VOLTAGE);
	registers->AddRegisterSemaphore(REG_MOTOR_VOLTAGE, &(sensors->SensorsSemaphore));
	registers->AddRegisterPointer<float>(REG_MOTOR_VOLTAGE, &Vmotor);
	registers->SetRegisterPermissions(REG_MOTOR_VOLTAGE, READ_PERMISSION);

	// Register of the motor current
	Imotor = 0;
	registers->AddRegister<float>(REG_MOTOR_CURRENT);
	registers->SetRegisterAsSingle(REG_MOTOR_CURRENT);
	registers->AddRegisterSemaphore(REG_MOTOR_CURRENT, &(sensors->SensorsSemaphore));
	registers->AddRegisterPointer<float>(REG_MOTOR_CURRENT, &Imotor);
	registers->SetRegisterPermissions(REG_MOTOR_CURRENT, READ_PERMISSION);

	// Register of the motor power
	Pmotor = 0;
	registers->AddRegister<float>(REG_MOTOR_POWER);
	registers->SetRegisterAsSingle(REG_MOTOR_POWER);
	registers->AddRegisterSemaphore(REG_MOTOR_POWER, &(sensors->SensorsSemaphore));
	registers->AddRegisterPointer<float>(REG_MOTOR_POWER, &Pmotor);
	registers->SetRegisterPermissions(REG_MOTOR_POWER, READ_PERMISSION);

	// Register of the motor energy double
	Emotor = 0;
	registers->AddRegister<double>(REG_MOTOR_ENERGY_DOUBLE);
	registers->SetRegisterAsSingle(REG_MOTOR_ENERGY_DOUBLE);
	registers->AddRegisterSemaphore(REG_MOTOR_ENERGY_DOUBLE, &(sensors->SensorsSemaphore));
	registers->AddRegisterPointer<double>(REG_MOTOR_ENERGY_DOUBLE, &Emotor);
	registers->SetRegisterPermissions(REG_MOTOR_ENERGY_DOUBLE, READ_PERMISSION);

	// Register of the motor energy recast in float
	Emotor_float = 0;
	registers->AddRegister<float>(REG_MOTOR_ENERGY_FLOAT);
	registers->SetRegisterAsSingle(REG_MOTOR_ENERGY_FLOAT);
	registers->AddRegisterSemaphore(REG_MOTOR_ENERGY_FLOAT, &(sensors->SensorsSemaphore));
	registers->AddRegisterPointer<float>(REG_MOTOR_ENERGY_FLOAT, &Emotor_float);
	registers->SetRegisterPermissions(REG_MOTOR_ENERGY_FLOAT, READ_PERMISSION);

	// Register of the sensor temperature
	Tmotor = 0;
	registers->AddRegister<float>(REG_MOTOR_TEMPERATURE);
	registers->SetRegisterAsSingle(REG_MOTOR_TEMPERATURE);
	registers->AddRegisterSemaphore(REG_MOTOR_TEMPERATURE, &(sensors->SensorsSemaphore));
	registers->AddRegisterPointer<float>(REG_MOTOR_TEMPERATURE, &Tmotor);
	registers->SetRegisterPermissions(REG_MOTOR_TEMPERATURE, READ_PERMISSION);
}

/**
 * @brief Add motor sensor and setup reading sequences
 */
void PlatformSensors::AddMotorSensor(void) {
	// Add sensor group
	sensors->AddSensorGroup(I2C2_ID, MOTOR, true);

	// Set group prescaler
	sensors->SetSensorGroupPrescaler(MOTOR, MOTOR_PRESCALER);

	// Add sensor
	sensors->AddSensor(MOTOR, ENERGY_MONITORING_LTC2947, LTC2947_ADDRESS);

	// Add reading of motor raw voltage
	sensors->AddSensorData<uint16_t>(ENERGY_MONITORING_LTC2947, REG_MOTOR_VOLTAGE_RAW, true);
	sensors->AddMemoryReadSequence(ENERGY_MONITORING_LTC2947, REG_MOTOR_VOLTAGE_RAW, LTC2947_V, LTC2947_V_SIZE);

	// Add reading of motor raw current
	sensors->AddSensorData<uint32_t>(ENERGY_MONITORING_LTC2947, REG_MOTOR_CURRENT_RAW, true);
	sensors->AddMemoryReadSequence(ENERGY_MONITORING_LTC2947, REG_MOTOR_CURRENT_RAW, LTC2947_I, LTC2947_I_SIZE);

	// Add reading of motor raw power
	sensors->AddSensorData<uint32_t>(ENERGY_MONITORING_LTC2947, REG_MOTOR_POWER_RAW, true);
	sensors->AddMemoryReadSequence(ENERGY_MONITORING_LTC2947, REG_MOTOR_POWER_RAW, LTC2947_P, LTC2947_P_SIZE);

	// Add reading of motor raw energy
	sensors->AddSensorData<uint64_t>(ENERGY_MONITORING_LTC2947, REG_MOTOR_ENERGY_RAW, true);
	sensors->AddMemoryReadSequence(ENERGY_MONITORING_LTC2947, REG_MOTOR_ENERGY_RAW, LTC2947_E1, LTC2947_ENERGY1_SIZE);

	// Add reading of sensor raw temperature
	sensors->AddSensorData<uint16_t>(ENERGY_MONITORING_LTC2947, REG_MOTOR_TEMPERATURE_RAW, true);
	sensors->AddMemoryReadSequence(ENERGY_MONITORING_LTC2947, REG_MOTOR_TEMPERATURE_RAW, LTC2947_TEMP, LTC2947_TEMP_SIZE);

	// Add sensor callback
	sensors->AddSensorCallback(ENERGY_MONITORING_LTC2947, (void*) this,
		  [](void* context) -> void {
			  PlatformSensors* self = (PlatformSensors*) context;
			  bool success;
			  uint16_t length;

			  // Convert motor voltage
			  if (self->sensors->IsSensorDataActive(ENERGY_MONITORING_LTC2947, REG_MOTOR_VOLTAGE_RAW)) {
				  uint16_t V_raw;
				  success = self->registers->ReadRegister<uint16_t>(REG_MOTOR_VOLTAGE_RAW, &V_raw, &length, false);
				  if (success)
					 self->Vmotor = V_raw * 0.002f; // LSB = 2mV
			  }

			  // Convert motor current
			  if (self->sensors->IsSensorDataActive(ENERGY_MONITORING_LTC2947, REG_MOTOR_CURRENT_RAW)) {
				  uint32_t I_raw;
				  success = self->registers->ReadRegister<uint32_t>(REG_MOTOR_CURRENT_RAW, &I_raw, &length, false);
				  if (success)
					 self->Imotor = I_raw * 0.003f * 0.999f; // LSB = 3mA + correction factor of 0.999 for a 60um copper thickness PCB
			  }

			  // Convert motor power
			  if (self->sensors->IsSensorDataActive(ENERGY_MONITORING_LTC2947, REG_MOTOR_POWER_RAW)) {
				  uint32_t P_raw;
				  success = self->registers->ReadRegister<uint32_t>(REG_MOTOR_POWER_RAW, &P_raw, &length, false);
				  if (success)
					 self->Pmotor = P_raw * 0.05f * 0.999f; // LSB = 50mW + correction factor of 0.999 for a 60um copper thickness PCB
			  }

			  // Convert motor energy
			  if (self->sensors->IsSensorDataActive(ENERGY_MONITORING_LTC2947, REG_MOTOR_ENERGY_RAW)) {
				  uint64_t E_raw;
				  success = self->registers->ReadRegister<uint64_t>(REG_MOTOR_ENERGY_RAW, &E_raw, &length, false);
				  if (success) {
					 self->Emotor = E_raw * 0.00001989f * 0.999f; // LSB = 19.89E-06 Ws or 0.6416 * 1/fEXT * 2^PRE * (DIV+1) [Ws] + correction factor of 0.999 for a 60um copper thickness PCB
				  	 self->Emotor_float = (float) self->Emotor;
				  }
			  }

			  // Convert sensor temperature
			  if (self->sensors->IsSensorDataActive(ENERGY_MONITORING_LTC2947, REG_MOTOR_TEMPERATURE_RAW)) {
				  uint16_t T_raw;
				  success = self->registers->ReadRegister<uint16_t>(REG_MOTOR_TEMPERATURE_RAW, &T_raw, &length, false);
				  if (success)
					 self->Tmotor = T_raw * 0.204f + 5.5; // in °C
			  }
		  }
	);
}

/**
 * @brief Activate motor sensor
 */
void PlatformSensors::ActivateMotorSensor(void) {
	// Activate sensor data
#ifdef MEAS_MOTOR_VOLTAGE_RAW
	sensors->ActivateSensorData(ENERGY_MONITORING_LTC2947, REG_MOTOR_VOLTAGE_RAW);
#endif
#ifdef MEAS_MOTOR_CURRENT_RAW
	sensors->ActivateSensorData(ENERGY_MONITORING_LTC2947, REG_MOTOR_CURRENT_RAW);
#endif
#ifdef MEAS_MOTOR_POWER_RAW
	sensors->ActivateSensorData(ENERGY_MONITORING_LTC2947, REG_MOTOR_POWER_RAW);
#endif
#ifdef MEAS_MOTOR_ENERGY_RAW
	sensors->ActivateSensorData(ENERGY_MONITORING_LTC2947, REG_MOTOR_ENERGY_RAW);
#endif
#ifdef MEAS_MOTOR_TEMPERATURE_RAW
	sensors->ActivateSensorData(ENERGY_MONITORING_LTC2947, REG_MOTOR_TEMPERATURE_RAW);
#endif

	// Activate sensor
#ifdef ENERGY_MONITORING_LTC2947_ACTIVE
	sensors->ActivateSensor(ENERGY_MONITORING_LTC2947);
#endif

#ifdef MOTOR_ACTIVE
	// Activate group
	sensors->ActivateSensorGroup(MOTOR);
#endif
}

/**
 * @brief Setup motor sensor publisher
 *
 * @param input interface_ID: interface to publish data on
 */
void PlatformSensors::SetupMotorSensorPublisher(void) {
	// Link interface to publisher
	publishers->LinkToInterface(MOTOR, PUBLISHER_MOTOR_INTERFACE);

	// Setup publisher
	publishers->SetPublisherPrescaler(MOTOR, PUBLISHER_MOTOR_PRESCALER);
	publishers->SetPublishAddress(MOTOR, PUBLISHER_MOTOR_INTERFACE, PUBLISHER_MOTOR_ADDRESS);

	// Add topics
	publishers->AddTopic(MOTOR, REG_MOTOR_VOLTAGE);
	publishers->AddTopic(MOTOR, REG_MOTOR_CURRENT);
	publishers->AddTopic(MOTOR, REG_MOTOR_POWER);
	publishers->AddTopic(MOTOR, REG_MOTOR_ENERGY_DOUBLE);
	publishers->AddTopic(MOTOR, REG_MOTOR_ENERGY_FLOAT);
	publishers->AddTopic(MOTOR, REG_MOTOR_TEMPERATURE);

	// Activate topics
#ifdef PUBLISH_MOTOR_TIMEBASE
	publishers->ActivateTopic(MOTOR, REG_TIMEBASE);
#endif
#ifdef PUBLISH_MOTOR_VOLTAGE_RAW
	publishers->ActivateTopic(MOTOR, REG_MOTOR_VOLTAGE_RAW);
#endif
#ifdef PUBLISH_MOTOR_CURRENT_RAW
	publishers->ActivateTopic(MOTOR, REG_MOTOR_CURRENT_RAW);
#endif
#ifdef PUBLISH_MOTOR_POWER_RAW
	publishers->ActivateTopic(MOTOR, REG_MOTOR_POWER_RAW);
#endif
#ifdef PUBLISH_MOTOR_ENERGY_RAW
	publishers->ActivateTopic(MOTOR, REG_MOTOR_ENERGY_RAW);
#endif
#ifdef PUBLISH_MOTOR_TEMPERATURE_RAW
	publishers->ActivateTopic(MOTOR, REG_MOTOR_TEMPERATURE_RAW);
#endif
#ifdef PUBLISH_MOTOR_VOLTAGE
	publishers->ActivateTopic(MOTOR, REG_MOTOR_VOLTAGE);
#endif
#ifdef PUBLISH_MOTOR_CURRENT
	publishers->ActivateTopic(MOTOR, REG_MOTOR_CURRENT);
#endif
#ifdef PUBLISH_MOTOR_POWER
	publishers->ActivateTopic(MOTOR, REG_MOTOR_POWER);
#endif
#ifdef PUBLISH_MOTOR_ENERGY_DOUBLE
	publishers->ActivateTopic(MOTOR, REG_MOTOR_ENERGY_DOUBLE);
#endif
#ifdef PUBLISH_MOTOR_ENERGY_FLOAT
	publishers->ActivateTopic(MOTOR, REG_MOTOR_ENERGY_FLOAT);
#endif
#ifdef PUBLISH_MOTOR_TEMPERATURE
	publishers->ActivateTopic(MOTOR, REG_MOTOR_TEMPERATURE);
#endif

#ifdef PUBLISHER_MOTOR_ACTIVE
	// Activate publisher
	publishers->ActivatePublisher(MOTOR);
#endif
}

/**
 * @brief Configure motor sensor
 */
void PlatformSensors::ConfigureMotorSensor(void) {
	// Reset the sensor
	sensors->ConfigureSensor(ENERGY_MONITORING_LTC2947, LTC2947_OPCTL, LTC2947_OPCTL_RST, HAL_MAX_DELAY);

	HAL_Delay(100);

	// Configure sensor
//	sensors->ConfigureSensor(ENERGY_MONITORING_LTC2947, LTC2947_TBCTL, 0xF2, HAL_MAX_DELAY);					// use external oscillator
	sensors->ConfigureSensor(ENERGY_MONITORING_LTC2947, LTC2947_OPCTL, LTC2947_OPCTL_CONT, HAL_MAX_DELAY); 	// continuous reading
}

/**
 * @brief Add voltage sensor and setup reading sequence
 */
void PlatformSensors::AddVoltageSensor(ADC_HandleTypeDef* ADC3_3V_, ADC_HandleTypeDef* ADC5V_) {
	ADC3_3V = ADC3_3V_;
	ADC5V = ADC5V_;

	// Setup voltage constants
	voltage_low_threshold = 7.0f;
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
