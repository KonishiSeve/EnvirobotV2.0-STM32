/*
 * Sensors.cpp
 *
 *  Created on: 14 déc. 2022
 *      Author: bignet
 */

#include <Sensors/Sensors.hpp>

/**
 * @brief Class constructor
 */
Sensors::Sensors() {
#if (defined(USE_UINT8_I2C_COMMUNICATION) || defined(USE_UINT8_SENSOR_FUNCTION)) && defined(USE_UINT8_REGISTER)
	sensor_data_uint8.reserve(SENSOR_MAX_REGISTER_NB_PER_TYPE);
#endif
#if (defined(USE_UINT16_I2C_COMMUNICATION) || defined(USE_UINT16_SENSOR_FUNCTION)) && defined(USE_UINT16_REGISTER)
	sensor_data_uint16.reserve(SENSOR_MAX_REGISTER_NB_PER_TYPE);
#endif
#if (defined(USE_UINT32_I2C_COMMUNICATION) || defined(USE_UINT32_SENSOR_FUNCTION)) && defined(USE_UINT32_REGISTER)
	sensor_data_uint32.reserve(SENSOR_MAX_REGISTER_NB_PER_TYPE);
#endif
#if (defined(USE_UINT64_I2C_COMMUNICATION) || defined(USE_UINT64_SENSOR_FUNCTION)) && defined(USE_UINT64_REGISTER)
	sensor_data_uint64.reserve(SENSOR_MAX_REGISTER_NB_PER_TYPE);
#endif
#if (defined(USE_INT8_I2C_COMMUNICATION) || defined(USE_INT8_SENSOR_FUNCTION)) && defined(USE_INT8_REGISTER)
	sensor_data_int8.reserve(SENSOR_MAX_REGISTER_NB_PER_TYPE);
#endif
#if (defined(USE_INT16_I2C_COMMUNICATION) || defined(USE_INT16_SENSOR_FUNCTION)) && defined(USE_INT16_REGISTER)
	sensor_data_int16.reserve(SENSOR_MAX_REGISTER_NB_PER_TYPE);
#endif
#if (defined(USE_INT32_I2C_COMMUNICATION) || defined(USE_INT32_SENSOR_FUNCTION)) && defined(USE_INT32_REGISTER)
	sensor_data_int32.reserve(SENSOR_MAX_REGISTER_NB_PER_TYPE);
#endif
#if (defined(USE_INT64_I2C_COMMUNICATION) || defined(USE_INT64_SENSOR_FUNCTION)) && defined(USE_INT64_REGISTER)
	sensor_data_int64.reserve(SENSOR_MAX_REGISTER_NB_PER_TYPE);
#endif
#if (defined(USE_FLOAT_I2C_COMMUNICATION) || defined(USE_FLOAT_SENSOR_FUNCTION)) && defined(USE_FLOAT_REGISTER)
	sensor_data_float.reserve(SENSOR_MAX_REGISTER_NB_PER_TYPE);
#endif
#if (defined(USE_DOUBLE_I2C_COMMUNICATION) || defined(USE_DOUBLE_SENSOR_FUNCTION)) && defined(USE_DOUBLE_REGISTER)
	sensor_data_double.reserve(SENSOR_MAX_REGISTER_NB_PER_TYPE);
#endif

	functions[UINT8_TYPE] = NULL;
	functions[UINT16_TYPE] = NULL;
	functions[UINT32_TYPE] = NULL;
	functions[UINT64_TYPE] = NULL;
	functions[INT8_TYPE] = NULL;
	functions[INT16_TYPE] = NULL;
	functions[INT32_TYPE] = NULL;
	functions[INT64_TYPE] = NULL;
	functions[FLOAT_TYPE] = NULL;
	functions[DOUBLE_TYPE] = NULL;

#if defined(USE_UINT8_SENSOR_FUNCTION) && defined(USE_UINT8_REGISTER)
	functions[UINT8_TYPE] = &uint8_t_sensor_functions;
#endif
#if defined(USE_UINT16_SENSOR_FUNCTION) && defined(USE_UINT16_REGISTER)
	functions[UINT16_TYPE] = &uint16_t_sensor_functions;
#endif
#if defined(USE_UINT32_SENSOR_FUNCTION) && defined(USE_UINT32_REGISTER)
	functions[UINT32_TYPE] = &uint32_t_sensor_functions;
#endif
#if defined(USE_UINT64_SENSOR_FUNCTION) && defined(USE_UINT64_REGISTER)
	functions[UINT64_TYPE] = &uint64_t_sensor_functions;
#endif
#if defined(USE_INT8_SENSOR_FUNCTION) && defined(USE_INT8_REGISTER)
	functions[INT8_TYPE] = &int8_t_sensor_functions;
#endif
#if defined(USE_INT16_SENSOR_FUNCTION) && defined(USE_INT16_REGISTER)
	functions[INT16_TYPE] = &int16_t_sensor_functions;
#endif
#if defined(USE_INT32_SENSOR_FUNCTION) && defined(USE_INT32_REGISTER)
	functions[INT32_TYPE] = &int32_t_sensor_functions;
#endif
#if defined(USE_INT64_SENSOR_FUNCTION) && defined(USE_INT64_REGISTER)
	functions[INT64_TYPE] = &int64_t_sensor_functions;
#endif
#if defined(USE_FLOAT_SENSOR_FUNCTION) && defined(USE_FLOAT_REGISTER)
	functions[FLOAT_TYPE] = &float_sensor_functions;
#endif
#if defined(USE_DOUBLE_SENSOR_FUNCTION) && defined(USE_DOUBLE_REGISTER)
	functions[DOUBLE_TYPE] = &double_sensor_functions;
#endif
}

/**
 * @brief Initialize the class with references to other classes. Set default values.
 *
 * @param input registers_: the Registers instance
 * @param input publishers_: the Publishers instance
 * @param input hardware_delay_: the HardwareDelay instance
 * @param input leds_: the LEDS instance
 * @param input ADC3_3V_: the ADC used for 3.3V sensing
 * @param input ADC5V_: the ADC used for 5V sensing
 */
void Sensors::Init(Registers* registers_, Publishers* publishers_, HardwareDelay* hardware_delay_, LEDS* leds_) {
	registers = registers_;
	publishers = publishers_;
	hardware_delay = hardware_delay_;
	leds = leds_;

	SensorsSemaphore = osSemaphoreNew(1, 1, NULL);
	osSemaphoreRelease(SensorsSemaphore);

	active = false;
//	delay_flag = false;
	delayed_index = 0;
	period_ms = DEFAULT_SENSORS_PERIOD;
}

/**
 * @brief Add class related registers
 */
void Sensors::AddRegisters(void) {
	// Register to access the general sensor reading status
	registers->AddRegister<uint8_t>(REG_SENSORS_READING_STATUS);
	registers->SetRegisterAsSingle(REG_SENSORS_READING_STATUS);
	registers->AddReadCallback<uint8_t>(REG_SENSORS_READING_STATUS, (void*) this,
			[](void* context, uint16_t register_id, uint8_t** output, uint16_t* length) -> bool {
			Sensors* self = (Sensors*) context;
			osSemaphoreAcquire(self->SensorsSemaphore, osWaitForever);
			**output = (uint8_t) self->active;
			osSemaphoreRelease(self->SensorsSemaphore);
			return true;
		}
	);
	registers->AddWriteCallback<uint8_t>(REG_SENSORS_READING_STATUS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Sensors* self = (Sensors*) context;
			return self->SetReadingStatus((bool) *input);
		}
	);

	// Register to set the status of a sensor group
	registers->AddRegister<uint8_t>(REG_SENSORS_GROUP_STATUS);
	registers->SetRegisterAsArray(REG_SENSORS_GROUP_STATUS, 2);
	registers->AddWriteCallback<uint8_t>(REG_SENSORS_GROUP_STATUS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Sensors* self = (Sensors*) context;
			if (length != 2) return false;
			return self->SetSensorGroupStatus(input[0], (bool) input[1]);
		}
	);

	// Register to deactivate all sensor groups
	registers->AddRegister<uint8_t>(REG_SENSORS_DEACTIVATE_GROUPS);
	registers->SetRegisterAsSingle(REG_SENSORS_DEACTIVATE_GROUPS);
	registers->AddWriteCallback<uint8_t>(REG_SENSORS_DEACTIVATE_GROUPS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Sensors* self = (Sensors*) context;
			return self->DeactivateAllSensorGroups();
		}
	);

	// Register to set a sensor status
	registers->AddRegister<uint8_t>(REG_SENSORS_SENSOR_STATUS);
	registers->SetRegisterAsArray(REG_SENSORS_SENSOR_STATUS, 2);
	registers->AddWriteCallback<uint8_t>(REG_SENSORS_SENSOR_STATUS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Sensors* self = (Sensors*) context;
			if (length != 2) return false;
			return self->SetSensorStatus(input[0], (bool) input[1]);
		}
	);

	// Register to deactivate all sensors from a group
	registers->AddRegister<uint8_t>(REG_SENSORS_DEACTIVATE_SENSORS);
	registers->SetRegisterAsSingle(REG_SENSORS_DEACTIVATE_SENSORS);
	registers->AddWriteCallback<uint8_t>(REG_SENSORS_DEACTIVATE_SENSORS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Sensors* self = (Sensors*) context;
			return self->DeactivateAllSensors(*input);
		}
	);

	// Register to set a sensor data status
	registers->AddRegister<uint16_t>(REG_SENSORS_DATA_STATUS);
	registers->SetRegisterAsArray(REG_SENSORS_DATA_STATUS, 3);
	registers->AddWriteCallback<uint16_t>(REG_SENSORS_DATA_STATUS, (void*) this,
		[](void* context, uint16_t register_id, uint16_t* input, uint16_t length) -> bool {
			Sensors* self = (Sensors*) context;
			if (length != 3) return false;
			return self->SetSensorDataStatus((uint8_t) input[0], input[1], (bool) input[2]);
		}
	);

	// Register to deactivate all sensor data from a sensor
	registers->AddRegister<uint8_t>(REG_SENSORS_DEACTIVATE_ALL_DATA);
	registers->SetRegisterAsSingle(REG_SENSORS_DEACTIVATE_ALL_DATA);
	registers->AddWriteCallback<uint8_t>(REG_SENSORS_DEACTIVATE_ALL_DATA, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Sensors* self = (Sensors*) context;
			return self->DeactivateAllSensorData(*input);
		}
	);

	// Register to set the prescaler factor for a sensor group
	registers->AddRegister<uint16_t>(REG_SENSORS_GROUP_PRESCALER);
	registers->SetRegisterAsArray(REG_SENSORS_GROUP_PRESCALER, 2);
	registers->AddWriteCallback<uint16_t>(REG_SENSORS_GROUP_PRESCALER, (void*) this,
		[](void* context, uint16_t register_id, uint16_t* input, uint16_t length) -> bool {
			Sensors* self = (Sensors*) context;
			if (length != 2) return false;
			return self->SetSensorGroupPrescaler((uint8_t) input[0], input[1]);
		}
	);

	// Register to start I2C sensor detection
	registers->AddRegister<uint32_t>(REG_SENSORS_DETECT_SENSORS);
	registers->SetRegisterAsArray(REG_SENSORS_DETECT_SENSORS, 2);
	registers->AddWriteCallback<uint32_t>(REG_SENSORS_DETECT_SENSORS, (void*) this,
		[](void* context, uint16_t register_id, uint32_t* input, uint16_t length) -> bool {
			Sensors* self = (Sensors*) context;
			if (length != 2) return false;
			self->DetectSensors(input[0], input[1]);
			return true;
		}
	);
}

/**
 * @brief Add a communication interface
 *
 * @param input ID: interface ID
 * @param input interface: hardware interface handle
 * @return whether successful
 */
bool Sensors::AddInterface(uint8_t ID, I2C_HandleTypeDef* interface) {
	bool success;
	// Check interface is not already registered
	FindInterface(ID, &success);
	if (success) return false;

	// Setup interface
	I2CInterface I2Cinterface;
	I2Cinterface.ID = ID;
	I2Cinterface.request_regular_process = false;
	I2Cinterface.error_flag = false;
	I2Cinterface.timeout = UINT32_MAX;

	I2Cinterface.interface = interface;

	I2Cinterface.access_source = 0;
	I2Cinterface.read_index = 0;
	I2Cinterface.pending_index = 0;
	I2Cinterface.request_index = 0;

	I2Cinterface.pending_sensors.reserve(PENDING_SENSORS_SIZE);
	I2Cinterface.pending_requests.reserve(PENDING_SENSORS_SIZE);

	I2Cinterface.current_group_ID = UINT8_MAX;
	I2Cinterface.current_sensor_ID = UINT8_MAX;
	I2Cinterface.current_data_address = UINT16_MAX;

	// Add interface
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	interfaces.push_back(I2Cinterface);
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Set the I2C access timeout for an interface
 *
 * @param input ID: interface ID
 * @param input timeout: timeout in ms
 * @return whether successful
 */
bool Sensors::SetInterfaceTimeout(uint8_t ID, uint32_t timeout) {
	bool success;
	I2CInterface* interface = FindInterface(ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	interface->timeout = timeout;
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Set the reading status of the class
 *
 * @param input status: reading status. true = read sensors
 * @return whether successful
 */
bool Sensors::SetReadingStatus(bool status) {
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	active = status;

	// Reset all read indexes if reading is stopped to restart a new reading from zero
	if (!status) {
		for (I2CInterface &interface : interfaces) {
			interface.read_index = 0;
			for (SensorGroup &sensor_group : interface.sensor_groups) {
				sensor_group.read_index = 0;
				for (Sensor &sensor : sensor_group.sensors) {
					sensor.read_index = 0;
				}
			}
		}
	}

	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Activate reading of sensors
 *
 * @return whether successful
 */
bool Sensors::ActivateReading() {
	return SetReadingStatus(true);
}

/**
 * @brief Deactivate reading of sensors
 *
 * @return whether successful
 */
bool Sensors::DeactivateReading() {
	return SetReadingStatus(false);
}

/**
 * @brief Add a sensor group to an interface
 *
 * @param input ID: interface ID
 * @param input group_ID: group ID to be created and linked to interface
 * @param input publishable: whether the sensor group is linked to a publisher. If true, a publisher is automatically generated with the group ID. DEFAULT=true
 * @return whether successful
 */
bool Sensors::AddSensorGroup(uint8_t ID, uint8_t group_ID, bool publishable) {
	bool success;
	// Find interface
	I2CInterface* interface = FindInterface(ID, &success);
	if (!success) return false;

	// Check group doesn't already exist
	FindSensorGroup(group_ID, &success);
	if (success) return false;

	// Setup sensor group
	SensorGroup sensor_group;
	sensor_group.parent = interface->ID; // interface
	sensor_group.active = false;
	sensor_group.ID = group_ID;
	sensor_group.publishable = publishable;
	sensor_group.prescaler = 1;
	sensor_group.read_index = 0;
	sensor_group.counter = 0;

	if (publishable) {
		// Add a publisher if publishable
		success = publishers->AddPublisher(group_ID);
		if (!success) return false;  								// TODO Or continue? Simply don't add the topic ?
		// Add the timebase as a default topic of the publisher
		success = publishers->AddTopic(group_ID, REG_TIMEBASE);
		if (!success) return false;  								// TODO Or continue, Simply don't add the topic ?
	}

	// Add sensor group
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	interface->sensor_groups.push_back(sensor_group);
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Set the a sensor group status
 *
 * @param input group_ID: group ID
 * @param input status: status. true=active
 * @return whether successful
 */
bool Sensors::SetSensorGroupStatus(uint8_t group_ID, bool status) {
	bool success;
	SensorGroup* sensor_group = FindSensorGroup(group_ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	sensor_group->active = status;

	// reset the read indexes if the group is deactivated
	if (!status) {
		sensor_group->read_index = 0;
		for (Sensor &sensor : sensor_group->sensors) {
			sensor.read_index = 0;
		}
	}
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Activate a sensor group
 *
 * @return whether successful
 */
bool Sensors::ActivateSensorGroup(uint8_t group_ID) {
	return SetSensorGroupStatus(group_ID, true);
}

/**
 * @brief Deactivate a sensor group
 *
 * @return whether successful
 */
bool Sensors::DeactivateSensorGroup(uint8_t group_ID) {
	return SetSensorGroupStatus(group_ID, false);
}

/**
 * @brief Deactivate all sensor groups on all interfaces
 *
 * @return whether successful
 */
bool Sensors::DeactivateAllSensorGroups(void) {
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	for (I2CInterface &interface : interfaces) {
		for (SensorGroup &sensor_group : interface.sensor_groups) {
			sensor_group.active = false;
		}
	}
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Set the prescaler of a sensor group
 *
 * @param input group_ID: group ID
 * @param input prescaler: prescaler value with respect to the Spin period
 * @return whether successful
 */
bool Sensors::SetSensorGroupPrescaler(uint8_t group_ID, uint16_t prescaler) {
	bool success;
	SensorGroup* sensor_group = FindSensorGroup(group_ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	sensor_group->prescaler = prescaler;
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Add a sensor to a sensor group
 *
 * @param input group_ID: group ID
 * @param input sensor_ID: sensor ID to be added to the group ID. Should be unique so can't be similar to the ID from another group
 * @param input address: I2C address of the I2C sensor. Set it to any value if the sensor embeds no I2C access but only code sequences
 * @return whether successful
 */
bool Sensors::AddSensor(uint8_t group_ID, uint8_t sensor_ID, uint16_t address) {
	bool success;
	// Find interface and sensor group
	I2CInterface* interface;
	SensorGroup* sensor_group = FindSensorGroupPath(group_ID, &interface, &success);
	if (!success) return false;

	// Check no sensor is already registered in any group
	FindSensor(sensor_ID, &success);
	if (success) return false;

	// Setup sensor
	Sensor sensor;
	sensor.active = false;

	// Sensor automatic detection so a sensor can be added but automatically discarded if not present
	if (interface->ID != INTERNAL) {
		sensor.present = (HAL_I2C_IsDeviceReady(interface->interface, address, 1, HAL_MAX_DELAY) == HAL_OK);
	} else {
		sensor.present = true;
	}

	sensor.processing = false;
	sensor.parent = sensor_group->ID;
	sensor.ID = sensor_ID;
	sensor.address = address;
	sensor.memory_address_size = 1;
	sensor.read_index = 0;
	sensor.context = NULL;
	sensor.callback = NULL;

	// Add sensor
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	sensor_group->sensors.push_back(sensor);
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Add a sensor callback which is run after reading all measures
 *
 * @param input sensor_ID: sensor ID
 * @param input context: context to be forwarded to the callback
 * @param input callback: callback function to run after reading sensor measures
 * @return whether successful
 */
bool Sensors::AddSensorCallback(uint8_t sensor_ID, void* context, void (*callback)(void*)) {
	bool success;
	Sensor* sensor = FindSensor(sensor_ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	sensor->context = context;
	sensor->callback = callback;
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Set the sensor status
 *
 * @param input sensor_ID: sensor ID
 * @param input status: status. true=active
 * @return whether successful
 */
bool Sensors::SetSensorStatus(uint8_t sensor_ID, bool status) {
	bool success;
	Sensor* sensor = FindSensor(sensor_ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	sensor->active = status;
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Activate a sensor
 *
 * @param input sensor_ID: sensor ID
 * @return whether successful
 */
bool Sensors::ActivateSensor(uint8_t sensor_ID) {
	return SetSensorStatus(sensor_ID, true);
}

/**
 * @brief Deactivate a sensor
 *
 * @param input sensor_ID: sensor ID
 * @return whether successful
 */
bool Sensors::DeactivateSensor(uint8_t sensor_ID) {
	return SetSensorStatus(sensor_ID, false);
}

/**
 * @brief Deactivate all sensors from a group
 *
 * @param input group_ID: group ID
 * @return whether successful
 */
bool Sensors::DeactivateAllSensors(uint8_t group_ID) {
	bool success;
	I2CInterface* interface;
	SensorGroup* sensor_group = FindSensorGroupPath(group_ID, &interface, &success);
	if (!success) return false;

	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	for (Sensor &sensor : sensor_group->sensors) {
		sensor.active = false;
	}
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Set the memory address size of the sensor in bytes. It is set to 1 byte by default at sensor creation. This is not the sensor address size.
 *
 * @param input sensor_ID: sensor ID
 * @param input memory_address_size: memory address size of the sensor in bytes
 * @return whether successful
 */
bool Sensors::SetSensorMemoryAddressSize(uint8_t sensor_ID, uint16_t memory_address_size) {
	bool success;
	Sensor* sensor = FindSensor(sensor_ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	sensor->memory_address_size = memory_address_size;
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief I2C write of 1 byte to the sensor for configuration purpose. BLOCKING function
 *
 * @param input sensor_ID: sensor ID
 * @param input data: single byte to send
 * @param input timeout: time left to configure the sensor in ms. DEFAULT=HAL_MAX_DELAY
 * @return the I2C access status
 */
HAL_StatusTypeDef Sensors::ConfigureSensor(uint8_t sensor_ID, uint8_t data, uint32_t timeout) {
	uint8_t buffer = data;
	return ConfigureSensor(sensor_ID, &buffer, 1, timeout);
}

/**
 * @brief I2C write multiple bytes to the sensor for configuration purpose. BLOCKING function
 *
 * @param input sensor_ID: sensor ID
 * @param input data: byte array to send
 * @param input size: data size in bytes
 * @param input timeout: time left to configure the sensor in ms. DEFAULT=HAL_MAX_DELAY
 * @return the I2C access status
 */
HAL_StatusTypeDef Sensors::ConfigureSensor(uint8_t sensor_ID, uint8_t* data, uint16_t size, uint32_t timeout) {
	bool success;
	// Find sensor
	Sensor* sensor = FindSensor(sensor_ID, &success);
	if (!success) return HAL_ERROR;

	// Check sensor has been detected and is present
	if (!sensor->present) return HAL_ERROR;

	// Find interface and sensor group
	SensorGroup* sensor_group = FindSensorGroup(sensor->parent, &success);
	if (!success) return HAL_ERROR;
	I2CInterface* interface = FindInterface(sensor_group->parent, &success);
	if (!success) return HAL_ERROR;

	// Compute end timestamp for timeout
	uint32_t end_timestamp = HAL_GetTick() + timeout;
	HAL_StatusTypeDef status;

	// Write to the sensor until I2C bus is free or timeout has elapsed
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	do {
		status = HAL_I2C_Master_Transmit(interface->interface, sensor->address, data, size, timeout);
	} while ((status == HAL_BUSY) && (HAL_GetTick() < end_timestamp));
	osSemaphoreRelease(SensorsSemaphore);

	// Return the interface status
	return status;
}

/**
 * @brief I2C memory write of 1 byte to the sensor for configuration purpose. BLOCKING function
 *
 * @param input sensor_ID: sensor ID
 * @param input memory_address: memory address of the sensor to write to
 * @param input data: single byte to send
 * @param input timeout: time left to configure the sensor in ms. DEFAULT=HAL_MAX_DELAY
 * @return the I2C access status
 */
HAL_StatusTypeDef Sensors::ConfigureSensor(uint8_t sensor_ID, uint16_t memory_address, uint8_t data, uint32_t timeout) {
	uint8_t buffer = data;
	return ConfigureSensor(sensor_ID, memory_address, &buffer, 1, timeout);
}

/**
 * @brief I2C memory write multiple bytes to the sensor for configuration purpose. BLOCKING function
 *
 * @param input sensor_ID: sensor ID
 * @param input memory_address: memory address of the sensor to write to
 * @param input data: byte array to send
 * @param input size: data size in bytes
 * @param input timeout: time left to configure the sensor in ms. DEFAULT=HAL_MAX_DELAY
 * @return the I2C access status
 */
HAL_StatusTypeDef Sensors::ConfigureSensor(uint8_t sensor_ID, uint16_t memory_address, uint8_t* data, uint16_t size, uint32_t timeout) {
	bool success;
	// Find sensor
	Sensor* sensor = FindSensor(sensor_ID, &success);
	if (!success) return HAL_ERROR;

	// Check sensor has been detected and is present
	if (!sensor->present) return HAL_ERROR;

	// Find interface and sensor group
	SensorGroup* sensor_group = FindSensorGroup(sensor->parent, &success);
	if (!success) return HAL_ERROR;
	I2CInterface* interface = FindInterface(sensor_group->parent, &success);
	if (!success) return HAL_ERROR;

	// Compute end timestamp for timeout
	uint32_t end_timestamp = HAL_GetTick() + timeout;
	HAL_StatusTypeDef status = HAL_BUSY;

	// Write to the sensor until I2C bus is free or timeout has elapsed
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	do {
		status = HAL_I2C_Mem_Write(interface->interface, sensor->address, memory_address, sensor->memory_address_size, data, size, timeout);
	} while ((status == HAL_BUSY) && (HAL_GetTick() < end_timestamp)); // try until the bus is free
	osSemaphoreRelease(SensorsSemaphore);

	// Return the interface status
	return status;
}

/**
 * @brief Add a typed sensor data to a sensor
 *
 * @param input sensor_ID: sensor ID
 * @param input sensor_register: sensor data register address. This address is stored and accessible as a register from Registers. It is a single register by design
 * @param input type: data type
 * @param input publishable: if true, add the sensor data register to the sensor group publisher as a topic if it exists. DEFAULT=true
 * @return whether successful
 */
bool Sensors::AddSensorDataWithType(uint8_t sensor_ID, uint16_t sensor_register, uint8_t type, bool publishable) {
	bool success;
	// Find sensor
	Sensor* sensor = FindSensor(sensor_ID, &success);
	if (!success) return false;

	// Find sensor group
	SensorGroup* sensor_group = FindSensorGroup(sensor->parent, &success);
	if (!success) return HAL_ERROR;

	// Override the publishable input if the sensor group is not publishable
	if (!sensor_group->publishable) publishable = false;

	// Check the data is not already registered in the sensor
	FindSensorData(sensor_ID, sensor_register, &success);
	if (success) return false;

	// Setup sensor data
	SensorData data;
	data.active = false;
	data.parent = sensor->ID;
	data.register_address = sensor_register;
	data.type =	type;
	data.read_index = 0;

	// Add the register and assign a memory pointer to store the sensor data value
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	switch (data.type) {
#if (defined(USE_UINT8_I2C_COMMUNICATION) || defined(USE_UINT8_SENSOR_FUNCTION)) && defined(USE_UINT8_REGISTER)
	case UINT8_TYPE:
		data.index = sensor_data_uint8.size();
		if (data.index >= SENSOR_MAX_REGISTER_NB_PER_TYPE) {osSemaphoreRelease(SensorsSemaphore); return false;}
		success = registers->AddRegister<uint8_t>(sensor_register); if (!success) {osSemaphoreRelease(SensorsSemaphore); return false;}
		sensor_data_uint8.push_back(0);
		success = registers->AddRegisterPointer<uint8_t>(sensor_register, &(sensor_data_uint8[data.index]));
		if (!success) {sensor_data_uint8.erase(sensor_data_uint8.begin() + data.index); osSemaphoreRelease(SensorsSemaphore); return false;}
		break;
#endif
#if (defined(USE_UINT16_I2C_COMMUNICATION) || defined(USE_UINT16_SENSOR_FUNCTION)) && defined(USE_UINT16_REGISTER)
	case UINT16_TYPE:
		data.index = sensor_data_uint16.size();
		if (data.index >= SENSOR_MAX_REGISTER_NB_PER_TYPE) {osSemaphoreRelease(SensorsSemaphore); return false;}
		success = registers->AddRegister<uint16_t>(sensor_register); if (!success) {osSemaphoreRelease(SensorsSemaphore); return false;}
		sensor_data_uint16.push_back(0);
		success = registers->AddRegisterPointer<uint16_t>(sensor_register, &(sensor_data_uint16[data.index]));
		if (!success) {sensor_data_uint16.erase(sensor_data_uint16.begin() + data.index); osSemaphoreRelease(SensorsSemaphore); return false;}
		break;
#endif
#if (defined(USE_UINT32_I2C_COMMUNICATION) || defined(USE_UINT32_SENSOR_FUNCTION)) && defined(USE_UINT32_REGISTER)
	case UINT32_TYPE:
		data.index = sensor_data_uint32.size();
		if (data.index >= SENSOR_MAX_REGISTER_NB_PER_TYPE) {osSemaphoreRelease(SensorsSemaphore); return false;}
		success = registers->AddRegister<uint32_t>(sensor_register); if (!success) {osSemaphoreRelease(SensorsSemaphore); return false;}
		sensor_data_uint32.push_back(0);
		success = registers->AddRegisterPointer<uint32_t>(sensor_register, &(sensor_data_uint32[data.index]));
		if (!success) {sensor_data_uint32.erase(sensor_data_uint32.begin() + data.index); osSemaphoreRelease(SensorsSemaphore); return false;}
		break;
#endif
#if (defined(USE_UINT64_I2C_COMMUNICATION) || defined(USE_UINT64_SENSOR_FUNCTION)) && defined(USE_UINT64_REGISTER)
	case UINT64_TYPE:
		data.index = sensor_data_uint64.size();
		if (data.index >= SENSOR_MAX_REGISTER_NB_PER_TYPE) {osSemaphoreRelease(SensorsSemaphore); return false;}
		success = registers->AddRegister<uint64_t>(sensor_register); if (!success) {osSemaphoreRelease(SensorsSemaphore); return false;}
		sensor_data_uint64.push_back(0);
		success = registers->AddRegisterPointer<uint64_t>(sensor_register, &(sensor_data_uint64[data.index]));
		if (!success) {sensor_data_uint64.erase(sensor_data_uint64.begin() + data.index); osSemaphoreRelease(SensorsSemaphore); return false;}
		break;
#endif
#if (defined(USE_INT8_I2C_COMMUNICATION) || defined(USE_INT8_SENSOR_FUNCTION)) && defined(USE_INT8_REGISTER)
	case INT8_TYPE:
		sensor_data_int8.push_back(0);
		if (data.index >= SENSOR_MAX_REGISTER_NB_PER_TYPE) {osSemaphoreRelease(SensorsSemaphore); return false;}
		success = registers->AddRegister<int8_t>(sensor_register); if (!success) {osSemaphoreRelease(SensorsSemaphore); return false;}
		data.index = sensor_data_int8.size();
		success = registers->AddRegisterPointer<int8_t>(sensor_register, &(sensor_data_int8[data.index]));
		if (!success) {sensor_data_int8.erase(sensor_data_int8.begin() + data.index); osSemaphoreRelease(SensorsSemaphore); return false;}
		break;
#endif
#if (defined(USE_INT16_I2C_COMMUNICATION) || defined(USE_INT16_SENSOR_FUNCTION)) && defined(USE_INT16_REGISTER)
	case INT16_TYPE:
		data.index = sensor_data_int16.size();
		if (data.index >= SENSOR_MAX_REGISTER_NB_PER_TYPE) {osSemaphoreRelease(SensorsSemaphore); return false;}
		success = registers->AddRegister<int16_t>(sensor_register); if (!success) {osSemaphoreRelease(SensorsSemaphore); return false;}
		sensor_data_int16.push_back(0);
		success = registers->AddRegisterPointer<int16_t>(sensor_register, &(sensor_data_int16[data.index]));
		if (!success) {sensor_data_int16.erase(sensor_data_int16.begin() + data.index); osSemaphoreRelease(SensorsSemaphore); return false;}
		break;
#endif
#if (defined(USE_INT32_I2C_COMMUNICATION) || defined(USE_INT32_SENSOR_FUNCTION)) && defined(USE_INT32_REGISTER)
	case INT32_TYPE:
		data.index = sensor_data_int32.size();
		if (data.index >= SENSOR_MAX_REGISTER_NB_PER_TYPE) {osSemaphoreRelease(SensorsSemaphore); return false;}
		success = registers->AddRegister<int32_t>(sensor_register); if (!success) {osSemaphoreRelease(SensorsSemaphore); return false;}
		sensor_data_int32.push_back(0);
		success = registers->AddRegisterPointer<int32_t>(sensor_register, &(sensor_data_int32[data.index]));
		if (!success) {sensor_data_int32.erase(sensor_data_int32.begin() + data.index); osSemaphoreRelease(SensorsSemaphore); return false;}
		break;
#endif
#if (defined(USE_INT64_I2C_COMMUNICATION) || defined(USE_INT64_SENSOR_FUNCTION)) && defined(USE_INT64_REGISTER)
	case INT64_TYPE:
		data.index = sensor_data_int64.size();
		if (data.index >= SENSOR_MAX_REGISTER_NB_PER_TYPE) {osSemaphoreRelease(SensorsSemaphore); return false;}
		success = registers->AddRegister<int64_t>(sensor_register); if (!success) {osSemaphoreRelease(SensorsSemaphore); return false;}
		sensor_data_int64.push_back(0);
		success = registers->AddRegisterPointer<int64_t>(sensor_register, &(sensor_data_int64[data.index]));
		if (!success) {sensor_data_int64.erase(sensor_data_int64.begin() + data.index); osSemaphoreRelease(SensorsSemaphore); return false;}
		break;
#endif
#if (defined(USE_FLOAT_I2C_COMMUNICATION) || defined(USE_FLOAT_SENSOR_FUNCTION)) && defined(USE_FLOAT_REGISTER)
	case FLOAT_TYPE:
		data.index = sensor_data_float.size();
		if (data.index >= SENSOR_MAX_REGISTER_NB_PER_TYPE) {osSemaphoreRelease(SensorsSemaphore); return false;}
		success = registers->AddRegister<float>(sensor_register); if (!success) {osSemaphoreRelease(SensorsSemaphore); return false;}
		sensor_data_float.push_back(0);
		success = registers->AddRegisterPointer<float>(sensor_register, &(sensor_data_float[data.index]));
		if (!success) {sensor_data_float.erase(sensor_data_float.begin() + data.index); osSemaphoreRelease(SensorsSemaphore); return false;}
		break;
#endif
#if (defined(USE_DOUBLE_I2C_COMMUNICATION) || defined(USE_DOUBLE_SENSOR_FUNCTION)) && defined(USE_DOUBLE_REGISTER)
	case DOUBLE_TYPE:
		data.index = sensor_data_double.size();
		if (data.index >= SENSOR_MAX_REGISTER_NB_PER_TYPE) {osSemaphoreRelease(SensorsSemaphore); return false;}
		success = registers->AddRegister<double>(sensor_register); if (!success) {osSemaphoreRelease(SensorsSemaphore); return false;}
		sensor_data_double.push_back(0);
		success = registers->AddRegisterPointer<double>(sensor_register, &(sensor_data_double[data.index]));
		if (!success) {sensor_data_double.erase(sensor_data_double.begin() + data.index); osSemaphoreRelease(SensorsSemaphore); return false;}
		break;
#endif
	default:
		osSemaphoreRelease(SensorsSemaphore);
		return false;
		break;
	}

	// Attach the sensor semaphore to the register as it will accessed during sensor processing
	success = registers->AddRegisterSemaphore(sensor_register, &SensorsSemaphore);

	// Set the register as read only
	success = registers->SetRegisterPermissions(sensor_register, READ_PERMISSION);
	if (!success) {osSemaphoreRelease(SensorsSemaphore); return false;}

	if (publishable) {
		// Add the register as a topic of the sensor group publisher if publishable
		success = publishers->AddTopic(sensor_group->ID, sensor_register);
		if (!success) {osSemaphoreRelease(SensorsSemaphore); return false;} // TODO Or continue? Simply don't add the topic ?
	}

	// Add sensor
	sensor->data.push_back(data);
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Check that the sensor data is active
 *
 * @param input sensor_ID: sensor ID
 * @param input sensor_register: sensor data register address
 * @return true if the sensor data is active
 */
bool Sensors::IsSensorDataActive(uint8_t sensor_ID, uint16_t sensor_register) {
	bool success;
	SensorData* sensor_data = FindSensorData(sensor_ID, sensor_register, &success);
	if (!success) return false;

	// No semaphore used as this function is called in sensor callbacks where the semaphore is already taken
	bool status = sensor_data->active;
	return status;
}

/**
 * @brief Set the sensor data status from a sensor
 *
 * @param input sensor_ID: sensor ID
 * @param input sensor_register: sensor data register address
 * @param input status: status. true=active
 * @return whether successful
 */
bool Sensors::SetSensorDataStatus(uint8_t sensor_ID, uint16_t sensor_register, bool status) {
	bool success;
	SensorData* sensor_data = FindSensorData(sensor_ID, sensor_register, &success);
	if (!success) return false;

	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	sensor_data->active = status;
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Activate a sensor data status from a sensor
 *
 * @param input sensor_ID: sensor ID
 * @param input sensor_register: sensor data register address
 * @return whether successful
 */
bool Sensors::ActivateSensorData(uint8_t sensor_ID, uint16_t sensor_register) {
	return SetSensorDataStatus(sensor_ID, sensor_register, true);
}

/**
 * @brief Deactivate a sensor data status from a sensor
 *
 * @param input sensor_ID: sensor ID
 * @param input sensor_register: sensor data register address
 * @return whether successful
 */
bool Sensors::DeactivateSensorData(uint8_t sensor_ID, uint16_t sensor_register) {
	return SetSensorDataStatus(sensor_ID, sensor_register, false);
}

/**
 * @brief Deactivate all sensor data status from a sensor
 *
 * @param input sensor_ID: sensor ID
 * @return whether successful
 */
bool Sensors::DeactivateAllSensorData(uint8_t sensor_ID) {
	bool success;
	Sensor* sensor = FindSensor(sensor_ID, &success);
	if (!success) return false;

	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	for (SensorData &data : sensor->data) {
		data.active = false;
	}
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Add a single byte I2C memory write access to the sensor data sequence. The sequence is run in the same order than creation (FIFO)
 *
 * @param input sensor_ID: sensor ID
 * @param input sensor_register: sensor data register address
 * @param input memory_address: sensor memory address to write to
 * @param input write_data: single byte to write
 * @return whether successful
 */
bool Sensors::AddMemoryWriteSequence(uint8_t sensor_ID, uint16_t sensor_register, uint16_t memory_address, uint8_t write_data) {
	uint8_t write_data_array = write_data;
	return AddMemoryWriteSequence(sensor_ID, sensor_register, memory_address, &write_data_array, 1);
}

/**
 * @brief Add a multiple bytes I2C memory write access to the sensor data sequence. The sequence is run in the same order than creation (FIFO)
 *
 * @param input sensor_ID: sensor ID
 * @param input sensor_register: sensor data register address
 * @param input memory_address: sensor memory address to write to
 * @param input write_data: byte array to write
 * @param input size: write_data size in bytes
 * @return whether successful
 */
bool Sensors::AddMemoryWriteSequence(uint8_t sensor_ID, uint16_t sensor_register, uint16_t memory_address, uint8_t write_data[], uint16_t size) {
	bool success;
	// Find sensor data
	SensorData* sensor_data = FindSensorData(sensor_ID, sensor_register, &success);
	if (!success) return false;

	// Safety check
	if (size > SENSOR_WRITE_SIZE) return false;									// Check that the write buffer used to store the sequence has enough space to save the write value

	// Setup the access
	I2CAccess I2C_access;
	I2C_access.access_mode = MEM_WRITE_MODE;
	I2C_access.memory_address = memory_address;
	for (uint8_t index = 0; index < size; index++) {
		I2C_access.write_data[index] = write_data[index];
	}
	I2C_access.size = size;

	// Add the access to the sequence
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	sensor_data->sequence.push_back(I2C_access);
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Add a I2C memory read access to the sensor data sequence. The sequence is run in the same order than creation (FIFO)
 *
 * @param input sensor_ID: sensor ID
 * @param input sensor_register: sensor data register address
 * @param input memory_address: sensor memory address to read from
 * @param input size: number of bytes to read
 * @return whether successful
 */
bool Sensors::AddMemoryReadSequence(uint8_t sensor_ID, uint16_t sensor_register, uint16_t memory_address, uint16_t size) {
	bool success;
	// Find sensor data
	SensorData* sensor_data = FindSensorData(sensor_ID, sensor_register, &success);
	if (!success) return false;

	// Safety checks
	if (size > SENSOR_BUFFER_SIZE) return false; 								// Check that the sensor read buffer has enough space to save the read value
	if (size > registers->GetTypeSize(sensor_data->type)) return false;			// Check that the size doesn't exceed the register type size

	// Setup the access
	I2CAccess I2C_access;
	I2C_access.access_mode = MEM_READ_MODE;
	I2C_access.memory_address = memory_address;
	I2C_access.size = size;

	// Add the access to the sequence
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	sensor_data->sequence.push_back(I2C_access);
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Add a single byte I2C write access to the sensor data sequence. The sequence is run in the same order than creation (FIFO)
 *
 * @param input sensor_ID: sensor ID
 * @param input sensor_register: sensor data register address
 * @param input write_data: single byte to write
 * @return whether successful
 */
bool Sensors::AddWriteSequence(uint8_t sensor_ID, uint16_t sensor_register, uint8_t write_data) {
	uint8_t write_data_array = write_data;
	return AddWriteSequence(sensor_ID, sensor_register, &write_data_array, 1);
}

/**
 * @brief Add a multiple bytes I2C write access to the sensor data sequence. The sequence is run in the same order than creation (FIFO)
 *
 * @param input sensor_ID: sensor ID
 * @param input sensor_register: sensor data register address
 * @param input write_data: byte array to write
 * @param input size: write_data size in bytes
 * @return whether successful
 */
bool Sensors::AddWriteSequence(uint8_t sensor_ID, uint16_t sensor_register, uint8_t write_data[], uint16_t size) {
	bool success;
	// Find sensor data
	SensorData* sensor_data = FindSensorData(sensor_ID, sensor_register, &success);
	if (!success) return false;

	// Safety check
	if (size > SENSOR_WRITE_SIZE) return false;									// Check that the write buffer used to store the sequence has enough space to save the write value

	// Setup the access
	I2CAccess I2C_access;
	I2C_access.access_mode = WRITE_MODE;
	for (uint8_t index = 0; index < size; index++) {
		I2C_access.write_data[index] = write_data[index];
	}
	I2C_access.size = size;

	// Add the access to the sequence
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	sensor_data->sequence.push_back(I2C_access);
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Add a I2C read access to the sensor data sequence. The sequence is run in the same order than creation (FIFO)
 *
 * @param input sensor_ID: sensor ID
 * @param input sensor_register: sensor data register address
 * @param input size: number of bytes to read
 * @return whether successful
 */
bool Sensors::AddReadSequence(uint8_t sensor_ID, uint16_t sensor_register, uint16_t size) {
	bool success;
	// Find sensor data
	SensorData* sensor_data = FindSensorData(sensor_ID, sensor_register, &success);
	if (!success) return false;

	// Safety checks
	if (size > SENSOR_BUFFER_SIZE) return false;								// Check that the sensor read buffer has enough space to save the read value
	if (size > registers->GetTypeSize(sensor_data->type)) return false;			// Check that the size doesn't exceed the register type size

	// Setup the access
	I2CAccess I2C_access;
	I2C_access.access_mode = READ_MODE;
	I2C_access.size = size;

	// Add the access to the sequence
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	sensor_data->sequence.push_back(I2C_access);
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Add a delay to the sensor data sequence. The sequence is run in the same order than creation (FIFO)
 *
 * @param input sensor_ID: sensor ID
 * @param input sensor_register: sensor data register address
 * @param input delay: delay in ms to wait during the sequence
 * @return whether successful
 */
bool Sensors::AddDelaySequence(uint8_t sensor_ID, uint16_t sensor_register, uint32_t delay) {
	bool success;
	// Find sensor data
	SensorData* sensor_data = FindSensorData(sensor_ID, sensor_register, &success);
	if (!success) return false;

	// Setup access
	I2CAccess I2C_access;
	I2C_access.access_mode = DELAY_MODE;
	I2C_access.delay = delay;

	// Add the access to the sequence
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	sensor_data->sequence.push_back(I2C_access);
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Force a detection of I2C sensors to update presence flag. Can be useful if a sensor if plugged after creation
 *
 * @param input trials: number of trials used to detect the sensor
 * @param input timeout: maximum time in ms left to detect a sensor
 */
void Sensors::DetectSensors(uint32_t trials, uint32_t timeout) {
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	HAL_StatusTypeDef status;
	for (I2CInterface &interface : interfaces) {
		for (SensorGroup &sensor_group : interface.sensor_groups) {
			for (Sensor &sensor : sensor_group.sensors) {
				if (interface.ID != INTERNAL) {
					// If I2C sensor, simple access to detect the sensor
					status = HAL_I2C_IsDeviceReady(interface.interface, sensor.address, trials, timeout);
					sensor.present = (status == HAL_OK);
				} else
					// If not I2C, force presence
					sensor.present = true;
			}
		}
	}
	osSemaphoreRelease(SensorsSemaphore);
}

/**
 * @brief Force the presence flag of a sensor
 *
 * @param input sensor_ID: sensor ID to make present
 * @return whether successful
 */
bool Sensors::ForcePresence(uint8_t sensor_ID) {
	bool success;
	// Find sensor
	Sensor* sensor = FindSensor(sensor_ID, &success);
	if (!success) return false;

	// Force presence
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	sensor->present = true;
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Function called periodically to start the reading of sensors if previous one is finished. Force the re-process of an access if error detected and the read flow is stopped
 */
void Sensors::Spin(void) {
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	if (active) {
		for (I2CInterface &interface : interfaces) {
			// Start the reading for each interface in parallel.
			// Once the DMA interrupts the CPU, the next data is processed by the interrupt
			// This optimizes the bus usage.

			// Start the loop if the previous one is finished and that there is no pending/request running or waiting
			interface.request_regular_process = true; // flag to restart a new regular reading loop
			if (RegularInterfaceTermination(&interface)) {
				if (interface.pending_requests.size() == 0 && interface.pending_sensors.size() == 0) {
					ReadNextSensor(interface.ID);
					osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
					continue;
				} else if (hardware_delay->IsDelayRegistered(interface.ID)) {
					hardware_delay->RemoveDelayMS(interface.ID);
					ReadNextSensor(interface.ID);
					osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
					continue;
				}
			}

			// Force a re-process of the current sensor reading if an error is triggered or if there is a timeout detected.
			// TODO It could be interesting to reset the driver if a timeout is detected (no driver activity for a certain time duration)
			if (interface.error_flag || HAL_GetTick() > interface.activity_timestamp + interface.timeout) {
				ReadNextSensor(interface.ID);
				osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
			}
		}
	}
	osSemaphoreRelease(SensorsSemaphore);

	osDelay(period_ms);
}

/**
 * @brief Find and process the next access among all groups and sensors for a given interface
 *
 * @param input ID: interface ID
 */
void Sensors::ReadNextSensor(uint8_t ID) {
	bool success;
	uint8_t index = FindInterfaceIndex(ID, &success);
	if (!success) {osSemaphoreRelease(SensorsSemaphore); return;}

	I2CInterface* interface = &(interfaces[index]);

	// Process pending requests
	if (interface->pending_requests.size() > 0) {
		for (uint8_t request_index = 0; request_index < interface->pending_requests.size(); request_index++) {
			// Find parents of the pending request
			SensorData* request_data = &(interface->pending_requests[request_index].data);
			SensorGroup* request_group;
			Sensor* request_sensor = FindSensorPath(request_data->parent, NULL, &request_group, &success);
			if (!success) {osSemaphoreRelease(SensorsSemaphore); return;}

			// If not already processing ...
			if (!interface->pending_requests[request_index].processing) {
				// Check that the sensor is not being processed in the regular loop. If not, process the pending request
				if (IsSensorPending(interface, request_sensor->ID) || (interface->current_sensor_ID == request_sensor->ID) || IsSensorRequested(interface, request_sensor->ID)) continue; // checks that sensor is not pending nor currently read in nominal loop nor current read by a request
				interface->pending_requests[request_index].processing = true;

				// Initialize the delay if the first item of the sequence is a delay so calling ExitRegular works
				if (request_data->sequence[0].access_mode == DELAY_MODE)
					request_sensor->timestamp = HAL_GetTick() + request_data->sequence[0].delay;
			}

			// If request is being processed, get the next access in the sequence to process
			I2CAccess* request_access = &(request_data->sequence[request_data->read_index]);

			// Check that the access have to be processed at the very moment
			if ((request_access->access_mode == DELAY_MODE && HAL_GetTick() > request_sensor->timestamp) || request_access->access_mode != DELAY_MODE /*|| (delay_flag && interface->access_source = REQUEST_ACCESS)*/) {
//				 delay_flag = false;																					// Flag to force process of the access if hardware delay is being used

				// In case of natural timeout (without hardware delay), move to the next sequence item
				if (request_access->access_mode == DELAY_MODE) {
					if (!MoveToNextSequence(request_data)) {
						*(interface->pending_requests[request_index].flag) = true;
						interface->pending_requests.erase(interface->pending_requests.begin() + request_index);
						return ExitPending(interface);
					}
					request_access = &(request_data->sequence[request_data->read_index]);
				}

				// Process the access
				interface->request_index = request_index;																// Save which request is being processed
				interface->access_source = REQUEST_ACCESS;																// Setup the flag to precise the source of the sensor access
				return ProcessAccess(interface, request_group, request_sensor, request_data, request_access);
			}
		}

		// If no request access to process, resume the regular loop or exit if regular loop finished and no pending sensor
		if (!interface->request_regular_process && RegularInterfaceTermination(interface) && interface->pending_sensors.size() == 0)
			return ExitRegular(interface);
	}

	// Process pending sensors from regular loop
	if (interface->pending_sensors.size() > 0) {
		for (uint8_t pending_index = 0; pending_index < interface->pending_sensors.size(); pending_index++) {
			// Find parents of the pending sensor
			Sensor* pending_sensor = interface->pending_sensors[pending_index];
			SensorGroup* pending_group = FindSensorGroup(pending_sensor->parent, &success);
			SensorData* pending_data = &(pending_sensor->data[pending_sensor->read_index]);
			I2CAccess* pending_access = &(pending_data->sequence[pending_data->read_index]);
			if (!success) {osSemaphoreRelease(SensorsSemaphore); return;}

			// If the sensor reading is not yet started, start only if the sensor is not being processed in a request
			// This specific case only occur if the nominal loop put the sensor into the pending list because a request is already reading the sensor so the nominal loop can resume reading of next sensors
			if (!pending_sensor->processing) {
				if (IsSensorRequested(interface, pending_sensor->ID)) continue;
				pending_sensor->processing = true;
			}

			// Check that the access have to be processed at the very moment
			if ((pending_access->access_mode == DELAY_MODE && HAL_GetTick() > pending_sensor->timestamp) || pending_access->access_mode != DELAY_MODE /*|| (delay_flag && interface->access_source = PENDING_ACCESS)*/) {
//				delay_flag = false;																						// Flag to force process of the access if hardware delay is being used

				// In case of natural timeout (without hardware delay), move to the next sequence item
				if (pending_access->access_mode == DELAY_MODE) {
					if (!MoveToNextSequence(pending_data)) {
						if (!MoveToNextData(pending_sensor)) {
							interface->pending_sensors.erase(interface->pending_sensors.begin() + pending_index);
							ProccessGroupTermination(interface, pending_group);
							return ExitPending(interface);
						}
					}
					pending_data = &(pending_sensor->data[pending_sensor->read_index]);
					pending_access = &(pending_data->sequence[pending_data->read_index]);
				}

				// Process the access
				interface->pending_index = pending_index;																// Save which pending sensor is being processed
				interface->access_source = PENDING_ACCESS;																// Setup the flag to precise the source of the sensor access
				return ProcessAccess(interface, pending_group, pending_sensor, pending_data, pending_access);
			}
		}
		// Exit regular if the regular loop is finished, no pending action to process at the moment (if only delays) and no fresh start is asked.
		// Without this line of code, the program would restart a new sensor loop if the regular loop if finished and no pending sequence is to process.
		// Only the spin() function should restart the loop by the mean of force_regular_process
		if (!interface->request_regular_process && RegularInterfaceTermination(interface))
			return ExitRegular(interface);
	}

	// Process regular sensors
	if (active) {
		interface->request_regular_process = false;	// reset the flag to start a new regular reading loop
		interface->pending_index = 0;

		// If there is no group, return
		if (interface->sensor_groups.size() == 0) {osSemaphoreRelease(SensorsSemaphore); return;}
		SensorGroup* sensor_group = &(interface->sensor_groups[interface->read_index]);

		// If the group is not the current group or no current group (UINT8_MAX), check conditions:
		// 		group is inactive
		// 		or prescaler condition not satisfied
		// 		or there is no sensor
		//		or group is pending
		// If so, move to the next group. Exit if no other group to stop the reading loop
		if (interface->current_group_ID == UINT8_MAX || sensor_group->ID != interface->current_group_ID) {
			if (!sensor_group->active || sensor_group->counter < sensor_group->prescaler - 1 || sensor_group->sensors.size() == 0 || (RegularGroupTermination(interface, sensor_group) && !PendingGroupTermination(interface, sensor_group))) {
				if (sensor_group->counter < sensor_group->prescaler) sensor_group->counter += 1;
				if (MoveToNextGroup(interface)) {return ReadNextSensor(ID);}
				return ExitRegular(interface);
			}
			// Select the group as the current group
			interface->current_group_ID = sensor_group->ID;
		}
		Sensor* sensor = &(sensor_group->sensors[sensor_group->read_index]);

		// If the sensor is not the current sensor or no current sensor (UINT8_MAX), check conditions:
		// 		sensor is inactive
		// 		or sensor not present
		// 		or sensor has no data
		//		or sensor is processed by a request (put the sensor in the pending list to process later when the request is finished)
		// If so, move to the next sensor. Exit if no other group to stop the reading loop
		if (interface->current_sensor_ID == UINT8_MAX || sensor->ID != interface->current_sensor_ID) { // avoid checking for sensor validity if already selected
			if (!sensor->active || !sensor->present || sensor->data.size() == 0) {
				if (MoveToNextSensor(interface, sensor_group)) {return ReadNextSensor(ID);}
				if (MoveToNextGroup(interface)) {return ReadNextSensor(ID);}
				return ExitRegular(interface);
			}
			if (IsSensorRequested(interface, sensor->ID)) {
				interface->pending_sensors.push_back(sensor);
				if (MoveToNextSensor(interface, sensor_group)) {return ReadNextSensor(ID);}
				if (MoveToNextGroup(interface)) {return ReadNextSensor(ID);}
				return ExitRegular(interface);
			}
			// Select the sensor as the current sensor
			interface->current_sensor_ID = sensor->ID;
		}
		SensorData* sensor_data = &(sensor->data[sensor->read_index]);

		// If the data is inactive or no sequence registered, select the next sensor data. If not, select next sensor, if not select next group, if not select next interface, if not stops

		// If the sensor data is not the current sensor data or no current sensor data (UINT16_MAX), check conditions:
		// 		sensor_data is inactive
		// 		or no sequence attached to the sensor data
		// If so, move to the next sensor data. Exit if no other group to stop the reading loop
		if (interface->current_data_address == UINT16_MAX || sensor_data->register_address != interface->current_data_address) { // avoid checking for data validity if already selected
			if (!sensor_data->active || sensor_data->sequence.size() == 0) {
				if (MoveToNextData(sensor)) {return ReadNextSensor(ID);}
				if (MoveToNextSensor(interface, sensor_group)) {return ReadNextSensor(ID);}
				if (MoveToNextGroup(interface)) {return ReadNextSensor(ID);}
				return ExitRegular(interface);
			}
			// Select the sensor data as the current sensor data
			interface->current_data_address = sensor_data->register_address;
		}
		I2CAccess* I2C_access = &(sensor_data->sequence[sensor_data->read_index]);

		// Process access
		interface->access_source = REGULAR_ACCESS;																		// Setup the flag to precise the source of the sensor access
		sensor->processing = true;																						// Set sensor processing flag to avoid starting a reading request of the sensor
		return ProcessAccess(interface, sensor_group, sensor, sensor_data, I2C_access);
	}
}

/**
 * @brief Process the access from the sensor data sequence
 *
 * @param input interface: the interface pointer linked to the access (to avoid looking for it anew)
 * @param input sensor_group: sensor group parent to the access (to avoid looking for it anew)
 * @param input sensor: sensor parent to the access (to avoid looking for it anew)
 * @param input sensor_data: sensor data parent to the access (to avoid looking for it anew)
 * @param input I2C_access: access to process
 */
void Sensors::ProcessAccess(I2CInterface* interface, SensorGroup* sensor_group, Sensor* sensor, SensorData* sensor_data, I2CAccess* I2C_access) {
	HAL_StatusTypeDef status = HAL_OK;

	// Process Delay
	if (I2C_access->access_mode == DELAY_MODE) {
		// Compute the end timeout timestamp
		sensor->timestamp = HAL_GetTick() + I2C_access->delay;

		// If from regular loop, put sensor in the pending list to be processed later and move to the next sensor. Exit if no other group to stop the reading loop
		if (interface->access_source == REGULAR_ACCESS) {
			interface->pending_sensors.push_back(sensor);
			if (MoveToNextSensor(interface, sensor_group)) {return ReadNextSensor(interface->ID);}
			if (MoveToNextGroup(interface)) {return ReadNextSensor(interface->ID);}
			return ExitRegular(interface);
		} else
			return ExitPending(interface);
	// Process I2C write access
	} else if (I2C_access->access_mode == WRITE_MODE) {
		status = HAL_I2C_Master_Transmit_DMA(interface->interface, sensor->address, I2C_access->write_data, I2C_access->size);
	// Process I2C memory write access
	} else if (I2C_access->access_mode == MEM_WRITE_MODE) {
		status = HAL_I2C_Mem_Write_DMA(interface->interface, sensor->address, I2C_access->memory_address, sensor->memory_address_size, I2C_access->write_data, I2C_access->size);
	// Process I2C read access
	} else if (I2C_access->access_mode == READ_MODE) {
		status = HAL_I2C_Master_Receive_DMA(interface->interface, sensor->address, interface->buffer, I2C_access->size);
	// Process I2C memory read access
	} else if (I2C_access->access_mode == MEM_READ_MODE) {
		status = HAL_I2C_Mem_Read_DMA(interface->interface, sensor->address, I2C_access->memory_address, sensor->memory_address_size, interface->buffer, I2C_access->size);
	// Process custom code access
	} else if (CODE_MODE) {
		switch (sensor_data->type) {
#if defined(USE_UINT8_SENSOR_FUNCTION) && defined(USE_UINT8_REGISTER)
		case UINT8_TYPE:
			ProcessCode<uint8_t>(interface, sensor_data, I2C_access);
			break;
#endif
#if defined(USE_UINT16_SENSOR_FUNCTION) && defined(USE_UINT16_REGISTER)
		case UINT16_TYPE:
			ProcessCode<uint16_t>(interface, sensor_data, I2C_access);
			break;
#endif
#if defined(USE_UINT32_SENSOR_FUNCTION) && defined(USE_UINT32_REGISTER)
		case UINT32_TYPE:
			ProcessCode<uint32_t>(interface, sensor_data, I2C_access);
			break;
#endif
#if defined(USE_UINT64_SENSOR_FUNCTION) && defined(USE_UINT64_REGISTER)
		case UINT64_TYPE:
			ProcessCode<uint64_t>(interface, sensor_data, I2C_access);
			break;
#endif
#if defined(USE_INT8_SENSOR_FUNCTION) && defined(USE_INT8_REGISTER)
		case INT8_TYPE:
			ProcessCode<int8_t>(interface, sensor_data, I2C_access);
			break;
#endif
#if defined(USE_INT16_SENSOR_FUNCTION) && defined(USE_INT16_REGISTER)
		case INT16_TYPE:
			ProcessCode<int16_t>(interface, sensor_data, I2C_access);
			break;
#endif
#if defined(USE_INT32_SENSOR_FUNCTION) && defined(USE_INT32_REGISTER)
		case INT32_TYPE:
			ProcessCode<int32_t>(interface, sensor_data, I2C_access);
			break;
#endif
#if defined(USE_INT64_SENSOR_FUNCTION) && defined(USE_INT64_REGISTER)
		case INT64_TYPE:
			ProcessCode<int64_t>(interface, sensor_data, I2C_access);
			break;
#endif
#if defined(USE_FLOAT_SENSOR_FUNCTION) && defined(USE_FLOAT_REGISTER)
		case FLOAT_TYPE:
			ProcessCode<float>(interface, sensor_data, I2C_access);
			break;
#endif
#if defined(USE_DOUBLE_SENSOR_FUNCTION) && defined(USE_DOUBLE_REGISTER)
		case DOUBLE_TYPE:
			ProcessCode<double>(interface, sensor_data, I2C_access);
			break;
#endif
		default:
			osSemaphoreRelease(SensorsSemaphore);
			return;
			break;
		}
		ProccessReceivedData(interface);
	}

	// Process anew if bus busy
	if (status == HAL_BUSY)
		return ProcessAccess(interface, sensor_group, sensor, sensor_data, I2C_access);

	// If a problem is detected, a new access is tried by the Spin function later.
	// 		TODO keep a re-process by Spin?
	//		TODO Remove or deactivate or unpresent the sensor instead?
	// 		TODO put in pending list to try later?
	if (status == HAL_ERROR)
		interface->error_flag = true;

	osSemaphoreRelease(SensorsSemaphore);
}

/**
 * @brief Function called by hardware interface and DMA ISR to assess the end of an I2C access
 *
 * @param input harware_interface: I2C hardware interface handle that generated the interruption
 */
void Sensors::ReceiveSensorData(I2C_HandleTypeDef* harware_interface) {
	bool success;
	// Find interface ID
	uint8_t ID = FindInterfaceIndexWithHandle(harware_interface, &success);
	if (!success) return;

	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	I2CInterface* interface = &(interfaces[ID]);

	// Process the end of I2C access
	ProccessReceivedData(interface);
}

/**
 * @brief Process the I2C access acknowledgment from the I2C hardware interface
 *
 * @param input interface: the interface pointer that requests a process of the return data
 */
void Sensors::ProccessReceivedData(I2CInterface* interface) {
	// Find current I2C access
	uint8_t ID = interface->ID;
	SensorGroup* sensor_group;
	Sensor* sensor;
	SensorData* sensor_data;
	I2CAccess* I2C_access;

	bool success;
	if (interface->access_source == REQUEST_ACCESS) {
		sensor_data = &(interface->pending_requests[interface->request_index].data);
		sensor = FindSensorPath(sensor_data->parent, NULL, &sensor_group, &success);
	} else if (interface->access_source == PENDING_ACCESS) {
		sensor = interface->pending_sensors[interface->pending_index];
		sensor_group = FindSensorGroup(sensor->parent, &success);
		if (sensor->read_index >= sensor->data.size()) {osSemaphoreRelease(SensorsSemaphore); return;}
		sensor_data = &(sensor->data[sensor->read_index]);
	} else {
		if (interface->read_index >= interface->sensor_groups.size()) {osSemaphoreRelease(SensorsSemaphore); return;}
		sensor_group = &(interface->sensor_groups[interface->read_index]);
		if (sensor_group->read_index >= sensor_group->sensors.size()) {osSemaphoreRelease(SensorsSemaphore); return;}
		sensor = &(sensor_group->sensors[sensor_group->read_index]);
		if (sensor->read_index >= sensor->data.size()) {osSemaphoreRelease(SensorsSemaphore); return;}
		sensor_data = &(sensor->data[sensor->read_index]);
	}

	I2C_access = &(sensor_data->sequence[sensor_data->read_index]);

	// Reset the reading buffer if first access
	// Fill the reading buffer if any read data has been received (READ_MODE or MEM_READ_MODE or CODE_MODE)
	switch (sensor_data->type) {
	// UINT8 CASE
#if (defined(USE_UINT8_I2C_COMMUNICATION) || defined(USE_UINT8_SENSOR_FUNCTION)) && defined(USE_UINT8_REGISTER)
	case UINT8_TYPE:
		if (sensor_data->read_index == 0)
			sensor_data_uint8[sensor_data->index] = 0; // Clean buffer when start access
#ifdef USE_UINT8_I2C_COMMUNICATION
		if (I2C_access->access_mode == READ_MODE || I2C_access->access_mode == MEM_READ_MODE)
			TransferBuffer<uint8_t>(&(sensor_data_uint8[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
#ifdef USE_UINT8_SENSOR_FUNCTION
		if (I2C_access->access_mode == CODE_MODE)
			TransferBuffer<uint8_t>(&(sensor_data_uint8[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
		break;
#endif
	// UINT16 CASE
#if (defined(USE_UINT16_I2C_COMMUNICATION) || defined(USE_UINT16_SENSOR_FUNCTION)) && defined(USE_UINT16_REGISTER)
	case UINT16_TYPE:
		if (sensor_data->read_index == 0)
			sensor_data_uint16[sensor_data->index] = 0; // Clean buffer when start access
#ifdef USE_UINT16_I2C_COMMUNICATION
		if (I2C_access->access_mode == READ_MODE || I2C_access->access_mode == MEM_READ_MODE)
			TransferBuffer<uint16_t>(&(sensor_data_uint16[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
#ifdef USE_UINT16_SENSOR_FUNCTION
		if (I2C_access->access_mode == CODE_MODE)
			TransferBuffer<uint16_t>(&(sensor_data_uint16[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
		break;
#endif
	// UINT32 CASE
#if (defined(USE_UINT32_I2C_COMMUNICATION) || defined(USE_UINT32_SENSOR_FUNCTION)) && defined(USE_UINT32_REGISTER)
	case UINT32_TYPE:
		if (sensor_data->read_index == 0)
			sensor_data_uint32[sensor_data->index] = 0; // Clean buffer when start access
#ifdef USE_UINT32_I2C_COMMUNICATION
		if (I2C_access->access_mode == READ_MODE || I2C_access->access_mode == MEM_READ_MODE)
			TransferBuffer<uint32_t>(&(sensor_data_uint32[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
#ifdef USE_UINT32_SENSOR_FUNCTION
		if (I2C_access->access_mode == CODE_MODE)
			TransferBuffer<uint32_t>(&(sensor_data_uint32[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
		break;
#endif
	// UINT64 CASE
#if (defined(USE_UINT64_I2C_COMMUNICATION) || defined(USE_UINT64_SENSOR_FUNCTION)) && defined(USE_UINT64_REGISTER)
	case UINT64_TYPE:
		if (sensor_data->read_index == 0)
			sensor_data_uint64[sensor_data->index] = 0; // Clean buffer when start access
#ifdef USE_UINT64_I2C_COMMUNICATION
		if (I2C_access->access_mode == READ_MODE || I2C_access->access_mode == MEM_READ_MODE)
			TransferBuffer<uint64_t>(&(sensor_data_uint64[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
#ifdef USE_UINT64_SENSOR_FUNCTION
		if (I2C_access->access_mode == CODE_MODE)
			TransferBuffer<uint64_t>(&(sensor_data_uint64[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
		break;
#endif
	// INT8 CASE
#if (defined(USE_INT8_I2C_COMMUNICATION) || defined(USE_INT8_SENSOR_FUNCTION)) && defined(USE_INT8_REGISTER)
	case INT8_TYPE:
		if (sensor_data->read_index == 0)
			sensor_data_int8[sensor_data->index] = 0; // Clean buffer when start access
#ifdef USE_INT8_I2C_COMMUNICATION
		if (I2C_access->access_mode == READ_MODE || I2C_access->access_mode == MEM_READ_MODE)
			TransferBuffer<int8_t>(&(sensor_data_int8[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
#ifdef USE_INT8_SENSOR_FUNCTION
		if (I2C_access->access_mode == CODE_MODE)
			TransferBuffer<int8_t>(&(sensor_data_int8[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
		break;
#endif
	// INT16 CASE
#if (defined(USE_INT16_I2C_COMMUNICATION) || defined(USE_INT16_SENSOR_FUNCTION)) && defined(USE_INT16_REGISTER)
	case INT16_TYPE:
		if (sensor_data->read_index == 0)
			sensor_data_int16[sensor_data->index] = 0; // Clean buffer when start access
#ifdef USE_INT16_I2C_COMMUNICATION
		if (I2C_access->access_mode == READ_MODE || I2C_access->access_mode == MEM_READ_MODE)
			TransferBuffer<int16_t>(&(sensor_data_int16[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
#ifdef USE_INT16_SENSOR_FUNCTION
		if (I2C_access->access_mode == CODE_MODE)
			TransferBuffer<int16_t>(&(sensor_data_int16[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
		break;
#endif
	// INT32 CASE
#if (defined(USE_INT32_I2C_COMMUNICATION) || defined(USE_INT32_SENSOR_FUNCTION)) && defined(USE_INT32_REGISTER)
	case INT32_TYPE:
		if (sensor_data->read_index == 0)
			sensor_data_int32[sensor_data->index] = 0; // Clean buffer when start access
#ifdef USE_INT32_I2C_COMMUNICATION
		if (I2C_access->access_mode == READ_MODE || I2C_access->access_mode == MEM_READ_MODE)
			TransferBuffer<int32_t>(&(sensor_data_int32[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
#ifdef USE_INT32_SENSOR_FUNCTION
		if (I2C_access->access_mode == CODE_MODE)
			TransferBuffer<int32_t>(&(sensor_data_int32[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
		break;
#endif
	// INT64 CASE
#if (defined(USE_INT64_I2C_COMMUNICATION) || defined(USE_INT64_SENSOR_FUNCTION)) && defined(USE_INT64_REGISTER)
	case INT64_TYPE:
		if (sensor_data->read_index == 0)
			sensor_data_int64[sensor_data->index] = 0; // Clean buffer when start access
#ifdef USE_INT64_I2C_COMMUNICATION
		if (I2C_access->access_mode == READ_MODE || I2C_access->access_mode == MEM_READ_MODE)
			TransferBuffer<int64_t>(&(sensor_data_int64[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
#ifdef USE_INT64_SENSOR_FUNCTION
		if (I2C_access->access_mode == CODE_MODE)
			TransferBuffer<int64_t>(&(sensor_data_int64[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
		break;
#endif
	// FLOAT CASE
#if (defined(USE_FLOAT_I2C_COMMUNICATION) || defined(USE_FLOAT_SENSOR_FUNCTION)) && defined(USE_FLOAT_REGISTER)
	case FLOAT_TYPE:
		if (sensor_data->read_index == 0)
			sensor_data_float[sensor_data->index] = 0; // Clean buffer when start access
#ifdef USE_FLOAT_I2C_COMMUNICATION
		if (I2C_access->access_mode == READ_MODE || I2C_access->access_mode == MEM_READ_MODE)
			TransferBuffer<float>(&(sensor_data_float[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
#ifdef USE_FLOAT_SENSOR_FUNCTION
		if (I2C_access->access_mode == CODE_MODE)
			TransferBuffer<float>(&(sensor_data_float[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
		break;
#endif
	// DOUBLE CASE
#if (defined(USE_DOUBLE_I2C_COMMUNICATION) || defined(USE_DOUBLE_SENSOR_FUNCTION)) && defined(USE_DOUBLE_REGISTER)
	case DOUBLE_TYPE:
		if (sensor_data->read_index == 0)
			sensor_data_double[sensor_data->index] = 0; // Clean buffer when start access
#ifdef USE_DOUBLE_I2C_COMMUNICATION
		if (I2C_access->access_mode == READ_MODE || I2C_access->access_mode == MEM_READ_MODE)
			TransferBuffer<double>(&(sensor_data_double[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
#ifdef USE_DOUBLE_SENSOR_FUNCTION
		if (I2C_access->access_mode == CODE_MODE)
			TransferBuffer<double>(&(sensor_data_double[sensor_data->index]), interface->buffer, I2C_access->size);
#endif
		break;
#endif
	default:
		break;
	}

	// Move to the next access from the sequence.
	// Request data case
	if (interface->access_source == REQUEST_ACCESS) {
		if (MoveToNextSequence(sensor_data)) {return ReadNextSensor(ID);}
		*(interface->pending_requests[interface->request_index].flag) = true;
		interface->pending_requests.erase(interface->pending_requests.begin() + interface->request_index);
		return ExitPending(interface);

	// Penging sensor case
	} else if (interface->access_source == PENDING_ACCESS) {
		if (MoveToNextSequence(sensor_data)) {return ReadNextSensor(ID);}
		if (MoveToNextData(sensor)) {return ReadNextSensor(ID);}
		interface->pending_sensors.erase(interface->pending_sensors.begin() + interface->pending_index);
		ProccessGroupTermination(interface, sensor_group);
		return ExitPending(interface);

	// Regular access case
	} else if (interface->access_source == REGULAR_ACCESS) {
		if (MoveToNextSequence(sensor_data)) {return ReadNextSensor(ID);}
		if (MoveToNextData(sensor)) {return ReadNextSensor(ID);}
		if (MoveToNextSensor(interface, sensor_group)) {return ReadNextSensor(ID);}
		if (MoveToNextGroup(interface)) {return ReadNextSensor(ID);}
		return ExitRegular(interface);
	}

	osSemaphoreRelease(SensorsSemaphore);
}

/**
 * @brief Static function called by the hardware delay ISR to restart the sensor reading sequence after some delay
 *
 * @param input context: the Sensors instance
 * @param input ID: the interface ID used to register the hardware delay from HardwareDelay
 */
void Sensors::HardwareDelayISR(void* context, uint8_t ID) {
	Sensors* self = reinterpret_cast<Sensors*>(context);
	osSemaphoreAcquire(self->SensorsSemaphore, osWaitForever);
//	self->delay_flag = true;
	osSemaphoreRelease(self->SensorsSemaphore);
	self->ReceiveDelayInterrupt(ID);
}

/**
 * @brief Function used to process the hardware delay
 *
 * @param input ID: the interface ID used to register the hardware delay from HardwareDelay
 */
void Sensors::ReceiveDelayInterrupt(uint8_t ID) {
	bool success;
	// Find the interface
	uint8_t index = FindInterfaceIndex(ID, &success);
	if (!success) return;
	I2CInterface* interface = &(interfaces[index]);

	// Delay from request case.
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	if (delay_source == REQUEST_ACCESS) {
		// Find access
		SensorData* request_data = &(interface->pending_requests[delayed_index].data);
		if (request_data->read_index >= request_data->sequence.size()) return;
		I2CAccess* request_access = &(request_data->sequence[request_data->read_index]);

		// Move to next access. Remove request if sequence is finished
		if (request_access->access_mode == DELAY_MODE) {
			if (!MoveToNextSequence(request_data)) {
				*(interface->pending_requests[delayed_index].flag) = true;
				interface->pending_requests.erase(interface->pending_requests.begin() + delayed_index);
				return ExitPending(interface);
			}
		}

	// Delay from pending sensor case.
	} else if (delay_source == PENDING_ACCESS) {
		// Find access
		Sensor* pending_sensor = interface->pending_sensors[delayed_index];
		SensorGroup* pending_group = FindSensorGroup(pending_sensor->parent, &success);
		if (pending_sensor->read_index >= pending_sensor->data.size()) return;
		SensorData* pending_data = &(pending_sensor->data[pending_sensor->read_index]);
		if (pending_data->read_index >= pending_data->sequence.size()) return;
		I2CAccess* pending_access = &(pending_data->sequence[pending_data->read_index]);

		// Move to next access. Remove pending sensor if no other data to process
		if (pending_access->access_mode == DELAY_MODE) {
			if (!MoveToNextSequence(pending_data)) {
				if (!MoveToNextData(pending_sensor)) {
					interface->pending_sensors.erase(interface->pending_sensors.begin() + delayed_index);
					ProccessGroupTermination(interface, pending_group);
					return ExitPending(interface);
				}
			}
		}
	}

	// In every other case, read current access
	return ReadNextSensor(ID);
}

/**
 * @brief Exit from regular loop. Check if there is a pending sensor or a request data to be processed. If true, move to the next access or setup a hardware delay if remains only running delays. Requests have the priority over pending sensors
 *
 * @param input interface: the interface pointer
 */
void Sensors::ExitRegular(I2CInterface* interface) {
	// If remain requests or pending sensors
	if (interface->pending_sensors.size() > 0 || interface->pending_requests.size() > 0) {
		bool success;
		uint32_t min_remaining_time = UINT32_MAX;
		uint32_t current_timestamp = HAL_GetTick();
		uint32_t remaining_time;

		// Check if access to process for requests
		for (uint8_t request_index = 0; request_index < interface->pending_requests.size(); request_index++) {
			// Do not take into account requests that are not being processed
			if (!interface->pending_requests[request_index].processing) continue;

			// Find access
			ReadRequest request = interface->pending_requests[request_index];
			SensorData* request_data = &(request.data);
			if (request_data->read_index >= request_data->sequence.size()) continue;
			I2CAccess* request_access = &(request_data->sequence[request_data->read_index]);
			Sensor* request_sensor = FindSensor(request_data->parent, &success);
			if (!success) continue;

			// If not a delay, process the access
			if (request_access->access_mode != DELAY_MODE) {return ReadNextSensor(interface->ID);}

			// If the delay has already elapsed, move to next access and process.
			if (request_sensor->timestamp <= current_timestamp) {
				// If no other access, exit the request loop and remove the request.
				if (!MoveToNextSequence(request_data)) {
					*(interface->pending_requests[request_index].flag) = true;									// flag to exit the blocking request loop from RequestRead
					interface->pending_requests.erase(interface->pending_requests.begin() + request_index);
					return ExitPending(interface);
				}
				return ReadNextSensor(interface->ID);
			}

			// Update the minimum delay
			remaining_time = request_sensor->timestamp - current_timestamp;
			if (remaining_time < min_remaining_time) {
				delay_source = REQUEST_ACCESS;
				delayed_index = request_index;
				min_remaining_time = remaining_time;
			}

		}

		// Check if access to process for pending sensors
		for (uint8_t pending_index = 0; pending_index < interface->pending_sensors.size(); pending_index++) {
			// Do not take into account pending sensors that are not being processed
			if (!interface->pending_sensors[pending_index]->processing) continue;

			// Find access
			Sensor* pending_sensor = interface->pending_sensors[pending_index];
			if (pending_sensor->read_index >= pending_sensor->data.size()) continue;
			SensorData* pending_data = &(pending_sensor->data[pending_sensor->read_index]);
			if (pending_data->read_index >= pending_data->sequence.size()) continue;
			I2CAccess* pending_access = &(pending_data->sequence[pending_data->read_index]);
			SensorGroup* pending_group = FindSensorGroup(pending_sensor->parent, &success);
			if (!success) continue;

			// If not a delay, process the access
			if (pending_access->access_mode != DELAY_MODE) {return ReadNextSensor(interface->ID);}

			// If the delay has already elapsed, move to next access and process.
			if (pending_sensor->timestamp <= current_timestamp) {
				if (!MoveToNextSequence(pending_data)) {
					// If no other data, remove the pending sensor and check group termination
					if (!MoveToNextData(pending_sensor)) {
						interface->pending_sensors.erase(interface->pending_sensors.begin() + pending_index);
						ProccessGroupTermination(interface, pending_group);
						return ExitPending(interface);
					}
				}
				return ReadNextSensor(interface->ID); // if delay and time has already elapsed, process
			}

			// Update the minimum delay
			remaining_time = pending_sensor->timestamp - current_timestamp;
			if (remaining_time < min_remaining_time) {
				delay_source = PENDING_ACCESS;
				delayed_index = pending_index;
				min_remaining_time = remaining_time;
			}
		}

		// If there are only delays remaining. Then start a hardware delay with the minimum delay that has been found
		// TODO "min_remaining_time < period_ms ?" not to start a hardware delay while another spin will occur? But what if one stops the sensors? Pending sensors won't finish!
		if (min_remaining_time > 0) { // && min_remaining_time < period_ms) {
			void* context = reinterpret_cast<void*>(this);
			hardware_delay->AddDelayMS(interface->ID, min_remaining_time, context, HardwareDelayISR);
		}
	}

	// Reset the current pointers to NONE and set the activity timestamp
	interface->activity_timestamp = HAL_GetTick();
	interface->current_group_ID = UINT8_MAX;
	interface->current_sensor_ID = UINT8_MAX;
	interface->current_data_address = UINT16_MAX;
	osSemaphoreRelease(SensorsSemaphore);
	return;
}

/**
 * @brief Exit from a pending or request access. Check if still regular access to process or do a regular exit to process possible remaining pending or request access
 *
 * @param input interface: the interface pointer
 */
void Sensors::ExitPending(I2CInterface* interface) {
	if (!RegularInterfaceTermination(interface)) ReadNextSensor(interface->ID);
	return ExitRegular(interface);
}

/**
 * @brief Check if the regular reading of sensors is finished
 *
 * @param input interface: the interface pointer
 * @return true if the regular reading is finished
 */
bool Sensors::RegularInterfaceTermination(I2CInterface* interface) {
	/*
	if (interface->read_index > 0) return false;
	if (interface->sensor_groups.size() == 0) return true;

	SensorGroup* sensor_group = &(interface->sensor_groups[0]);
	return RegularGroupTermination(interface, sensor_group);
	*/

	// The group ID is set to UINT8_MAX when loop is finished and ready to start
	return interface->current_group_ID == UINT8_MAX;
}

/**
 * @brief Function to reset the group and to spin the group publisher if there is no sensor left in the group
 *
 * @param input interface: the interface pointer
 * @param input sensor_group: sensor group to process termination
 */
void Sensors::ProccessGroupTermination(I2CInterface* interface, SensorGroup* sensor_group) {
	// Check group termination
	if (RegularGroupTermination(interface, sensor_group) && PendingGroupTermination(interface, sensor_group)) {
		// Reset prescaler counter
		sensor_group->counter = 0;

		// Spin publisher if publishable
		if (sensor_group->publishable) {
			osSemaphoreRelease(SensorsSemaphore);
			publishers->SpinPublisher(sensor_group->ID);
			osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
		}
	}
}

/**
 * @brief Check the group termination in the regular point of view
 *
 * @param input interface: the interface pointer
 * @param input sensor_group: sensor group to check
 * @return true if no sensor left to read in the regular loop in the sensor group
 */
bool Sensors::RegularGroupTermination(I2CInterface* interface, SensorGroup* sensor_group) {
	// Check sensor group read index
	if (sensor_group->read_index > 0) return false;
	if (sensor_group->sensors.size() == 0) return true;

	// Check read index of the first sensor
	Sensor* sensor = &(sensor_group->sensors[0]);
	if (sensor->read_index > 0) return false;
	if (sensor->data.size() == 0) return true;

	// Check the sensor is not pending.
	// If it is actually the case, it means that the regular list has been processed. Then, do not check the related data.
	for (Sensor* pending_sensor : interface->pending_sensors) {
		if (pending_sensor->ID == sensor->ID) return true;
	}

	// Check read index of first data
	SensorData* sensor_data = &(sensor->data[0]);
	if (sensor_data->read_index > 0) return false;

	return true;
}

/**
 * @brief Check the group termination in a pending point of view
 *
 * @param input interface: the interface pointer
 * @param input sensor_group: sensor group to check
 * @return true if no sensor left to read in the pending list in the sensor group
 */
bool Sensors::PendingGroupTermination(I2CInterface* interface, SensorGroup* sensor_group) {
	// Check that no sensor with the same ID is found in the pending list
	for (Sensor* pending_sensor : interface->pending_sensors) {
		for (Sensor &registered_sensor : sensor_group->sensors) {
			if (pending_sensor->ID == registered_sensor.ID) return false;
		}
	}
	return true;
}

/**
 * @brief Find an interface from ID
 *
 * @param input ID: interface ID to find
 * @param output success: whether successful
 * @return the interface pointer
 */
I2CInterface* Sensors::FindInterface(uint8_t ID, bool* success) {
	uint8_t index = FindInterfaceIndex(ID, success);
	if (not *success)
		return NULL;

	I2CInterface* interface = &(interfaces[index]);

	return interface;
}

/**
 * @brief Find an interface index from ID
 *
 * @param input ID: interface ID to find
 * @param output success: whether successful
 * @return the interface index
 */
uint8_t Sensors::FindInterfaceIndex(uint8_t ID, bool* success) {
	uint8_t index = 0;
	for (I2CInterface &interface : interfaces) {
		if (interface.ID == ID) {
			*success = true;
			return index;
		}
		index++;
	}
	*success = false;
	return 0;
}

/**
 * @brief Find an interface index from I2C hardware interface handle
 *
 * @param input interface: the I2C hardware interface handle to find
 * @param output success: whether successful
 * @return the interface index
 */
uint8_t Sensors::FindInterfaceIndexWithHandle(I2C_HandleTypeDef* interface, bool* success) {
	uint8_t index = 0;
	for (I2CInterface &I2C_interface : interfaces) {
		if (I2C_interface.interface == interface) {
			*success = true;
			return index;
		}
		index++;
	}
	*success = false;
	return 0;
}

/**
 * @brief Find a sensor group
 *
 * @param input group_ID: the group ID to find
 * @param output success: whether successful
 * @return the sensor group pointer
 */
SensorGroup* Sensors::FindSensorGroup(uint8_t group_ID, bool* success) {
	return FindSensorGroupPath(group_ID, NULL, success);
}

/**
 * @brief Find a sensor
 *
 * @param input sensor_ID: the sensor ID to find
 * @param output success: whether successful
 * @return the sensor pointer
 */
Sensor* Sensors::FindSensor(uint8_t sensor_ID, bool* success) {
	return FindSensorPath(sensor_ID, NULL, NULL, success);
}

/**
 * @brief Find a sensor data
 *
 * @param input sensor_ID: the sensor ID with the sensor data to find
 * @param input sensor_register: the sensor data register address to find
 * @param output success: whether successful
 * @return the sensor data pointer
 */
SensorData* Sensors::FindSensorData(uint8_t sensor_ID, uint16_t sensor_register, bool* success) {
	Sensor* sensor = FindSensor(sensor_ID, success);
	if (not *success) return NULL;

	for (SensorData &data : sensor->data) {
		if (data.register_address == sensor_register) {
			SensorData* data_pointer = &data;
			*success = true;
			return data_pointer;
		}
	}
	*success = false;
	return NULL;
}

/**
 * @brief Find the full path of a sensor group
 *
 * @param input group_ID: the group ID to find
 * @param output interface_: interface linked to the sensor group
 * @param output success: whether successful
 * @return the sensor group pointer
 */
SensorGroup* Sensors::FindSensorGroupPath(uint8_t group_ID, I2CInterface** interface_, bool* success) {
	for (I2CInterface &interface : interfaces) {
		for (SensorGroup &sensor_group : interface.sensor_groups) {
			if (sensor_group.ID == group_ID) {
				SensorGroup* sensor_group_pointer = &sensor_group;
				if (interface_ != NULL) *interface_ = &interface;
				*success = true;
				return sensor_group_pointer;
			}
		}
	}
	*success = false;
	return NULL;
}

/**
 * @brief Find the full path of a sensor
 *
 * @param input sensor_ID: the sensor ID to find
 * @param output interface_: interface linked to the sensor
 * @param output sensor_group_: the sensor group linked to the sensor
 * @param output success: whether successful
 * @return the sensor pointer
 */
Sensor* Sensors::FindSensorPath(uint8_t sensor_ID, I2CInterface** interface_, SensorGroup** sensor_group_, bool* success) {
	for (I2CInterface &interface : interfaces) {
		for (SensorGroup &sensor_group : interface.sensor_groups) {
			for (Sensor &sensor : sensor_group.sensors) {
				if (sensor.ID == sensor_ID) {
					Sensor* sensor_pointer = &sensor;
					if (interface_ != NULL) *interface_ = &interface;
					if (sensor_group_ != NULL) *sensor_group_ = &sensor_group;
					*success = true;
					return sensor_pointer;
				}
			}
		}
	}
	*success = false;
	return NULL;
}

/**
 * @brief Check whether the sensor is in the pending list
 *
 * @param input interface: interface linked to the sensor
 * @param input sensor_ID: sensor ID to check
 * @return true if the sensor is part of the pending list
 */
bool Sensors::IsSensorPending(I2CInterface* interface, uint8_t sensor_ID) {
	for (uint8_t index = 0; index < interface->pending_sensors.size(); index++) {
		if (interface->pending_sensors[index]->ID == sensor_ID) {
			return true;
		}
	}
	return false;
}

/**
 * @brief Check whether the sensor is part of the request list
 *
 * @param input interface: interface linked to the sensor
 * @param input sensor_ID: sensor ID to check
 * @return true if there is a data linked to sensor in the request list
 */
bool Sensors::IsSensorRequested(I2CInterface* interface, uint8_t sensor_ID) {
	bool success;
	for (uint8_t request_index = 0; request_index < interface->pending_requests.size(); request_index++) {
		Sensor* sensor = FindSensor(interface->pending_requests[request_index].data.parent, &success);
		if (!success) continue;

		if (sensor->ID == sensor_ID) {
			return true;
		}
	}
	return false;
}

/**
 * @brief Select the next group
 *
 * @param input interface: the interface to select the next group
 * @param input reset: true to reset the interface reading index if interface terminated
 * @return false if no other group left
 */
bool Sensors::MoveToNextGroup(I2CInterface* interface, bool reset) {
	interface->read_index += 1;
	if (interface->read_index < interface->sensor_groups.size())
		return true;

	if (reset) interface->read_index = 0;
	return false;
}

/**
 * @brief Select the next sensor. Process group termination at group termination
 *
 * @param input interface: the interface
 * @param input sensor_group: the sensor group to select the next sensor
 * @param input reset: true to reset the group reading index and process group termination if group terminated
 * @return false if no other sensor left
 */
bool Sensors::MoveToNextSensor(I2CInterface* interface, SensorGroup* sensor_group, bool reset) {
	sensor_group->read_index += 1;
	if (sensor_group->read_index < sensor_group->sensors.size())
		return true;

	if (reset) {
		sensor_group->read_index = 0;
		ProccessGroupTermination(interface, sensor_group);
	}
	return false;
}

/**
 * @brief Select the next sensor data. Process callback at data termination
 *
 * @param input sensor: the sensor to select the next data
 * @param input reset: true to reset the sensor reading index, stop sensor processing and process sensor callback if sensor terminated
 * @return false if no other sensor data left
 */
bool Sensors::MoveToNextData(Sensor* sensor, bool reset) {
	sensor->read_index += 1;
	if (sensor->read_index < sensor->data.size()) {
		SensorData* sensor_data = &(sensor->data[sensor->read_index]);
		if (sensor_data->sequence.size() == 0) return true;
		I2CAccess* I2C_access = &(sensor_data->sequence[0]);
		if (I2C_access->access_mode == DELAY_MODE)
			sensor->timestamp = HAL_GetTick() + sensor_data->sequence[sensor_data->read_index].delay;
		return true;
	}

	if (reset) {
		sensor->processing = false;
		sensor->read_index = 0;

		// Process the callback
		if (sensor->callback != NULL) sensor->callback(sensor->context);
	}
	return false;
}

/**
 * @brief Select the next access
 *
 * @param input sensor_data: the sensor data to select the next access
 * @param input reset: true to reset the sensor data reading index if sensor data terminated
 * @return false if no other access left
 */
bool Sensors::MoveToNextSequence(SensorData* sensor_data, bool reset) {
	sensor_data->read_index += 1;

	if (sensor_data->read_index < sensor_data->sequence.size()) {
		// If the next sequence is a Delay, set the end timestamp
		I2CAccess* I2C_access = &(sensor_data->sequence[sensor_data->read_index]);
		if (I2C_access->access_mode == DELAY_MODE) {
			bool success;
			Sensor* sensor = FindSensor(sensor_data->parent, &success);
			if (success)
				sensor->timestamp = HAL_GetTick() + sensor_data->sequence[sensor_data->read_index].delay;
		}
		return true;
	}

	if (reset)
		sensor_data->read_index = 0;
	return false;
}
