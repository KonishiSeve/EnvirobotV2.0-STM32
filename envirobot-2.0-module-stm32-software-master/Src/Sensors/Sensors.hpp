/*
 * Sensors.hpp
 *
 *  Created on: 14 déc. 2022
 *      Author: bignet
 */

#pragma once

#include <vector>
#include <stm32h750xx.h>
#include <stm32h7xx_hal.h>
#include "cmsis_os.h"

#include "Registers/Registers.hpp"
#include "Publishers/Publishers.hpp"
#include "HardwareDelay/HardwareDelay.hpp"
#include "LEDS/LEDS.hpp"

#include "Definitions/SensorsDefinition.h"
#include "RegisterMaps/RegisterMapSensors.h"
#include "Configurations/SensorsConfiguration.h"
#include "Configurations/RegistersConfiguration.h"

#define VREFINT_CAL 						((uint16_t*) VREFINT_CAL_ADDR)		// memory address of the calibration data for the internal reference

#define PENDING_SENSORS_SIZE 				5		// maximum number of pending sensors
#define SENSOR_WRITE_SIZE 					64		// size in bytes of the write buffer in sequence
#define SENSOR_BUFFER_SIZE 					64		// size in bytes of the read buffer
#define SENSOR_MAX_REGISTER_NB_PER_TYPE 	20		// maximum number of sensor data per type

#define MEM_WRITE_MODE 		0
#define MEM_READ_MODE 		1 // a read will writes the data into the buffer allocated to the data. The next read will shift the already registered data into the buffer by the 8xsize of the access
#define WRITE_MODE 			2
#define READ_MODE 			3 // a read will writes the data into the buffer allocated to the data. Another read will shift the already registered data into the buffer by the 8xsize of the access
#define DELAY_MODE 			4 // a delay will skip the current sensor
#define CODE_MODE			5

#define REGULAR_ACCESS		0
#define PENDING_ACCESS		1
#define REQUEST_ACCESS		2

struct I2CAccess;
struct Sensor;
struct SensorGroup;
struct I2CInterface;

template <typename T>
struct SensorFunction {
	void* context;
	T (*function)(void*);
};

// access struct from sequence
struct I2CAccess {
	uint8_t access_mode;
	uint16_t size;

	// MEM_WRITE_MODE and MEM_READ_MODE parameters
	uint16_t memory_address;

	// WRITE_MODE and MEM_WRITE_MODE parameters
	uint8_t write_data[SENSOR_WRITE_SIZE];

	// DELAY_MODE parameters
	uint32_t delay;

	// CODE_MODE parameters
	uint8_t function_index;
};

// Sensor data struct
struct SensorData {
	bool active;							// whether active or not
	uint8_t parent;							// sensor parent ID
	bool publishable;						// if true, then a topic is added to the group publisher if the group is also publishable
	uint16_t register_address;				// sensor data register address from Registers
	uint8_t index; 							// data index in sensor_data buffers
	uint8_t type;							// register type
	uint8_t read_index;						// sequence read index
	std::vector<I2CAccess> sequence;		// sequence of access
};

// Sensor struct
struct Sensor {
	bool active;							// whether active or not
	uint8_t parent;							// group parent ID
	bool present;							// true if the sensor has been detected
	bool processing;						// true if the sensor is being processed
	uint8_t ID;								// sensor ID
	uint16_t address;						// I2C device address. Not used if not an I2C sensor
	uint16_t memory_address_size;			// Size in byte of the memory address of the sensor. Not the device address size.
	uint32_t timestamp;						// timestamp used to process timeout
	uint8_t read_index;						// sensor data read index
	std::vector<SensorData> data;			// vector of data to be read
	void* context;							// context to forward to the sensor callback
	void (*callback)(void*);				// callback to be processed when all data have been read
};

// SensorGroup struct
struct SensorGroup {
	bool active;							// whether active or not
	uint8_t parent;							// interface parent ID
	uint8_t ID;								// group ID. It is unique and can't be used on other interfaces
	bool publishable; 						// if true, a publisher is automatically created with the same group ID
	uint16_t prescaler;						// prescaler threshold
	uint16_t counter;						// prescaler counter
	uint8_t read_index;						// sensor read index
	std::vector<Sensor> sensors;			// vector of sensors
};

// ReadRequest struct
struct ReadRequest {
	bool processing;						// true if the request is being processed
	bool* flag;								// request termination flag pointer to exit the blocking loop in RequestRead
	SensorData data;						// Data to read
};

// I2CInterface struct
struct I2CInterface {
	uint8_t ID;									// interface ID
	bool request_regular_process;				// flag to start the regular loop when ReadNextSensor is called
	bool error_flag;							// error flag to force the process of the current access with Spin
	uint32_t activity_timestamp;				// last activity timestamp to detect reading blockage
	uint32_t timeout;							// time left to force the process of the current access with Spin

	I2C_HandleTypeDef* interface;				// hardware interface handle
	uint8_t buffer[SENSOR_BUFFER_SIZE];			// read buffer array for each sensor

	uint8_t current_group_ID;					// current selected group. UINT8_MAX if no group selected
	uint8_t current_sensor_ID;					// current selected sensor. UINT8_MAX if no sensor selected
	uint16_t current_data_address;				// current selected data. UINT16_MAX if no data selected

	uint8_t access_source;						// specify which part of the code process an access, either regular, pending or request
	uint8_t read_index;							// group read index
	uint8_t pending_index;						// pending sensor index that process an access
	uint8_t request_index;						// request data index that process an access
	std::vector<SensorGroup> sensor_groups;		// vector of regular groups
	std::vector<Sensor*> pending_sensors; 		// vector of pending sensor pointers
	std::vector<ReadRequest> pending_requests;	// vector of read requests
};

// Sensors class to manage reading of I2C and internal sensors. It is compatible with almost every I2C sensor and does optimize the reading time by avoiding waiting times as much as possible.
class Sensors {
public:
	Sensors();
	void Init(Registers* registers, Publishers* publishers, HardwareDelay* hardware_delay, LEDS* leds);
	void AddRegisters(void);

	bool SetReadingStatus(bool status);
	bool ActivateReading();
	bool DeactivateReading();

	bool AddInterface(uint8_t ID, I2C_HandleTypeDef* interface);
	bool SetInterfaceTimeout(uint8_t ID, uint32_t timeout);

	bool AddSensorGroup(uint8_t ID, uint8_t group_ID, bool publishable = true);
	bool SetSensorGroupStatus(uint8_t group_ID, bool status);
	bool ActivateSensorGroup(uint8_t group_ID);
	bool DeactivateSensorGroup(uint8_t group_ID);
	bool DeactivateAllSensorGroups(void);
	bool SetSensorGroupPrescaler(uint8_t group_ID, uint16_t prescaler);

	bool AddSensor(uint8_t group_ID, uint8_t sensor_ID, uint16_t address);
	bool AddSensorCallback(uint8_t sensor_ID, void* context, void (*callback)(void*));
	bool SetSensorStatus(uint8_t sensor_ID, bool status);
	bool ActivateSensor(uint8_t sensor_ID);
	bool DeactivateSensor(uint8_t sensor_ID);
	bool DeactivateAllSensors(uint8_t group_ID);
	bool SetSensorMemoryAddressSize(uint8_t sensor_ID, uint16_t memory_address_size);
	HAL_StatusTypeDef ConfigureSensor(uint8_t sensor_ID, uint8_t data, uint32_t timeout = HAL_MAX_DELAY);
	HAL_StatusTypeDef ConfigureSensor(uint8_t sensor_ID, uint8_t* data, uint16_t size, uint32_t timeout = HAL_MAX_DELAY);
	HAL_StatusTypeDef ConfigureSensor(uint8_t sensor_ID, uint16_t memory_address, uint8_t data, uint32_t timeout = HAL_MAX_DELAY);
	HAL_StatusTypeDef ConfigureSensor(uint8_t sensor_ID, uint16_t memory_address, uint8_t* data, uint16_t size, uint32_t timeout = HAL_MAX_DELAY);

	template<typename T>
	T ReadSensor(uint8_t sensor_ID, HAL_StatusTypeDef* status, uint32_t timeout = HAL_MAX_DELAY);
	template<typename T>
	T ReadSensor(uint8_t sensor_ID, uint16_t memory_address, HAL_StatusTypeDef* status, uint32_t timeout = HAL_MAX_DELAY);

	template<typename T>
	bool AddSensorData(uint8_t sensor_ID, uint16_t sensor_register/*, uint8_t size*/, bool publishable = true);
	bool IsSensorDataActive(uint8_t sensor_ID, uint16_t sensor_register);
	bool SetSensorDataStatus(uint8_t sensor_ID, uint16_t sensor_register, bool status);
	bool ActivateSensorData(uint8_t sensor_ID, uint16_t sensor_register);
	bool DeactivateSensorData(uint8_t sensor_ID, uint16_t sensor_register);
	bool DeactivateAllSensorData(uint8_t sensor_ID);

	bool AddMemoryWriteSequence(uint8_t sensor_ID, uint16_t sensor_register, uint16_t memory_address, uint8_t write_data);
	bool AddMemoryWriteSequence(uint8_t sensor_ID, uint16_t sensor_register, uint16_t memory_address, uint8_t write_data[], uint16_t size);
	bool AddMemoryReadSequence(uint8_t sensor_ID, uint16_t sensor_register, uint16_t memory_address, uint16_t size);
	bool AddWriteSequence(uint8_t sensor_ID, uint16_t sensor_register, uint8_t write_data);
	bool AddWriteSequence(uint8_t sensor_ID, uint16_t sensor_register, uint8_t write_data[], uint16_t size);
	bool AddReadSequence(uint8_t sensor_ID, uint16_t sensor_register, uint16_t size);
	bool AddDelaySequence(uint8_t sensor_ID, uint16_t sensor_register, uint32_t delay);

	template<typename T>
	bool AddCodeSequence(uint8_t sensor_ID, uint16_t sensor_register, void* context, T (*function)(void*), uint16_t size = sizeof(T));

	void DetectSensors(uint32_t trials, uint32_t timeout = HAL_MAX_DELAY);
	bool ForcePresence(uint8_t sensor_ID);
	void Spin(void);
	void ReceiveSensorData(I2C_HandleTypeDef* interface);

	template<typename T>
	T RequestRead(uint8_t sensor_ID, uint16_t sensor_register, bool* success, uint32_t timeout = HAL_MAX_DELAY);

	// OS
	osSemaphoreId_t SensorsSemaphore;

private:
	bool AddSensorDataWithType(uint8_t sensor_ID, uint16_t sensor_register, uint8_t type, bool publishable = true);

	I2CInterface* FindInterface(uint8_t ID, bool* success);
	uint8_t FindInterfaceIndex(uint8_t ID, bool* success);
	uint8_t FindInterfaceIndexWithHandle(I2C_HandleTypeDef* interface, bool* success);

	SensorGroup* FindSensorGroup(uint8_t group_ID, bool* success);
	Sensor* FindSensor(uint8_t sensor_ID, bool* success);
	SensorData* FindSensorData(uint8_t sensor_ID, uint16_t sensor_register, bool* success);

	SensorGroup* FindSensorGroupPath(uint8_t group_ID, I2CInterface** interface, bool* success);
	Sensor* FindSensorPath(uint8_t sensor_ID, I2CInterface** interface, SensorGroup** sensor_group, bool* success);
	bool IsSensorPending(I2CInterface* interface, uint8_t sensor_ID);
	bool IsSensorRequested(I2CInterface* interface, uint8_t sensor_ID);

	void ReadNextSensor(uint8_t ID);
	void ProcessAccess(I2CInterface* interface, SensorGroup* sensor_group, Sensor* sensor, SensorData* sensor_data, I2CAccess* I2C_access);
	void ProccessReceivedData(I2CInterface* interface);
	void ProccessGroupTermination(I2CInterface* interface, SensorGroup* sensor_group);
	static void HardwareDelayISR(void* context, uint8_t ID);
	void ReceiveDelayInterrupt(uint8_t ID);

	template<typename T>
	void TransferBuffer(T *memory_address, uint8_t *buffer, uint16_t size);
	template<typename T>
	void ProcessCode(I2CInterface* interface, SensorData* sensor_data, I2CAccess* I2C_access);

	void ExitRegular(I2CInterface* interface);
	void ExitPending(I2CInterface* interface);

	bool RegularInterfaceTermination(I2CInterface* interface);
	bool RegularGroupTermination(I2CInterface* interface, SensorGroup* sensor_group);
	bool PendingGroupTermination(I2CInterface* interface, SensorGroup* sensor_group);

	bool MoveToNextGroup(I2CInterface* interface, bool reset = true);
	bool MoveToNextSensor(I2CInterface* interface, SensorGroup* sensor_group, bool reset = true);
	bool MoveToNextData(Sensor* sensor, bool reset = true);
	bool MoveToNextSequence(SensorData* sensor_data, bool reset = true);

	bool active;
	uint32_t period_ms;
//	bool delay_flag;
	uint8_t delay_source;		// flag to know which part of the code used a hardware delay
	uint8_t delayed_index;		// index of the element that used a hardware delay

	std::vector<I2CInterface> interfaces;

	// sensor data buffers
#if (defined(USE_UINT8_I2C_COMMUNICATION) || defined(USE_UINT8_SENSOR_FUNCTION)) && defined(USE_UINT8_REGISTER)
	std::vector<uint8_t> sensor_data_uint8;
#endif
#if (defined(USE_UINT16_I2C_COMMUNICATION) || defined(USE_UINT16_SENSOR_FUNCTION)) && defined(USE_UINT16_REGISTER)
	std::vector<uint16_t> sensor_data_uint16;
#endif
#if (defined(USE_UINT32_I2C_COMMUNICATION) || defined(USE_UINT32_SENSOR_FUNCTION)) && defined(USE_UINT32_REGISTER)
	std::vector<uint32_t> sensor_data_uint32;
#endif
#if (defined(USE_UINT64_I2C_COMMUNICATION) || defined(USE_UINT64_SENSOR_FUNCTION)) && defined(USE_UINT64_REGISTER)
	std::vector<uint64_t> sensor_data_uint64;
#endif
#if (defined(USE_INT8_I2C_COMMUNICATION) || defined(USE_INT8_SENSOR_FUNCTION)) && defined(USE_INT8_REGISTER)
	std::vector<int8_t> sensor_data_int8;
#endif
#if (defined(USE_INT16_I2C_COMMUNICATION) || defined(USE_INT16_SENSOR_FUNCTION)) && defined(USE_INT16_REGISTER)
	std::vector<int16_t> sensor_data_int16;
#endif
#if (defined(USE_INT32_I2C_COMMUNICATION) || defined(USE_INT32_SENSOR_FUNCTION)) && defined(USE_INT32_REGISTER)
	std::vector<int32_t> sensor_data_int32;
#endif
#if (defined(USE_INT64_I2C_COMMUNICATION) || defined(USE_INT64_SENSOR_FUNCTION)) && defined(USE_INT64_REGISTER)
	std::vector<int64_t> sensor_data_int64;
#endif
#if (defined(USE_FLOAT_I2C_COMMUNICATION) || defined(USE_FLOAT_SENSOR_FUNCTION)) && defined(USE_FLOAT_REGISTER)
	std::vector<float> sensor_data_float;
#endif
#if (defined(USE_DOUBLE_I2C_COMMUNICATION) || defined(USE_DOUBLE_SENSOR_FUNCTION)) && defined(USE_DOUBLE_REGISTER)
	std::vector<double> sensor_data_double;
#endif

	// sensor code access buffers
	void* functions[NUMBER_OF_TYPES];
#if defined(USE_UINT8_SENSOR_FUNCTION) && defined(USE_UINT8_REGISTER)
	std::vector<SensorFunction<uint8_t>> uint8_t_sensor_functions;
#endif
#if defined(USE_UINT16_SENSOR_FUNCTION) && defined(USE_UINT16_REGISTER)
	std::vector<SensorFunction<uint16_t>> uint16_t_sensor_functions;
#endif
#if defined(USE_UINT32_SENSOR_FUNCTION) && defined(USE_UINT32_REGISTER)
	std::vector<SensorFunction<uint32_t>> uint32_t_sensor_functions;
#endif
#if defined(USE_UINT64_SENSOR_FUNCTION) && defined(USE_UINT64_REGISTER)
	std::vector<SensorFunction<uint64_t>> uint64_t_sensor_functions;
#endif
#if defined(USE_INT8_SENSOR_FUNCTION) && defined(USE_INT8_REGISTER)
	std::vector<SensorFunction<int8_t>> int8_t_sensor_functions;
#endif
#if defined(USE_INT16_SENSOR_FUNCTION) && defined(USE_INT16_REGISTER)
	std::vector<SensorFunction<int16_t>> int16_t_sensor_functions;
#endif
#if defined(USE_INT32_SENSOR_FUNCTION) && defined(USE_INT32_REGISTER)
	std::vector<SensorFunction<int32_t>> int32_t_sensor_functions;
#endif
#if defined(USE_INT64_SENSOR_FUNCTION) && defined(USE_INT64_REGISTER)
	std::vector<SensorFunction<int64_t>> int64_t_sensor_functions;
#endif
#if defined(USE_FLOAT_SENSOR_FUNCTION) && defined(USE_FLOAT_REGISTER)
	std::vector<SensorFunction<float>> float_sensor_functions;
#endif
#if defined(USE_DOUBLE_SENSOR_FUNCTION) && defined(USE_DOUBLE_REGISTER)
	std::vector<SensorFunction<double>> double_sensor_functions;
#endif

	Registers* registers;
	Publishers* publishers;
	HardwareDelay* hardware_delay;
	LEDS* leds;
};

/**
 * @brief Add a typed sensor data to a sensor
 *
 * @param input sensor_ID: sensor ID
 * @param input sensor_register: sensor data register address. This address is stored and accessible as a register from Registers. It is a single register by design
 * @param input publishable: if true, add the sensor data register to the sensor group publisher as a topic if it exists. DEFAULT=true
 * @return whether successful
 */
template<typename T>
bool Sensors::AddSensorData(uint8_t sensor_ID, uint16_t sensor_register, bool publishable) {
	bool success;
	uint8_t type = registers->GetTypeID<T>(&success);
	if (!success) return false;

	return AddSensorDataWithType(sensor_ID, sensor_register, type, publishable);
}

/**
 * @brief Request and return the read of a data from a sensor. It is processed as soon as possible. BLOCKING function.
 *
 * @param input sensor_ID: sensor ID
 * @param input sensor_register: sensor data register address to read
 * @param output success: whether successful
 * @param input timeout: time left to read the value from the sensor. DEFAULT=HAL_MAX_DELAY
 * @return the typed data read from the sensor
 */
template<typename T>
T Sensors::RequestRead(uint8_t sensor_ID, uint16_t sensor_register, bool* success, uint32_t timeout) {
	// Find register
	Register target_register = registers->FindRegister(sensor_register, success);
	if (not *success) return 0;

	// Check the type of the register fits the template type
	uint8_t registered_type = registers->GetTypeID<T>(success);
	if (not *success) return 0;
	if (target_register.type != registered_type) {*success = false; return 0;}

	// Find the sensor data, copy the content
	SensorData sensor_data = *FindSensorData(sensor_ID, sensor_register, success);
	if (not *success) return 0;

	// Reset the sequence
	sensor_data.read_index = 0;

	// Return if no sequence
	if (sensor_data.sequence.size() == 0) {*success = false; return 0;}

	// Find interface
	I2CInterface* interface;
	FindSensorGroupPath(sensor_data.parent, &interface, success);
	if (not *success) return 0;

	// Compute timeout timestamp
	uint32_t end_timestamp = HAL_MAX_DELAY;
	if (timeout != HAL_MAX_DELAY)
		end_timestamp = HAL_GetTick() + timeout;

	// Setup the read flag to exit the function
	bool read_flag = false;

	// Setup read request
	ReadRequest read_request;
	read_request.processing = false;
	read_request.flag = &read_flag;
	read_request.data = sensor_data;

	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);

	// Add the read sequence
	interface->pending_requests.push_back(read_request);

	// Execute sequence if regular loop finished and no pending sensor
	if (RegularInterfaceTermination(interface) && interface->pending_sensors.size() == 0)
		ReadNextSensor(interface->ID);

	// Loop unit the read flag is set or that a timeout occurs
	while (1) {
		if (HAL_GetTick() >= end_timestamp) {
			*success = false;
			return 0;
		}

		osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
		if (read_flag) break;
		osSemaphoreRelease(SensorsSemaphore);

		// TODO Add a osDelay to leave space for another task?
	}

	osSemaphoreRelease(SensorsSemaphore);

	// Get the read value
	T sensor_value;
	uint16_t length;
	*success = registers->ReadRegister<T>(sensor_register, &sensor_value, &length);
	if (not *success) return 0;

	// Return read value
	*success = true;
	return sensor_value;
}

/**
 * @brief Add a code access to a sequence
 *
 * @param input sensor_ID: sensor ID
 * @param input sensor_register: sensor data register address
 * @param input context: context to forward to the code access
 * @param input function: function to be processed in the sequence
 * @param input size: number of bytes returned by the function
 * @return whether successful
 */
template<typename T>
bool Sensors::AddCodeSequence(uint8_t sensor_ID, uint16_t sensor_register, void* context, T (*function)(void*), uint16_t size) {
	bool success;
	// Find data
	SensorData* sensor_data = FindSensorData(sensor_ID, sensor_register, &success);
	if (!success) return false;

	// Find template type
	uint8_t type = registers->GetTypeID<T>(&success);
	if (!success) return false;

	// Check the type is supported
	if (functions[type] == NULL)
		return false;

	// Safety checks
	if (size > SENSOR_BUFFER_SIZE) return false;								// Check that the sensor read buffer has enough space to save the read value
	if (size > registers->GetTypeSize(type)) return false;						// Check that the size doesn't exceed the register type size

	// Setup the function
	SensorFunction<T> sensor_function;
	sensor_function.context = context;
	sensor_function.function = function;

	// Find the code buffer
	std::vector<SensorFunction<T>>* reinterpreted_functions;
	reinterpreted_functions = reinterpret_cast <std::vector<SensorFunction<T>>*> (functions[type]);

	// Setup the access
	I2CAccess access;
	access.access_mode = CODE_MODE;
	access.size = size;
	access.function_index = (uint8_t) reinterpreted_functions->size();

	// Add the access to the sequence
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	sensor_data->sequence.push_back(access);
	reinterpreted_functions->push_back(sensor_function);
	osSemaphoreRelease(SensorsSemaphore);
	return true;
}

/**
 * @brief Transfer data from a byte buffer to a typed buffer. Append data from buffer to the memory_address LSB. Previous content is left shifted at each access
 *
 * @param output memory_address: memory pointer to write to
 * @param input buffer: byte array to append to memory_address
 * @param input size: size in bytes of the input buffer
 */
template<typename T>
void Sensors::TransferBuffer(T* memory_address, uint8_t* buffer, uint16_t size) {
	// Get memory_address content into temp
	uint64_t temp = 0;
	std::memcpy(&temp, memory_address, sizeof(T));

	// Shift left the memory_address content to leave the size of buffer free on LSB
	if (temp != 0)
		temp = temp << (size << 3); // * 8 <=> Shift left by 3 to accumulate reads (MSB first)

	// Transfer data from buffer to memory_adress LSB
	for (uint8_t index = 0; index < size; index++) {
		temp |= (buffer[index] << ((size - 1 - index) << 3));
	}

	// Recast temp
	T* reinterpreted_temp = reinterpret_cast<T*>(&temp);

	// Update memory_address content
	*memory_address = *reinterpreted_temp;
}

/**
 * @brief Process a code function from sequence
 *
 * @param input interface: the interface pointer
 * @param input sensor_data: sensor data parent to the access (to avoid looking for it anew)
 * @param input I2C_access: access to process
 */
template<typename T>
void Sensors::ProcessCode(I2CInterface* interface, SensorData* sensor_data, I2CAccess* I2C_access) {
	// Check that the type is supported by custom functions
	if (functions[sensor_data->type] == NULL)
		return;

	// Get the code function from sequence
	std::vector<SensorFunction<T>>* reinterpreted_functions = reinterpret_cast <std::vector<SensorFunction<T>>*> (functions[sensor_data->type]);
	SensorFunction<T> sensor_function = reinterpreted_functions->at(I2C_access->function_index);

	// Process code function
	T code_output = sensor_function.function(sensor_function.context);

	// Transfer function output into byte buffer
	uint8_t array_output[I2C_access->size];
	std::memcpy(&array_output[0], &code_output, I2C_access->size);

	// Transfer to interface buffer. Mirror required
	for (uint8_t index = 0; index < I2C_access->size; index++) {
		interface->buffer[index] = array_output[I2C_access->size - 1 - index];
	}
}

/**
 * @brief I2C read of a sensor for configuration purpose. BLOCKING function
 *
 * @param input sensor_ID: sensor ID
 * @param output status: I2C access status
 * @param input timeout: time left to read the sensor in ms. DEFAULT=HAL_MAX_DELAY
 * @return the read value from sensor
 */
template<typename T>
T Sensors::ReadSensor(uint8_t sensor_ID, HAL_StatusTypeDef* status, uint32_t timeout) {
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
	uint16_t size = sizeof(T);
	uint8_t data[size];
	*status = HAL_BUSY;

	// Read from sensor until I2C bus is free or timeout has elapsed
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	do {
		*status = HAL_I2C_Master_Receive(interface->interface, sensor->address, data, size, timeout);
	} while ((*status == HAL_BUSY) && (HAL_GetTick() < end_timestamp));
	osSemaphoreRelease(SensorsSemaphore);

	// Return the read value
	T result = 0;
	if (*status == HAL_OK)
		TransferBuffer<T>(&result, data, size);
	return result;
}

/**
 * @brief I2C memory read of a sensor for configuration purpose. BLOCKING function
 *
 * @param input sensor_ID: sensor ID
 * @param input memory_address: memory address of the sensor to write to
 * @param output status: I2C access status
 * @param input timeout: time left to read the sensor in ms. DEFAULT=HAL_MAX_DELAY
 * @return the read value from sensor
 */
template<typename T>
T Sensors::ReadSensor(uint8_t sensor_ID, uint16_t memory_address, HAL_StatusTypeDef* status, uint32_t timeout) {
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
	uint16_t size = sizeof(T);
	uint8_t data[size];
	*status = HAL_BUSY;

	// Read from sensor until I2C bus is free or timeout has elapsed
	osSemaphoreAcquire(SensorsSemaphore, osWaitForever);
	do {
		*status = HAL_I2C_Mem_Read(interface->interface, sensor->address, memory_address, sensor->memory_address_size, data, size, timeout);
	} while ((*status == HAL_BUSY) && (HAL_GetTick() < end_timestamp)); // try until the bus is free
	osSemaphoreRelease(SensorsSemaphore);

	// Return the read value
	T result = 0;
	if (*status == HAL_OK)
			TransferBuffer<T>(&result, data, size);
	return result;
}
