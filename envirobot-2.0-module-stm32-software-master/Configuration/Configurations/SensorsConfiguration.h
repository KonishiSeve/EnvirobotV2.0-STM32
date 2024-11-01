///////////////////
// Configuration //
///////////////////
//#define READ_SENSORS

//#define USE_PRESSURE_SENSORS
//#define USE_DS2778
#define USE_LTC2947
//#define USE_ADCs

////////////////////////
// Sensors Parameters //
////////////////////////
#define DEFAULT_SENSORS_PERIOD			10 		//ms
#define DIVISION_STAGE_5V				2		// division stage for the 5V input

///////////////
// GROUP IDs //
///////////////
#define VOLTAGES			0
#define PRESSURE_SENSORS 	1
#define BATTERY			 	2
#define MOTOR				3

////////////////
// SENSOR IDs //
////////////////
#define SENSOR_ADC						0
#define PRESSURE_SENSOR_1 				1
#define PRESSURE_SENSOR_2 				2
#define BATTERY_PROTECTION_DS2778		3
#define ENERGY_MONITORING_LTC2947		4

//////////////////////
// GROUP Prescalers //
//////////////////////
#define VOLTAGES_PRESCALER				100 	// at 1Hz
#define PRESSURE_SENSORS_PRESCALER	 	1		// at 100Hz (careful, there is a delay in the reading so make sure it is smaller than 10ms in total)
#define BATTERY_PRESCALER		 		100  	// at 1Hz
#define MOTOR_PRESCALER					10		// at 10Hz

//////////////////////////////
// DEFAULT ACTIVATED GROUPS //
//////////////////////////////
//#define VOLTAGES_ACTIVE
//#define PRESSURE_SENSORS_ACTIVE
//#define BATTERY_ACTIVE
#define MOTOR_ACTIVE

///////////////////////////////
// DEFAULT ACTIVATED SENSORS //
///////////////////////////////
#define SENSOR_ADC_ACTIVE
#define PRESSURE_SENSOR_1_ACTIVE
#define PRESSURE_SENSOR_2_ACTIVE
#define BATTERY_PROTECTION_DS2778_ACTIVE
#define ENERGY_MONITORING_LTC2947_ACTIVE

////////////////////////////
// DEFAULT ACTIVATED DATA //
////////////////////////////
// SENSOR_ADC
#define MEAS_3_3V_ACTIVE
#define MEAS_5V_ACTIVE

// ENERGY_MONITORING_LTC2947
//#define MEAS_MOTOR_VOLTAGE_RAW
//#define MEAS_MOTOR_CURRENT_RAW
//#define MEAS_MOTOR_POWER_RAW
//#define MEAS_MOTOR_ENERGY_RAW
//#define MEAS_MOTOR_TEMPERATURE_RAW

// BATTERY_PROTECTION_DS2778
#define MEAS_CELL1_RAW
#define MEAS_CELL2_RAW
#define MEAS_IBAT_RAW
#define MEAS_TBAT_RAW

// PRESSURE_SENSORs
#define MEAS_P1_RAW
#define MEAS_P2_RAW
#define MEAS_T1_RAW
#define MEAS_T2_RAW

///////////////////////
// I2C Communication //
///////////////////////
//#define USE_UINT8_I2C_COMMUNICATION
#define USE_UINT16_I2C_COMMUNICATION
#define USE_UINT32_I2C_COMMUNICATION
#define USE_UINT64_I2C_COMMUNICATION
//#define USE_INT8_I2C_COMMUNICATION
//#define USE_INT16_I2C_COMMUNICATION
//#define USE_INT32_I2C_COMMUNICATION
//#define USE_INT64_I2C_COMMUNICATION
//#define USE_FLOAT_I2C_COMMUNICATION
//#define USE_DOUBLE_I2C_COMMUNICATION

//////////////////////
// Sensor Functions //
//////////////////////
//#define USE_UINT8_SENSOR_FUNCTION
#define USE_UINT16_SENSOR_FUNCTION
//#define USE_UINT32_SENSOR_FUNCTION
//#define USE_UINT64_SENSOR_FUNCTION
//#define USE_INT8_SENSOR_FUNCTION
//#define USE_INT16_SENSOR_FUNCTION
//#define USE_INT32_SENSOR_FUNCTION
//#define USE_INT64_SENSOR_FUNCTION
#define USE_FLOAT_SENSOR_FUNCTION
//#define USE_DOUBLE_SENSOR_FUNCTION
