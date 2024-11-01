///////////////////
// Configuration //
///////////////////
//#define READ_SENSORS

#define USE_ADCs

////////////////////////
// Sensors Parameters //
////////////////////////
#define DEFAULT_SENSORS_PERIOD			10	 	//ms
#define DIVISION_STAGE_5V				2		// division stage for the 5V input

///////////////
// GROUP IDs //
///////////////
#define VOLTAGES						0

////////////////
// SENSOR IDs //
////////////////
#define SENSOR_ADC						0

//////////////////////
// GROUP Prescalers //
//////////////////////
#define VOLTAGES_PRESCALER				1

//////////////////////////////
// DEFAULT ACTIVATED GROUPS //
//////////////////////////////
#define VOLTAGES_ACTIVE

///////////////////////////////
// DEFAULT ACTIVATED SENSORS //
///////////////////////////////
#define SENSOR_ADC_ACTIVE

////////////////////////////
// DEFAULT ACTIVATED DATA //
////////////////////////////
// SENSOR_ADC
#define MEAS_3_3V_ACTIVE
#define MEAS_5V_ACTIVE

///////////////////////
// I2C Communication //
///////////////////////
//#define USE_UINT8_I2C_COMMUNICATION
//#define USE_UINT16_I2C_COMMUNICATION
//#define USE_UINT32_I2C_COMMUNICATION
//#define USE_UINT64_I2C_COMMUNICATION
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
//#define USE_UINT16_SENSOR_FUNCTION
//#define USE_UINT32_SENSOR_FUNCTION
//#define USE_UINT64_SENSOR_FUNCTION
//#define USE_INT8_SENSOR_FUNCTION
//#define USE_INT16_SENSOR_FUNCTION
//#define USE_INT32_SENSOR_FUNCTION
//#define USE_INT64_SENSOR_FUNCTION
#define USE_FLOAT_SENSOR_FUNCTION
//#define USE_DOUBLE_SENSOR_FUNCTION
