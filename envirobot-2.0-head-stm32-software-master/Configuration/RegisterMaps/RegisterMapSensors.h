#include "Definitions/RegistersDefinition.h"

#define REG_SENSORS_READING_STATUS		0x0200 // uint8_t
#define REG_SENSORS_GROUP_STATUS		0x0201 // uint8_t	array 2		write only	(0=index, 1=value)
#define REG_SENSORS_DEACTIVATE_GROUPS	0x0202 // uint8_t				write only	(unused input)
#define REG_SENSORS_SENSOR_STATUS		0x0203 // uint8_t	array 2		write only	(0=index, 1=value)
#define REG_SENSORS_DEACTIVATE_SENSORS	0x0204 // uint8_t				write only	(0=sensor_group)
#define REG_SENSORS_DATA_STATUS			0x0205 // uint16_t	array 3		write only	(0=index, 1=register, 2=value)
#define REG_SENSORS_DEACTIVATE_ALL_DATA	0x0206 // uint8_t				write only	(0=sensor)
#define REG_SENSORS_GROUP_PRESCALER		0x0207 // uint16_t  array 2		write only	(0=index, 1=value)
#define REG_SENSORS_DETECT_SENSORS		0x0208 // uint32_t 	array 2		write only  (0=trials, 1=timeout)

#define REG_3_3V						0x0210 // float					read only
#define REG_5V							0x0211 // float					read only

#define REG_PRESSURE_SENSOR_P1_RAW		0x0212 // uint32_t 				read only
#define REG_PRESSURE_SENSOR_P2_RAW		0x0213 // uint32_t 				read only
#define REG_PRESSURE_SENSOR_T1_RAW		0x0214 // uint32_t 				read only
#define REG_PRESSURE_SENSOR_T2_RAW		0x0215 // uint32_t 				read only
#define REG_PRESSURE_SENSOR_P1			0x0216 // int32_t				read only
#define REG_PRESSURE_SENSOR_P2			0x0217 // int32_t 				read only
#define REG_PRESSURE_SENSOR_T1			0x0218 // int32_t 				read only
#define REG_PRESSURE_SENSOR_T2			0x0219 // int32_t 				read only

#define REG_CELL1_RAW					0x021A // uint16_t 				read only
#define REG_CELL2_RAW					0x021B // uint16_t 				read only
#define REG_IBAT_RAW					0x021C // uint16_t 				read only
#define REG_TBAT_RAW					0x021D // uint16_t 				read only
#define REG_VCELL1						0x021E // float 				read only
#define REG_VCELL2 						0x021F // float 				read only
#define REG_VBAT						0x0220 // float 				read only
#define REG_IBAT						0x0221 // float 				read only
#define REG_TBAT						0x0222 // float 				read only

#define REG_MOTOR_VOLTAGE_RAW			0x0223 // uint16_t 				read only
#define REG_MOTOR_CURRENT_RAW			0x0224 // uint32_t 				read only
#define REG_MOTOR_POWER_RAW				0x0225 // uint32_t				read only
#define REG_MOTOR_ENERGY_RAW			0x0226 // uint64_t				read only
#define REG_MOTOR_TEMPERATURE_RAW		0x0227 // uint16_t				read only
#define REG_MOTOR_VOLTAGE				0x0228 // float					read only
#define REG_MOTOR_CURRENT				0x0229 // float					read only
#define REG_MOTOR_POWER					0x022A // float					read only
#define REG_MOTOR_ENERGY_DOUBLE			0x022B // double				read only
#define REG_MOTOR_ENERGY_FLOAT			0x022C // float					read only
#define REG_MOTOR_TEMPERATURE			0x022D // float					read only
