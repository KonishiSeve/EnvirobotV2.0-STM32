#include "Definitions/RegistersDefinition.h"

// Registers
#define REG_SERVO_STATE							0x0130 // uint8_t
#define REG_SERVO_GET_SETPOINTS					0x0131 // float 	array 2 	read only	(0=servo1, 1=servo2)
#define REG_SERVO_OFFSET						0x0132 // uint16_t	array 2 				(write: 0=index, 1=value, read: 0=servo1, 1=servo2)
#define REG_SERVO_MOVEFROMTO					0x0133 // float 	array 3 	write only 	(0=index, 1=start, 2=target)
#define REG_SERVO_MOVETO						0x0134 // float 	array 2 	write only 	(0=index, 1=target)
#define REG_SERVO_SYMMETRIC_MOVE				0x0135 // float 	array 2 	write only 	(0=start, 1=target)
#define REG_SERVO_ASYMMETRIC_MOVE				0x0136 // float 	array 2 	write only 	(0=start, 1=target)
#define REG_SERVO_TRAJECTORY_MODE				0x0137 // uint8_t 	array 2 				(write: 0=index, 1=value, read: 0=servo1, 1=servo2)
#define REG_SERVO_TRAJECTORY_CYCLIC				0x0138 // uint8_t 	array 2 				(write: 0=index, 1=value, read: 0=servo1, 1=servo2)
#define REG_SERVO_PLAN_TRAJECTORY				0x0139 // float 	array 3 	write only 	(0=index, 1=start, 2=target)
#define REG_SERVO_START_TRAJECTORY				0x013A // uint8_t 				write only 	(input = index)
#define REG_SERVO_MAX_SPEED						0x013B // float 	array 2 				(write: 0=index, 1=value, read: 0=servo1, 1=servo2)
#define REG_SERVO_MAX_ACCELERATION				0x013C // float 	array 2 				(write: 0=index, 1=value, read: 0=servo1, 1=servo2)
#define REG_SERVO_SIN_FREQUENCY					0x013D // float 	array 2 				(write: 0=index, 1=value, read: 0=servo1, 1=servo2)
#define REG_SERVO_SIN_PHASE						0x013E // float 	array 2 				(write: 0=index, 1=value, read: 0=servo1, 1=servo2)

// Indexes
#define ALL_SERVOS 	0
#define SERVO1		1
#define SERVO2		2
