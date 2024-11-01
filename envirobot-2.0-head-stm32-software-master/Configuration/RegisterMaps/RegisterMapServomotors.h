#include "Definitions/RegistersDefinition.h"

// Registers
#define REG_SERVO_STATE							0x0120 // uint8_t
#define REG_SERVO_GET_SETPOINTS					0x0121 // float 	array 2 	read only	(0=servo1, 1=servo2)
#define REG_SERVO_OFFSET						0x0122 // uint16_t	array 2 				(write: 0=index, 1=value, read: 0=servo1, 1=servo2)
#define REG_SERVO_MOVEFROMTO					0x0123 // float 	array 3 	write only 	(0=index, 1=start, 2=target)
#define REG_SERVO_MOVETO						0x0124 // float 	array 2 	write only 	(0=index, 1=target)
#define REG_SERVO_SYMMETRIC_MOVE				0x0125 // float 	array 2 	write only 	(0=start, 1=target)
#define REG_SERVO_ASYMMETRIC_MOVE				0x0126 // float 	array 2 	write only 	(0=start, 1=target)
#define REG_SERVO_TRAJECTORY_MODE				0x0127 // uint8_t 	array 2 				(write: 0=index, 1=value, read: 0=servo1, 1=servo2)
#define REG_SERVO_TRAJECTORY_CYCLIC				0x0128 // uint8_t 	array 2 				(write: 0=index, 1=value, read: 0=servo1, 1=servo2)
#define REG_SERVO_PLAN_TRAJECTORY				0x0129 // float 	array 3 	write only 	(0=index, 1=start, 2=target)
#define REG_SERVO_START_TRAJECTORY				0x012A // uint8_t 				write only 	(input = index)
#define REG_SERVO_MAX_SPEED						0x012B // float 	array 2 				(write: 0=index, 1=value, read: 0=servo1, 1=servo2)
#define REG_SERVO_MAX_ACCELERATION				0x012C // float 	array 2 				(write: 0=index, 1=value, read: 0=servo1, 1=servo2)
#define REG_SERVO_SIN_FREQUENCY					0x012D // float 	array 2 				(write: 0=index, 1=value, read: 0=servo1, 1=servo2)
#define REG_SERVO_SIN_PHASE						0x012E // float 	array 2 				(write: 0=index, 1=value, read: 0=servo1, 1=servo2)

// Indexes
#define ALL_SERVOS 	0
#define SERVO1		1
#define SERVO2		2
