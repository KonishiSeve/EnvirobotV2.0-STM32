#include "Definitions/RegistersDefinition.h"

// Registers
#define REG_CONTROLLER_RESET					0x0100 // uint8_t 				write only  (unusued input)
#define REG_BRIDGE_STATE						0x0101 // uint8_t
#define REG_CONTROLLER_STATE					0x0102 // uint8_t
#define REG_CONTROLLER_PERIOD					0x0103 // uint32_t
#define REG_CONTROLLER_MODE						0x0104 // uint8_t
#define REG_CONTROLLER_INPUT_FILTER				0x0105 // uint8_t
#define REG_CONTROLLER_SET_CONFIGURATION		0x0106 // float 	array 2 	write only 	(0=index, 1=value)
#define REG_CONTROLLER_GET_CONFIGURATION		0x0107 // float 	array 9 	read only
#define REG_CONTROLLER_INTEGRATION_MODE 		0x0108 // uint8_t
#define REG_CONTROLLER_DERIVATION_MODE			0x0109 // uint8_t
#define REG_MOTOR_ENCODER_THRESHOLD_SECURITY	0x010A // uint32_t
#define REG_MOTOR_SETZERO						0x010B // uint8_t 				write only 	(unusued input)
#define REG_MOTOR_CURRENT_COMPENSATION			0x010C // float
#define REG_MOTOR_ENCODER						0x010D // float 				read only
#define REG_MOTOR_SETPOINT						0x010E // float 				read only
#define REG_MOTOR_HBRIDGE_VOLTAGE				0x011F // float 				read only
#define REG_MOTOR_HBRIDGE_CURRENT				0x0110 // float 				read only
#define REG_MOTOR_HBRIDGE_POWER					0x0111 // float 				read only
#define REG_MOTOR_MOVEFROMTO					0x0112 // float 	array 2 	write only 	(0=start, 1=target)
#define REG_MOTOR_MOVETO						0x0113 // float 				write only
#define REG_MOTOR_SET_PARAMETER					0x0114 // float 	array 2 				(0=index, 1=value)
#define REG_MOTOR_GET_PARAMETERS				0x0115 // float 	vector 		read only
#define REG_MOTOR_TRAJECTORY_MODE				0x0116 // uint8_t
#define REG_MOTOR_TRAJECTORY_CYCLIC				0x0117 // uint8_t
#define REG_MOTOR_PLAN_TRAJECTORY				0x0118 // float 	array 2 	write only 	(0=start, 1=target)
#define REG_MOTOR_START_TRAJECTORY				0x0119 // uint8_t 				write only 	(unusued input)
#define REG_MOTOR_MAX_SPEED						0x0120 // float
#define REG_MOTOR_MAX_ACCELERATION				0x0121 // float
#define REG_MOTOR_SIN_FREQUENCY					0x0122 // float
#define REG_MOTOR_SIN_PHASE						0x0123 // float

// REG_CONTROLLER_MODE values
#define POSITION_MODE 0
#define TORQUE_MODE 1
#define PWM_MODE 2

// REG_CONTROLLER_INPUT_FILTER value for POSITION_MODE
#define OUTPUT_MOTOR_POSITION_FILTER 	0 	// position in ° after reductor
#define INPUT_MOTOR_POSITION_FILTER 	1 	// position in ° before reductor
// REG_CONTROLLER_INPUT_FILTER value for TORQUE_MODE
#define CURRENT_FILTER					0 	// current in A
#define TORQUE_FILTER					1 	// torque in mNm before reductor
#define TORQUE_REDUCTOR_FILTER			2 	// torque in mNm after reductor

// REG_CONTROLLER_SET_CONFIGURATION and REG_CONTROLLER_GET_CONFIGURATION indexes
#define CONTROLLER_CONFIGURATION_K 				0
#define CONTROLLER_CONFIGURATION_Ti 			1
#define CONTROLLER_CONFIGURATION_Td 			2
#define CONTROLLER_CONFIGURATION_ISAT		 	3
#define CONTROLLER_CONFIGURATION_P_ACTIVE 		4
#define CONTROLLER_CONFIGURATION_I_ACTIVE 		5
#define CONTROLLER_CONFIGURATION_D_ACTIVE 		6
#define CONTROLLER_CONFIGURATION_S_ACTIVE 		7
#define CONTROLLER_CONFIGURATION_MODEL_ACTIVE	8

// REG_CONTROLLER_INTEGRATION_MODE values
#define INT_RECT 0
#define INT_TRAP 1

// REG_CONTROLLER_DERIVATION_MODE values
#define DER_EULER_1T 0
#define DER_EULER_2T 1

// REG_MOTOR_TRAJECTORY_MODE values
#define TRAJECTORY_STEP 0
#define TRAJECTORY_SLOPE 1
#define TRAJECTORY_TRAPEZOIDAL 2
#define TRAJECTORY_SINUSOIDAL 3

// REG_MOTOR_SET_PARAMETER and REG_MOTOR_GET_PARAMETERS indexes
#define MOTOR_REDUCTION_RATIO 			0
#define MOTOR_ELECTRIC_RESISTOR 		1
#define MOTOR_SPEED_CONSTANT	 		2
#define MOTOR_TORQUE_CONSTANT_INVERSE 	3
#define MOTOR_ENCODER_TICK_NUMBER 		4
