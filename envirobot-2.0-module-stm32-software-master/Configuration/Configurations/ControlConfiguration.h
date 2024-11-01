#include "RegisterMaps/RegisterMapController.h"

///////////////////
// Configuration //
///////////////////
//#define BRIDGE_ACTIVE
//#define CONTROLLER_ACTIVE
//#define SERVOMOTORS_ACTIVE

///////////////////////////
// CONTROLLER PARAMETERS //
///////////////////////////
#define DEFAULT_CONTROLLER_PERIOD 					10 				// ms
#define DEFAULT_CONTROLLER_MODE						POSITION_MODE
#define DEFAULT_ERROR_INTEGRATION_MODE				INT_TRAP
#define DEFAULT_ERROR_DERIVATION_MODE				DER_EULER_1T
#define DEFAULT_SPEED_DERIVATION_MODE				DER_EULER_1T 	// mode to derive the encoder speed
#define DEFAULT_ENCODER_SECURITY_THRESHOLD 			90000			// 0 = inactive

#define DEFAULT_CURRENT_CONVERSION_FACTOR			(375 * 3.3f) / (65536 * 100) // R = 100R, H-bridge 1/375 current feedback in 16-bit ADC
#define DEFAULT_CURRENT_COMPENSATION_FACTOR			1.0215f			// from H-bridge current feedback characterization
#define DEFAULT_REDUCTION_RATIO						138
#define DEFAULT_EQUIVALENT_ELECTRIC_RESISTANCE		13900			// tuned, not computed, theoretically 24000 * R / 8.4V if battery fully charged
#define DEFAULT_EQUIVALENT_SPEED_CONSTANT			0.004f 			// tuned, not computed, theoretically (60 / (DEFAULT_ENCODER_TICK_NUMBER * 4)) * (24000 / 8.4) * (1 / k), k in tr/min/V
#define DEFAULT_TORQUE_CONSTANT_INVERSE				1 / 5.01f
#define DEFAULT_ENCODER_TICK_NUMBER					1024

#define SERVO_MAX_PULSE 							18000
#define SERVO_MIN_PULSE 							9000
#define DEFAULT_SERVO1_OFFSET						13500
#define DEFAULT_SERVO2_OFFSET						12700

////////////////////////////
// POSITION CONFIGURATION //
////////////////////////////
#define DEFAULT_POSITION_FILTER						OUTPUT_MOTOR_POSITION_FILTER
#define DEFAULT_POSITION_K							1.0f
#define DEFAULT_POSITION_Ti_inv						1 / 1.0f
#define DEFAULT_POSITION_Td							0.05f
#define DEFAULT_POSITION_Isat						3000 			// with respect to 24000, the PWM pulse
#define DEFAULT_POSITION_P_ACTIVE					true
#define DEFAULT_POSITION_I_ACTIVE					true
#define DEFAULT_POSITION_D_ACTIVE					true
#define DEFAULT_POSITION_S_ACTIVE					true

//////////////////////////
// TORQUE CONFIGURATION //
//////////////////////////
#define DEFAULT_TORQUE_FILTER						TORQUE_REDUCTOR_FILTER
#define DEFAULT_TORQUE_K							5000
#define DEFAULT_TORQUE_Ti_inv						1 / 0.3f
#define DEFAULT_TORQUE_Td							0.005f
#define DEFAULT_TORQUE_Isat							10000 			// with respect to 24000, the PWM pulse
#define DEFAULT_TORQUE_P_ACTIVE						true
#define DEFAULT_TORQUE_I_ACTIVE						true
#define DEFAULT_TORQUE_D_ACTIVE						false
#define DEFAULT_TORQUE_S_ACTIVE						true
#define DEFAULT_TORQUE_MODEL_ACTIVE					true

///////////////////////////////
// TRAJECTORY CONFIGURATIONS //
///////////////////////////////
#define DEFAULT_MOTOR_TRAJECTORY					TRAJECTORY_STEP
#define DEFAULT_MOTOR_CYCLIC						false
#define DEFAULT_MOTOR_MAX_SPEED						1000.0f			// in pulse/ms with respect to 24000 the PWM max counter
#define DEFAULT_MOTOR_ACCEL_SPEED					1.0f			// in pulse/ms^2 with respect to 24000 the PWM max counter
#define DEFAULT_MOTOR_FREQUENCY						1.0f			// in Hz
#define DEFAULT_MOTOR_PHASE							0				// between 0 and 2*pi

#define DEFAULT_SERVO1_TRAJECTORY					TRAJECTORY_STEP
#define DEFAULT_SERVO1_CYCLIC						false
#define DEFAULT_SERVO1_MAX_SPEED					1000.0f			// in pulse/ms with respect to 24000 the PWM max counter
#define DEFAULT_SERVO1_ACCEL_SPEED					1.0f			// in pulse/ms^2 with respect to 24000 the PWM max counter
#define DEFAULT_SERVO1_FREQUENCY					1.0f			// in Hz
#define DEFAULT_SERVO1_PHASE						0				// between 0 and 2*pi

#define DEFAULT_SERVO2_TRAJECTORY					DEFAULT_SERVO1_TRAJECTORY
#define DEFAULT_SERVO2_CYCLIC						DEFAULT_SERVO1_CYCLIC
#define DEFAULT_SERVO2_MAX_SPEED					DEFAULT_SERVO1_MAX_SPEED
#define DEFAULT_SERVO2_ACCEL_SPEED					DEFAULT_SERVO1_ACCEL_SPEED
#define DEFAULT_SERVO2_FREQUENCY					DEFAULT_SERVO1_FREQUENCY
#define DEFAULT_SERVO2_PHASE						DEFAULT_SERVO1_PHASE
