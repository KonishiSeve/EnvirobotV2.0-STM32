/*
 * Servomotors.cpp
 *
 *  Created on: 13 déc. 2022
 *      Author: bignet
 */

#include <Servomotors/Servomotors.hpp>

/**
 * @brief Class constructor
 *
 * @param input servomotor1_pulse_: timer pulse pointer of the servomotor 1
 * @param input servomotor2_pulse_: timer pulse pointer of the servomotor 2
 */
Servomotors::Servomotors(uint32_t* servomotor1_pulse_, uint32_t* servomotor2_pulse_) {
	servomotor1_pulse = servomotor1_pulse_;
	servomotor2_pulse = servomotor2_pulse_;
}

/**
 * @brief Initialize the class with references to other classes.
 *
 * @param input registers_: the Registers instance
 * @param input publishers_: the Publishers instance
 */
void Servomotors::Init(Registers* registers_, Publishers* publishers_) {
	registers = registers_;
	publishers = publishers_;

	ServomotorsSemaphore = osSemaphoreNew(1, 1, NULL);
	osSemaphoreRelease(ServomotorsSemaphore);

	Reset();

	// Setup trajectory generator of servomotor 1
	servomotor1_trajectory_generator.SetTrajectoryMode(DEFAULT_SERVO1_TRAJECTORY);
	servomotor1_trajectory_generator.SetTrajectoryCyclicValue(DEFAULT_SERVO1_CYCLIC);
	servomotor1_trajectory_generator.SetTrajectoryMaxSpeed(DEFAULT_SERVO1_MAX_SPEED);
	servomotor1_trajectory_generator.SetTrajectoryMaxAcceleration(DEFAULT_SERVO1_ACCEL_SPEED);
	servomotor1_trajectory_generator.SetTrajectoryFrequency(DEFAULT_SERVO1_FREQUENCY);
	servomotor1_trajectory_generator.SetTrajectoryPhase(DEFAULT_SERVO1_PHASE);

	// Setup trajectory generator of servomotor 2
	servomotor2_trajectory_generator.SetTrajectoryMode(DEFAULT_SERVO2_TRAJECTORY);
	servomotor2_trajectory_generator.SetTrajectoryCyclicValue(DEFAULT_SERVO2_CYCLIC);
	servomotor2_trajectory_generator.SetTrajectoryMaxSpeed(DEFAULT_SERVO2_MAX_SPEED);
	servomotor2_trajectory_generator.SetTrajectoryMaxAcceleration(DEFAULT_SERVO2_ACCEL_SPEED);
	servomotor2_trajectory_generator.SetTrajectoryFrequency(DEFAULT_SERVO2_FREQUENCY);
	servomotor2_trajectory_generator.SetTrajectoryPhase(DEFAULT_SERVO2_PHASE);

	// Define default offsets
	servomotor1_offset = DEFAULT_SERVO1_OFFSET;
	servomotor2_offset = DEFAULT_SERVO2_OFFSET;
}

/**
 * @brief Add class related registers
 */
void Servomotors::AddRegisters(void) {
	// Register to access the servomotors state, whether active or not
	registers->AddRegister<uint8_t>(REG_SERVO_STATE);
	registers->SetRegisterAsSingle(REG_SERVO_STATE);
	registers->AddReadCallback<uint8_t>(REG_SERVO_STATE, (void*) this,
			[](void* context, uint16_t register_id, uint8_t** output, uint16_t* length) -> bool {
			Servomotors* self = (Servomotors*) context;
			osSemaphoreAcquire(self->ServomotorsSemaphore, osWaitForever);
			**output = (uint8_t) self->servomotors_active;
			osSemaphoreRelease(self->ServomotorsSemaphore);
			return true;
		}
	);
	registers->AddWriteCallback<uint8_t>(REG_SERVO_STATE, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Servomotors* self = (Servomotors*) context;
			if (*input > 0)
				self->ActivateServomotors();
			else
				self->DeactivateServomotors();
			return true;
		}
	);

	// Register to get the servomotor setpoints
	registers->AddRegister<float>(REG_SERVO_GET_SETPOINTS);
	registers->SetRegisterAsArray(REG_SERVO_GET_SETPOINTS, 2);
	registers->AddRegisterSemaphore(REG_SERVO_GET_SETPOINTS, &ServomotorsSemaphore);
	registers->AddReadCallback<float>(REG_SERVO_GET_SETPOINTS, (void*) this,
		[](void* context, uint16_t register_id, float** output, uint16_t* length) -> bool {
			Servomotors* self = (Servomotors*) context;
			*output[0] = self->servomotor1_target;
			*output[1] = self->servomotor2_target;
			return true;
		}
	);

	// Register to access the servomotor offsets
	registers->AddRegister<uint16_t>(REG_SERVO_OFFSET);
	registers->SetRegisterAsArray(REG_SERVO_OFFSET, 2);
	registers->AddRegisterSemaphore(REG_SERVO_OFFSET, &ServomotorsSemaphore);
	registers->AddReadCallback<uint16_t>(REG_SERVO_OFFSET, (void*) this,
		[](void* context, uint16_t register_id, uint16_t** output, uint16_t* length) -> bool {
			Servomotors* self = (Servomotors*) context;
			*output[0] = self->servomotor1_offset;
			*output[1] = self->servomotor2_offset;
			return true;
		}
	);
	registers->AddWriteCallback<uint16_t>(REG_SERVO_OFFSET, (void*) this,
		[](void* context, uint16_t register_id, uint16_t* input, uint16_t length) -> bool {
			Servomotors* self = (Servomotors*) context;
			if (length != 2) return false;
			if (input[0] == ALL_SERVOS || input[0] == SERVO1)
				self->servomotor1_offset = input[1];
			if (input[0] == ALL_SERVOS || input[0] == SERVO2)
				self->servomotor2_offset = input[1];
			return true;
		}
	);

	// Register to move one or  both servomotors from a start position to a target position
	registers->AddRegister<float>(REG_SERVO_MOVEFROMTO);
	registers->SetRegisterAsArray(REG_SERVO_MOVEFROMTO, 3);
	registers->AddWriteCallback<float>(REG_SERVO_MOVEFROMTO, (void*) this,
		[](void* context, uint16_t register_id, float* input, uint16_t length) -> bool {
			Servomotors* self = (Servomotors*) context;
			if (length != 3) return false;
			if (input[0] == ALL_SERVOS || input[0] == SERVO1)
				self->MoveServo1FromTo(input[1], input[2]);
			if (input[0] == ALL_SERVOS || input[0] == SERVO2)
				self->MoveServo2FromTo(input[1], input[2]);
			return true;
		}
	);

	// Register to move one or  both servomotors from the current position to a target position
	registers->AddRegister<float>(REG_SERVO_MOVETO);
	registers->SetRegisterAsArray(REG_SERVO_MOVETO, 2);
	registers->AddWriteCallback<float>(REG_SERVO_MOVETO, (void*) this,
		[](void* context, uint16_t register_id, float* input, uint16_t length) -> bool {
			Servomotors* self = (Servomotors*) context;
			if (length != 2) return false;
			if (input[0] == ALL_SERVOS || input[0] == SERVO1)
				self->MoveServo1To(input[1]);
			if (input[0] == ALL_SERVOS || input[0] == SERVO2)
				self->MoveServo2To(input[1]);
			return true;
		}
	);

	// Register to move servomotors symmetrically from one position to another
	registers->AddRegister<float>(REG_SERVO_SYMMETRIC_MOVE);
	registers->SetRegisterAsArray(REG_SERVO_SYMMETRIC_MOVE, 2);
	registers->AddWriteCallback<float>(REG_SERVO_SYMMETRIC_MOVE, (void*) this,
		[](void* context, uint16_t register_id, float* input, uint16_t length) -> bool {
			Servomotors* self = (Servomotors*) context;
			if (length != 2) return false;
			self->SymmetricMoveFromTo(input[0], input[1]);
			return true;
		}
	);

	// Register to move servomotors asymmetrically from one position to another
	registers->AddRegister<float>(REG_SERVO_ASYMMETRIC_MOVE);
	registers->SetRegisterAsArray(REG_SERVO_ASYMMETRIC_MOVE, 2);
	registers->AddWriteCallback<float>(REG_SERVO_ASYMMETRIC_MOVE, (void*) this,
		[](void* context, uint16_t register_id, float* input, uint16_t length) -> bool {
			Servomotors* self = (Servomotors*) context;
			if (length != 2) return false;
			self->AsymmetricMoveFromTo(input[0], input[1]);
			return true;
		}
	);

	// Register to access the trajectory mode of servomotors
	registers->AddRegister<uint8_t>(REG_SERVO_TRAJECTORY_MODE);
	registers->SetRegisterAsArray(REG_SERVO_TRAJECTORY_MODE, 2);
	registers->AddRegisterSemaphore(REG_SERVO_TRAJECTORY_MODE, &ServomotorsSemaphore);
	registers->AddReadCallback<uint8_t>(REG_SERVO_TRAJECTORY_MODE, (void*) this,
		[](void* context, uint16_t register_id, uint8_t** output, uint16_t* length) -> bool {
			Servomotors* self = (Servomotors*) context;
			*output[0] = self->servomotor1_trajectory_generator.mode;
			*output[1] = self->servomotor2_trajectory_generator.mode;
			return true;
		}
	);
	registers->AddWriteCallback<uint8_t>(REG_SERVO_TRAJECTORY_MODE, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Servomotors* self = (Servomotors*) context;
			if (length != 2) return false;
			if (input[0] == ALL_SERVOS || input[0] == SERVO1)
				self->servomotor1_trajectory_generator.mode = input[1];
			if (input[0] == ALL_SERVOS || input[0] == SERVO2)
				self->servomotor2_trajectory_generator.mode = input[1];
			return true;
		}
	);

	// Register to access the trajectory cyclic parameter of servomotors
	registers->AddRegister<uint8_t>(REG_SERVO_TRAJECTORY_CYCLIC);
	registers->SetRegisterAsArray(REG_SERVO_TRAJECTORY_CYCLIC, 2);
	registers->AddRegisterSemaphore(REG_SERVO_TRAJECTORY_CYCLIC, &ServomotorsSemaphore);
	registers->AddReadCallback<uint8_t>(REG_SERVO_TRAJECTORY_CYCLIC, (void*) this,
		[](void* context, uint16_t register_id, uint8_t** output, uint16_t* length) -> bool {
			Servomotors* self = (Servomotors*) context;
			*output[0] = self->servomotor1_trajectory_generator.cyclic;
			*output[1] = self->servomotor2_trajectory_generator.cyclic;
			return true;
		}
	);
	registers->AddWriteCallback<uint8_t>(REG_SERVO_TRAJECTORY_CYCLIC, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Servomotors* self = (Servomotors*) context;
			if (length != 2) return false;
			if (input[0] == ALL_SERVOS || input[0] == SERVO1)
				self->servomotor1_trajectory_generator.cyclic = input[1];
			if (input[0] == ALL_SERVOS || input[0] == SERVO2)
				self->servomotor2_trajectory_generator.cyclic = input[1];
			return true;
		}
	);

	// Register to plan a trajectory for servomotors based on selection index
	registers->AddRegister<float>(REG_SERVO_PLAN_TRAJECTORY);
	registers->SetRegisterAsArray(REG_SERVO_PLAN_TRAJECTORY, 3);
	registers->AddRegisterSemaphore(REG_SERVO_PLAN_TRAJECTORY, &ServomotorsSemaphore);
	registers->AddWriteCallback<float>(REG_SERVO_PLAN_TRAJECTORY, (void*) this,
		[](void* context, uint16_t register_id, float* input, uint16_t length) -> bool {
			Servomotors* self = (Servomotors*) context;
			if (length != 3) return false;
			if (input[0] == ALL_SERVOS || input[0] == SERVO1)
				self->servomotor1_trajectory_generator.PlanTrajectory(input[1], input[2]);
			if (input[0] == ALL_SERVOS || input[0] == SERVO2)
				self->servomotor2_trajectory_generator.PlanTrajectory(input[1], input[2]);
			return true;
		}
	);

	// Register to start a planned trajectory for servomotors based on selection index
	registers->AddRegister<uint8_t>(REG_SERVO_START_TRAJECTORY);
	registers->SetRegisterAsSingle(REG_SERVO_START_TRAJECTORY);
	registers->AddRegisterSemaphore(REG_SERVO_START_TRAJECTORY, &ServomotorsSemaphore);
	registers->AddWriteCallback<uint8_t>(REG_SERVO_START_TRAJECTORY, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			Servomotors* self = (Servomotors*) context;
			if (length != 1) return false;
			if (input[0] == ALL_SERVOS || input[0] == SERVO1)
				self->servomotor1_trajectory_generator.StartTrajectory();
			if (input[0] == ALL_SERVOS || input[0] == SERVO2)
				self->servomotor2_trajectory_generator.StartTrajectory();
			return true;
		}
	);

	// Register to access the trajectory maximum speed
	registers->AddRegister<float>(REG_SERVO_MAX_SPEED);
	registers->SetRegisterAsArray(REG_SERVO_MAX_SPEED, 2);
	registers->AddRegisterSemaphore(REG_SERVO_MAX_SPEED, &ServomotorsSemaphore);
	registers->AddReadCallback<float>(REG_SERVO_MAX_SPEED, (void*) this,
		[](void* context, uint16_t register_id, float** output, uint16_t* length) -> bool {
			Servomotors* self = (Servomotors*) context;
			*output[0] = self->servomotor1_trajectory_generator.speed_max;
			*output[1] = self->servomotor2_trajectory_generator.speed_max;
			return true;
		}
	);
	registers->AddWriteCallback<float>(REG_SERVO_MAX_SPEED, (void*) this,
		[](void* context, uint16_t register_id, float* input, uint16_t length) -> bool {
			Servomotors* self = (Servomotors*) context;
			if (length != 2) return false;
			if (input[0] == ALL_SERVOS || input[0] == SERVO1)
				self->servomotor1_trajectory_generator.speed_max = input[1];
			if (input[0] == ALL_SERVOS || input[0] == SERVO2)
				self->servomotor2_trajectory_generator.speed_max = input[1];
			return true;
		}
	);

	// Register to access the trajectory maximum acceleration
	registers->AddRegister<float>(REG_SERVO_MAX_ACCELERATION);
	registers->SetRegisterAsArray(REG_SERVO_MAX_ACCELERATION, 2);
	registers->AddRegisterSemaphore(REG_SERVO_MAX_ACCELERATION, &ServomotorsSemaphore);
	registers->AddReadCallback<float>(REG_SERVO_MAX_ACCELERATION, (void*) this,
		[](void* context, uint16_t register_id, float** output, uint16_t* length) -> bool {
			Servomotors* self = (Servomotors*) context;
			*output[0] = self->servomotor1_trajectory_generator.accel_max;
			*output[1] = self->servomotor2_trajectory_generator.accel_max;
			return true;
		}
	);
	registers->AddWriteCallback<float>(REG_SERVO_MAX_ACCELERATION, (void*) this,
		[](void* context, uint16_t register_id, float* input, uint16_t length) -> bool {
			Servomotors* self = (Servomotors*) context;
			if (length != 2) return false;
			if (input[0] == ALL_SERVOS || input[0] == SERVO1)
				self->servomotor1_trajectory_generator.accel_max = input[1];
			if (input[0] == ALL_SERVOS || input[0] == SERVO2)
				self->servomotor2_trajectory_generator.accel_max = input[1];
			return true;
		}
	);

	// Register to access the trajectory sinus frequency
	registers->AddRegister<float>(REG_SERVO_SIN_FREQUENCY);
	registers->SetRegisterAsArray(REG_SERVO_SIN_FREQUENCY, 2);
	registers->AddRegisterSemaphore(REG_SERVO_SIN_FREQUENCY, &ServomotorsSemaphore);
	registers->AddReadCallback<float>(REG_SERVO_SIN_FREQUENCY, (void*) this,
		[](void* context, uint16_t register_id, float** output, uint16_t* length) -> bool {
			Servomotors* self = (Servomotors*) context;
			*output[0] = self->servomotor1_trajectory_generator.frequency;
			*output[1] = self->servomotor2_trajectory_generator.frequency;
			return true;
		}
	);
	registers->AddWriteCallback<float>(REG_SERVO_SIN_FREQUENCY, (void*) this,
		[](void* context, uint16_t register_id, float* input, uint16_t length) -> bool {
			Servomotors* self = (Servomotors*) context;
			if (length != 2) return false;
			if (input[0] == ALL_SERVOS || input[0] == SERVO1)
				self->servomotor1_trajectory_generator.frequency = input[1];
			if (input[0] == ALL_SERVOS || input[0] == SERVO2)
				self->servomotor2_trajectory_generator.frequency = input[1];
			return true;
		}
	);

	// Register to access the trajectory sinus phase
	registers->AddRegister<float>(REG_SERVO_SIN_PHASE);
	registers->SetRegisterAsArray(REG_SERVO_SIN_PHASE, 2);
	registers->AddRegisterSemaphore(REG_SERVO_SIN_PHASE, &ServomotorsSemaphore);
	registers->AddReadCallback<float>(REG_SERVO_SIN_PHASE, (void*) this,
		[](void* context, uint16_t register_id, float** output, uint16_t* length) -> bool {
			Servomotors* self = (Servomotors*) context;
			*output[0] = self->servomotor1_trajectory_generator.phase;
			*output[1] = self->servomotor2_trajectory_generator.phase;
			return true;
		}
	);
	registers->AddWriteCallback<float>(REG_SERVO_SIN_PHASE, (void*) this,
		[](void* context, uint16_t register_id, float* input, uint16_t length) -> bool {
			Servomotors* self = (Servomotors*) context;
			if (length != 2) return false;
			if (input[0] == ALL_SERVOS || input[0] == SERVO1)
				self->servomotor1_trajectory_generator.phase = input[1];
			if (input[0] == ALL_SERVOS || input[0] == SERVO2)
				self->servomotor2_trajectory_generator.phase = input[1];
			return true;
		}
	);
}

/**
 * @brief Setup the servomotors publisher
 *
 * @param input interface_ID: the interface ID to publish on
 */
void Servomotors::SetupPublisher(uint8_t interface_ID) {
	// Add publisher
	publishers->AddPublisher(PUBLISHER_SERVOMOTORS);

	// Add an interface to the publisher
	publishers->LinkToInterface(PUBLISHER_SERVOMOTORS, interface_ID);

	// Setup publisher params
	publishers->SetPublisherPrescaler(PUBLISHER_CONTROLLER, PUBLISHER_SERVOMOTORS_PRESCALER);
	publishers->SetPublishAddress(PUBLISHER_CONTROLLER, interface_ID, PUBLISHER_SERVOMOTORS_ADDRESS);

	// Add topics
	publishers->AddTopic(PUBLISHER_SERVOMOTORS, REG_TIMEBASE);
	publishers->AddTopic(PUBLISHER_SERVOMOTORS, REG_SERVO_GET_SETPOINTS);

	// Activate topics
#ifdef PUBLISH_SERVOMOTORS_TIMEBASE
	publishers->ActivateTopic(PUBLISHER_SERVOMOTORS, REG_TIMEBASE);
#endif
#ifdef PUBLISH_SERVOMOTORS_SETPOINTS
	publishers->ActivateTopic(PUBLISHER_SERVOMOTORS, REG_SERVO_GET_SETPOINTS);
#endif

#ifdef PUBLISHER_SERVOMOTORS_ACTIVE
	// Activate publisher
	publishers->ActivatePublisher(PUBLISHER_SERVOMOTORS);
#endif
}

/**
 * @brief Activate servomotors
 */
void Servomotors::ActivateServomotors(void) {
	osSemaphoreAcquire(ServomotorsSemaphore, osWaitForever);
	servomotors_active = true;
	osSemaphoreRelease(ServomotorsSemaphore);
}

/**
 * @brief Deactivate servomotors
 */
void Servomotors::DeactivateServomotors(void) {
	osSemaphoreAcquire(ServomotorsSemaphore, osWaitForever);
	servomotors_active = false;
	*servomotor1_pulse = 0;
	*servomotor2_pulse = 0;
	osSemaphoreRelease(ServomotorsSemaphore);
}

/**
 * @brief Reset servomotors
 */
void Servomotors::Reset(void) {
	DeactivateServomotors();
}

/**
 * @brief Move servomotors from the current position to respective targets
 *
 * @param input target_servo1: target of servo1 in °
 * @param input target_servo2: target of servo2 in °
 */
void Servomotors::MoveTo(float target_servo1, float target_servo2) {
	MoveServo1To(target_servo1);
	MoveServo2To(target_servo2);
}

/**
 * @brief Move servomotor 1 from the current position to the target
 *
 * @param input target: target position in °
 */
void Servomotors::MoveServo1To(float target) {
	MoveServo1FromTo(*servomotor1_pulse, target);
}

/**
 * @brief Move servomotor 2 from the current position to the target
 *
 * @param input target: target position in °
 */
void Servomotors::MoveServo2To(float target) {
	MoveServo2FromTo(*servomotor2_pulse, target);
}

/**
 * @brief Move servomotor 1 from the start position to the target
 *
 * @param input start: start position in °
 * @param input target: target position in °
 */
void Servomotors::MoveServo1FromTo(float start, float target) {
	osSemaphoreAcquire(ServomotorsSemaphore, osWaitForever);
	servomotor1_trajectory_generator.StartTrajectory(start, target);
	osSemaphoreRelease(ServomotorsSemaphore);
}

/**
 * @brief Move servomotor 2 from the start position to the target
 *
 * @param input start: start position in °
 * @param input target: target position in °
 */
void Servomotors::MoveServo2FromTo(float start, float target) {
	osSemaphoreAcquire(ServomotorsSemaphore, osWaitForever);
	servomotor2_trajectory_generator.StartTrajectory(-start, -target);
	osSemaphoreRelease(ServomotorsSemaphore);
}

/**
 * @brief Symmetrically move servomotors from the start position to the target. Symmetric means that servomotors go up and down at the same time in phase
 *
 * @param input start: start position in °
 * @param input target: target position in °
 */
void Servomotors::SymmetricMoveFromTo(float start, float target) {
	MoveServo1FromTo(start, target);
	MoveServo2FromTo(start, target);
}

/**
 * @brief Asymmetrically move servomotors from the start position to the target. Asymmetric means that servomotors go up and down out of phase
 *
 * @param input start: start position in °
 * @param input target: target position in °
 */
void Servomotors::AsymmetricMoveFromTo(float start, float target) {
	MoveServo1FromTo(start, target);
	MoveServo2FromTo(target, start);
}

/**
 * @brief Function to compute setpoints and control servomotors. Called periodically by a freeRTOS task. The delay is defined by the controller
 */
void Servomotors::Control(void) {
	osSemaphoreAcquire(ServomotorsSemaphore, osWaitForever);
	if (servomotors_active) {
		servomotor1_target = servomotor1_trajectory_generator.GenerateSetPoint();
		servomotor2_target = servomotor2_trajectory_generator.GenerateSetPoint();

		servomotor1_PWM = (uint16_t) ((int16_t) (setpoint_conversion * servomotor1_target) + servomotor1_offset);
		if (servomotor1_PWM > SERVO_MAX_PULSE) servomotor1_PWM = SERVO_MAX_PULSE;
		if (servomotor1_PWM < SERVO_MIN_PULSE) servomotor1_PWM = SERVO_MIN_PULSE;

		servomotor2_PWM = (uint16_t) ((int16_t) (setpoint_conversion * servomotor2_target) + servomotor2_offset);
		if (servomotor2_PWM > SERVO_MAX_PULSE) servomotor2_PWM = SERVO_MAX_PULSE;
		if (servomotor2_PWM < SERVO_MIN_PULSE) servomotor2_PWM = SERVO_MIN_PULSE;

		*servomotor1_pulse = servomotor1_PWM;
		*servomotor2_pulse = servomotor2_PWM;
	}
	osSemaphoreRelease(ServomotorsSemaphore);

	publishers->SpinPublisher(PUBLISHER_SERVOMOTORS);
}

/**
 * @brief Set both servomotors trajectory into the same mode
 *
 * @param input mode: trajectory mode
 */
void Servomotors::SetServomotorsTrajectoryMode(uint8_t mode) {
	osSemaphoreAcquire(ServomotorsSemaphore, osWaitForever);
	servomotor1_trajectory_generator.SetTrajectoryMode(mode);
	servomotor2_trajectory_generator.SetTrajectoryMode(mode);
	osSemaphoreRelease(ServomotorsSemaphore);
}

/**
 * @brief Set both servomotors trajectory cyclic value
 *
 * @param input cyclic: whether cyclic or not
 */
void Servomotors::SetServomotorsTrajectoryCyclicValue(bool cyclic) {
	osSemaphoreAcquire(ServomotorsSemaphore, osWaitForever);
	servomotor1_trajectory_generator.SetTrajectoryCyclicValue(cyclic);
	servomotor2_trajectory_generator.SetTrajectoryCyclicValue(cyclic);
	osSemaphoreRelease(ServomotorsSemaphore);
}

/**
 * @brief Set both servomotors trajectory maximum speed
 *
 * @param input max_speed: maximum speed allowed by the trajectory generator
 */
void Servomotors::SetServomotorsTrajectoryMaxSpeed(float speed_max) {
	osSemaphoreAcquire(ServomotorsSemaphore, osWaitForever);
	servomotor1_trajectory_generator.SetTrajectoryMaxSpeed(speed_max);
	servomotor2_trajectory_generator.SetTrajectoryMaxSpeed(speed_max);
	osSemaphoreRelease(ServomotorsSemaphore);
}

/**
 * @brief Set both servomotors trajectory maximum acceleration
 *
 * @param input accel_max: maximum acceleration allowed by the trajectory generator
 */
void Servomotors::SetServomotorsTrajectoryMaxAcceleration(float accel_max) {
	osSemaphoreAcquire(ServomotorsSemaphore, osWaitForever);
	servomotor1_trajectory_generator.SetTrajectoryMaxAcceleration(accel_max);
	servomotor2_trajectory_generator.SetTrajectoryMaxAcceleration(accel_max);
	osSemaphoreRelease(ServomotorsSemaphore);
}

/**
 * @brief Set both servomotors trajectory sinus frequency
 *
 * @param input frequency: sinus frequency
 */
void Servomotors::SetServomotorsTrajectoryFrequency(float frequency) {
	osSemaphoreAcquire(ServomotorsSemaphore, osWaitForever);
	servomotor1_trajectory_generator.SetTrajectoryFrequency(frequency);
	servomotor2_trajectory_generator.SetTrajectoryFrequency(frequency);
	osSemaphoreRelease(ServomotorsSemaphore);
}

/**
 * @brief Set both servomotors trajectory sinus phase
 *
 * @param input phase: sinus phase
 */
void Servomotors::SetServomotorsTrajectoryPhase(float phase) {
	osSemaphoreAcquire(ServomotorsSemaphore, osWaitForever);
	servomotor1_trajectory_generator.SetTrajectoryPhase(phase);
	servomotor2_trajectory_generator.SetTrajectoryPhase(phase);
	osSemaphoreRelease(ServomotorsSemaphore);
}
