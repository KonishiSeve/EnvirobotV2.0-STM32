/*
 * Servomotors.hpp
 *
 *  Created on: 13 déc. 2022
 *      Author: bignet
 */

#pragma once

#include "cmsis_os.h"

#include "Trajectory/TrajectoryGenerator.hpp"
#include "Registers/Registers.hpp"
#include "Publishers/Publishers.hpp"

#include "RegisterMaps/RegisterMapServomotors.h"
#include "Configurations/ControlConfiguration.h"

// Class servomotors used to control two servomotors
class Servomotors {
public:
	Servomotors(uint32_t* servomotor1_pulse, uint32_t* servomotor2_pulse);

	void Init(Registers* registers, Publishers* publishers);
	void AddRegisters(void);
	void SetupPublisher(uint8_t interface_ID);

	void ActivateServomotors(void);
	void DeactivateServomotors(void);
	void Reset(void);

	void MoveTo(float target_servo1, float target_servo2);
	void MoveServo1To(float target);
	void MoveServo2To(float target);

	void MoveServo1FromTo(float start, float target);
	void MoveServo2FromTo(float start, float target);
	void SymmetricMoveFromTo(float start, float target);
	void AsymmetricMoveFromTo(float start, float target);

	void SetServomotorsTrajectoryMode(uint8_t mode);
	void SetServomotorsTrajectoryCyclicValue(bool cyclic);
	void SetServomotorsTrajectoryMaxSpeed(float speed_max);
	void SetServomotorsTrajectoryMaxAcceleration(float accel_max);
	void SetServomotorsTrajectoryFrequency(float frequency);
	void SetServomotorsTrajectoryPhase(float phase);

	void Control(void);

	TrajectoryGenerator servomotor1_trajectory_generator;
	TrajectoryGenerator servomotor2_trajectory_generator;
private:
	// OS
	osSemaphoreId_t ServomotorsSemaphore;

	Registers* registers;
	Publishers* publishers;

	bool servomotors_active;

	uint32_t* servomotor1_pulse;
	uint32_t* servomotor2_pulse;

	float servomotor1_target;
	float servomotor2_target;

	uint16_t servomotor1_PWM;
	uint16_t servomotor2_PWM;

	uint16_t servomotor1_offset;
	uint16_t servomotor2_offset;

	// Constants
	float setpoint_conversion = (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 90.0f;
};
