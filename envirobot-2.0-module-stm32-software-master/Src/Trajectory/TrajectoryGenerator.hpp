/*
 * TrajectoryGenerator.hpp
 *
 *  Created on: Nov 23, 2022
 *      Author: bignet
 */

#pragma once

#include <stm32h750xx.h>
#include <stm32h7xx_hal.h>
#include <math.h>

#include "Configurations/ControlConfiguration.h"

// TrajectoryGenerator class used to generate setpoints to controller with predefined shapes
class TrajectoryGenerator {
public:
	TrajectoryGenerator();
	float GenerateSetPoint(void);
	void StartTrajectory(void);
	void StartTrajectory(float start_position_, float end_position_);
	void PlanTrajectory(float start_position_, float end_position_);

	void SetTrajectoryMode(uint8_t mode);
	void SetTrajectoryCyclicValue(bool cyclic);
	void SetTrajectoryMaxSpeed(float speed_max);
	void SetTrajectoryMaxAcceleration(float accel_max);
	void SetTrajectoryFrequency(float frequency);
	void SetTrajectoryPhase(float phase);

	uint8_t GetTrajectoryMode();
	bool GetTrajectoryCyclicValue();
	float GetTrajectoryMaxSpeed();
	float GetTrajectoryMaxAcceleration();
	float GetTrajectoryFrequency();
	float GetTrajectoryPhase();

	// Configuration parameters
	uint8_t cyclic;
	uint8_t mode;
	float speed_max;
	float accel_max;
	float frequency;
	float phase;

private:
	void ComputeTimings(void);
	float Step(float target);
	float Slope(uint32_t start_time, uint32_t time, float start, float target);
	float Trapezoidal(uint32_t start_time, uint32_t time, float start, float target);
	float Sinusoidal(uint32_t start_time, uint32_t time, float start, float target);

	// Variables
	float setpoint;
	int8_t sign;
	uint32_t start_time;
	uint32_t end_time;
	float start_position;
	float end_position;
	uint32_t accel_time;
	uint32_t speed_time;
};
