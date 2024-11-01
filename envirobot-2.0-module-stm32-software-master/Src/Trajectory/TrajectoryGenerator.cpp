/*
 * TrajectoryGenerator.cpp
 *
 *  Created on: Nov 23, 2022
 *      Author: bignet
 */

#include <Trajectory/TrajectoryGenerator.hpp>

/**
 * @brief Class constructor
 */
TrajectoryGenerator::TrajectoryGenerator() {
	cyclic = false;
	mode = TRAJECTORY_STEP;
	speed_max = 1000.0f; 		// /ms
	accel_max = 1.0f; 			// /ms^2
	frequency = 1.0f;
}

/**
 * @brief Generate the setpoint based on the selected trajectory mode and timestamp
 *
 * @return the setpoint
 */
float TrajectoryGenerator::GenerateSetPoint(void) {
	if (mode == TRAJECTORY_STEP) {
		setpoint = Step(end_position);
		return setpoint;
	}

	uint32_t time = HAL_GetTick();
	if (time > end_time) {
		if (cyclic) {
			StartTrajectory(end_position, start_position);
			return start_position;
		} else
			return end_position;//return setpoint;
	}

	switch (mode) {
		case TRAJECTORY_SLOPE:
			setpoint = Slope(start_time, time, start_position, end_position);
			break;
		case TRAJECTORY_TRAPEZOIDAL:
			setpoint = Trapezoidal(start_time, time, start_position, end_position);
			break;
		case TRAJECTORY_SINUSOIDAL:
			setpoint = Sinusoidal(start_time, time, start_position, end_position);
			break;
		default:
			break;
	}
	return setpoint;
}

/**
 * @brief Start a planned trajectory
 */
void TrajectoryGenerator::StartTrajectory(void) {
	start_time = HAL_GetTick();
	ComputeTimings();
}

/**
 * @brief Start a trajectory from the start_position_ to the end_position_
 *
 * @param input start_position_
 * @param input end_position_
 */
void TrajectoryGenerator::StartTrajectory(float start_position_, float end_position_) {
	PlanTrajectory(start_position_, end_position_);
	StartTrajectory();
}

/**
 * @brief Plan a trajectory from the start_position_ to the end_position_. The motion is not started here
 *
 * @param input start_position_
 * @param input end_position_
 */
void TrajectoryGenerator::PlanTrajectory(float start_position_, float end_position_) {
	start_position = start_position_;
	end_position = end_position_;

	(end_position_ > start_position) ? sign = 1 : sign = -1;
}

/**
 * @brief Compute the trajectory timings based on the selected trajectory mode
 */
void TrajectoryGenerator::ComputeTimings(void) {
	uint32_t accel_time_no_speed_sat;
	switch (mode) {
		// STEP
		case 0:
			end_time = start_time;
			break;
		// SLOPE
		case 1:
			end_time = start_time + abs(end_position - start_position) / speed_max;
			break;
		// TRAPEZOIDAL
		case 2:
			accel_time_no_speed_sat = sqrt(abs(end_position - start_position) / accel_max);
			accel_time = speed_max / accel_max;
			if (accel_time_no_speed_sat > accel_time)  {
				speed_time = abs(end_position - start_position) / speed_max;
				end_time = start_time + accel_time + speed_time;
			} else {
				accel_time = accel_time_no_speed_sat;
				speed_time = accel_time;
				end_time = start_time + 2.0f * accel_time_no_speed_sat;
			}
			break;
		// SINUSOIDAL
		case 3:
			end_time = start_time + 1000.0f / (2.0f * frequency);
			break;
		default:
			break;
	}
}

/**
 * @brief Generate the step setpoint (step in position)
 *
 * @param input target: target value
 * @return setpoint
 */
float TrajectoryGenerator::Step(float target) {
	return target;
}

/**
 * @brief Generate the slope setpoint (slope in position motion)
 *
 * @param input start_time: the timestamp corresponding to the beginning of the motion
 * @param input time: the current timestamp
 * @param input start: start value
 * @param input target: target value
 * @return setpoint
 */
float TrajectoryGenerator::Slope(uint32_t start_time, uint32_t time, float start, float target) {
	if (time < start_time) return start;
	return start + sign * speed_max * (time - start_time);
}

/**
 * @brief Generate the trapezoidal setpoint (trapezoidal in speed motion)
 *
 * @param input start_time: the timestamp corresponding to the beginning of the motion
 * @param input time: the current timestamp
 * @param input start: start value
 * @param input target: target value
 * @return setpoint
 */
float TrajectoryGenerator::Trapezoidal(uint32_t start_time, uint32_t time, float start, float target) {
	if (time < start_time) return start;
	if (time < start_time + accel_time) {
		return sign * 0.5f * accel_max * powf((time  - start_time), 2) + start;
	} else if (time < start_time + speed_time) {
		return sign * speed_max * (time - start_time - 0.5f * accel_time) + start;
	} else {
		if (speed_time > accel_time) {
			return - sign * 0.5f * accel_max * powf((time - start_time - speed_time), 2) + sign * speed_max * (time - start_time - 0.5f * accel_time) + start;
		} else {
			return - sign * 0.5f * accel_max * powf((time - start_time - speed_time), 2) + sign * accel_max * speed_time * (time - start_time - speed_time) + 0.5f * (start + target);
		}
	}
}

/**
 * @brief Generate the sinusoidal setpoint (sinusoidal in position)
 *
 * @param input start_time: the timestamp corresponding to the beginning of the motion
 * @param input time: the current timestamp
 * @param input start: start value
 * @param input target: target value
 * @return setpoint
 */
float TrajectoryGenerator::Sinusoidal(uint32_t start_time, uint32_t time, float start, float target) {
	if (time < start_time) return start;
	return (int32_t) (start - (target - start) * (cos(2.0f * M_PI * frequency * (time - start_time) * 0.001f + phase) - 1) / 2.0f); // or / 1000.0f instead of * 0.001f but more efficient
}

/**
 * @brief Set the trajectory mode
 *
 * @param input mode_: the trajectory mode
*/
void TrajectoryGenerator::SetTrajectoryMode(uint8_t mode_) {
	mode = mode_;
}

/**
 * @brief Set the cyclic falg
 *
 * @param input cyclic_: whether the motion is cyclic or not
*/
void TrajectoryGenerator::SetTrajectoryCyclicValue(bool cyclic_) {
	cyclic = cyclic_;
}

/**
 * @brief Set the maximum speed
 *
 * @param input speed_max_: the maximum speed
*/
void TrajectoryGenerator::SetTrajectoryMaxSpeed(float speed_max_) {
	speed_max = speed_max_;
}

/**
 * @brief Set the maximum acceleration
 *
 * @param input accel_max_: the maximum acceleration
*/
void TrajectoryGenerator::SetTrajectoryMaxAcceleration(float accel_max_) {
	accel_max = accel_max_;
}

/**
 * @brief Set the sinusoidal frequency
 *
 * @param input frequency_: the sinusoidal frequency
*/
void TrajectoryGenerator::SetTrajectoryFrequency(float frequency_) {
	frequency = frequency_;
}

/**
 * @brief Set the sinusoidal phase
 *
 * @param input frequency_: the sinusoidal phase
*/
void TrajectoryGenerator::SetTrajectoryPhase(float phase_) {
	phase = phase_;
}

/**
 * @brief Get the trajectory mode
 *
 * @return the trajectory mode
*/
uint8_t TrajectoryGenerator::GetTrajectoryMode() {
	return mode;
}

/**
 * @brief Get the cyclic flag
 *
 * @return whether the trajectory is cyclic or not
*/
bool TrajectoryGenerator::GetTrajectoryCyclicValue() {
	return cyclic;
}

/**
 * @brief Get the maximum speed
 *
 * @return the maximum speed
*/
float TrajectoryGenerator::GetTrajectoryMaxSpeed() {
	return speed_max;
}

/**
 * @brief Get the maximum acceleration
 *
 * @return the maximum acceleration
*/
float TrajectoryGenerator::GetTrajectoryMaxAcceleration() {
	return accel_max;
}

/**
 * @brief Get the sinusoidal frequency
 *
 * @return the sinusoidal frequency
*/
float TrajectoryGenerator::GetTrajectoryFrequency() {
	return frequency;
}

/**
 * @brief Get the sinusoidal phase
 *
 * @return the sinusoidal phase
*/
float TrajectoryGenerator::GetTrajectoryPhase() {
	return phase;
}
