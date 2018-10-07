/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file ObstacleAvoidance.cpp
 *
 * @author Mohammed Kabir <mhkabir@mit.edu>
 *
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <drivers/drv_hrt.h>

#include "ObstacleAvoidance.h"

namespace obstacle_avoidance
{

ObstacleAvoidance::ObstacleAvoidance() :
	_avoidanceTrajectoryPub(nullptr),
	_paramHandle(),
	_desiredTrajectory_valid(false),
	_obstacleDistance_valid(false),
	_vehicleAttitude_valid(false),
	_avoidance_initialized(false)
{
	_paramHandle.avoid_distance = param_find("AVOID_DIST");
	_paramHandle.max_deceleration = param_find("MPC_ACC_HOR");

	// Initialize uORB topics.
	_initialize_topics();

	_check_params(true);
}

void ObstacleAvoidance::update()
{
	_check_params(false);

	_update_topics();

	uint64_t now = hrt_absolute_time();

	if (_avoidance_initialized &&
	    (now - _desiredTrajectory.timestamp > AVOIDANCE_TIMEOUT_US ||
	     now - _obstacleDistance.timestamp > AVOIDANCE_TIMEOUT_US)) {
		PX4_WARN("Data timeout");
		_avoidance_initialized = false;
		return;
	}

	if (!_avoidance_initialized &&
	    _desiredTrajectory_valid &&
	    _obstacleDistance_valid &&
	    _vehicleAttitude_valid) {
		_avoidance_initialized = true;
		PX4_INFO("Initialized");
	}

	if (!_avoidance_initialized) {
		// Wait till we receieve all information atleast once
		return;
	}

	// Get vehicle attitude for frame transformations
	matrix::Quaternionf q_att(&_vehicleAttitude.q[0]);
	matrix::Eulerf euler_att(q_att);

	// Get desired velocity in global frame
	matrix::Vector3f desired_vel {};
	desired_vel(0) = _desiredTrajectory.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[0];
	desired_vel(1) = _desiredTrajectory.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[1];

	// Project to body frame
	desired_vel = q_att.conjugate_inversed(desired_vel);

	//PX4_INFO("Desired body: (%f, %f, %f)",
	//	double(desired_vel(0)), double(desired_vel(1)), double(desired_vel(2)));

	// Get obstacle information
	for (int i = 0; i < 72; i++) {

		if (_obstacleDistance.distances[i] == UINT16_MAX) {
			// Invalid reading
			continue;
		}

		if (_obstacleDistance.distances[i] == _obstacleDistance.max_distance + 1) {
			// No obstacle detected in this sector
			continue;
		}

		// Distance to obstacle in meters
		float obs_dist_m = float(_obstacleDistance.distances[i]) / 100.0f;
		// Angle to obstacle in radians (body frame)
		float obs_angle_rad = i * float(_obstacleDistance.increment) * (M_PI_F / 180.0f);

		// Correct measurement for vehicle tilt
		obs_dist_m = obs_dist_m *
			     cosf(obs_angle_rad) *
			     cosf(euler_att.theta()) *
			     cosf(euler_att.phi());

		// Calculate max velocity component in obstacle direction to fully stop before hitting safety margin
		float dist_to_stop = fmaxf(obs_dist_m - _params.avoid_distance, 0.0f);
		// v^2 = u^2 - 2 * a * s - assume constant acceleration braking maneovre.
		float vel_limit = sqrtf(2 * _params.max_deceleration * dist_to_stop);

		PX4_INFO("distance : %fm, vel limit %fm/s", double(obs_dist_m), double(vel_limit));

		// Rotate body frame desired velocity to point to object
		matrix::Quaternionf body_to_obstacle(matrix::Eulerf(0.0f, 0.0f, obs_angle_rad));

		matrix::Vector3f desired_vel_obj = body_to_obstacle.conjugate(desired_vel);

		// Limit velocity in direction of object
		desired_vel_obj(0) = fminf(desired_vel_obj(0), vel_limit);

		// Rotate corrected velocity vector back to body frame
		desired_vel = body_to_obstacle.conjugate_inversed(desired_vel_obj);

	}

	// Project back to to global frame
	desired_vel = q_att.conjugate(desired_vel);

	/*
	PX4_INFO("Desired : (%f, %f, %f), Corrected : (%f, %f, %f)",
		double(_desiredTrajectory.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[0]),
		double(_desiredTrajectory.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[1]),
		double(_desiredTrajectory.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[2]),
		double(desired_vel(0)), double(desired_vel(1)), double(desired_vel(2)));*/

	// Publish modified trajectory
	vehicle_trajectory_waypoint_s avoidanceTrajectory(_desiredTrajectory);

	// Update XY velocity
	avoidanceTrajectory.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[0] = desired_vel(0);
	avoidanceTrajectory.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[1] = desired_vel(1);

	int instance;
	orb_publish_auto(ORB_ID(vehicle_trajectory_waypoint), &_avoidanceTrajectoryPub, &avoidanceTrajectory, &instance,
			 ORB_PRIO_DEFAULT);

}


// Take desired velocity, return corrected velocity.
// Input :
// 1. Unit vector to object
// 2. Maximum acceleration
// 3.
/*
void ObstacleAvoidance::_correct_velocity(const bool force)
{

	float k_p = 1.0f;

	if (dist_m > _params.keep_distance) {
		// Slow down as we get closer to obstacle
	} else if (dist_m <= _params.keep_distance) {
		// Obstacle inside avoidance radius, stop

	}

	bool updated;
	orb_check(_parameterSub, &updated);

	if (updated) {
		parameter_update_s paramUpdate;
		orb_copy(ORB_ID(parameter_update), _parameterSub, &paramUpdate);
	}

	if (updated || force) {
		_update_params();
	}
}*/

void ObstacleAvoidance::_check_params(const bool force)
{
	bool updated;
	orb_check(_parameterSub, &updated);

	if (updated) {
		parameter_update_s paramUpdate;
		orb_copy(ORB_ID(parameter_update), _parameterSub, &paramUpdate);
	}

	if (updated || force) {
		_update_params();
	}
}

void ObstacleAvoidance::_initialize_topics()
{
	_desiredTrajectorySub = orb_subscribe(ORB_ID(vehicle_trajectory_waypoint_desired));
	_obstacleDistanceSub = orb_subscribe(ORB_ID(obstacle_distance));
	_vehicleAttitudeSub = orb_subscribe(ORB_ID(vehicle_attitude));
	_parameterSub = orb_subscribe(ORB_ID(parameter_update));

}

void ObstacleAvoidance::_update_topics()
{
	_desiredTrajectory_valid = _orb_update(ORB_ID(vehicle_trajectory_waypoint_desired), _desiredTrajectorySub,
					       &_desiredTrajectory);
	_obstacleDistance_valid = _orb_update(ORB_ID(obstacle_distance), _obstacleDistanceSub, &_obstacleDistance);
	_vehicleAttitude_valid = _orb_update(ORB_ID(vehicle_attitude), _vehicleAttitudeSub, &_vehicleAttitude);

}


bool ObstacleAvoidance::_orb_update(const struct orb_metadata *meta, int handle, void *buffer)
{
	bool newData = false;

	// check if there is new data to grab
	if (orb_check(handle, &newData) != OK) {
		return false;
	}

	if (!newData) {
		return false;
	}

	if (orb_copy(meta, handle, buffer) != OK) {
		return false;
	}

	return true;
}

void ObstacleAvoidance::_update_params()
{
	param_get(_paramHandle.avoid_distance, &_params.avoid_distance);
	param_get(_paramHandle.max_deceleration, &_params.max_deceleration);
}


} // namespace obstacle_avoidance
