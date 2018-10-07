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

#pragma once

#include <px4_workqueue.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/parameter_update.h>
#include <matrix/math.hpp>
#include <matrix/Matrix.hpp>

namespace obstacle_avoidance
{

class ObstacleAvoidance
{
public:

	ObstacleAvoidance();
	virtual ~ObstacleAvoidance() = default;

	/*
	 * Get new obstacle data and desired trajectory, and update the avoidance trajectory
	 */
	void update();

protected:
	/*
	 * Called once to initialize uORB topics.
	 */
	void _initialize_topics();

	/*
	 * Update uORB topics.
	 */
	void _update_topics();

	/*
	 * Update parameters.
	 */
	void _update_params();

	/*
	 * Convenience function for polling uORB subscriptions.
	 *
	 * @return true if there was new data and it was successfully copied
	 */
	static bool _orb_update(const struct orb_metadata *meta, int handle, void *buffer);

	/* timeout after which avoidance is reset if no messages have been received */
	static constexpr uint32_t AVOIDANCE_TIMEOUT_US = 2000000;

	orb_advert_t _avoidanceTrajectoryPub;

	int _parameterSub;

private:

	/**
	* Handles for parameters
	**/
	struct {
		param_t avoid_distance;
		param_t max_deceleration;
	} _paramHandle;

	struct {
		float avoid_distance;
		float max_deceleration;
	} _params;

	int _desiredTrajectorySub;
	int _obstacleDistanceSub;
	int _vehicleAttitudeSub;

	struct vehicle_trajectory_waypoint_s	_desiredTrajectory;
	struct obstacle_distance_s				_obstacleDistance;
	struct vehicle_attitude_s				_vehicleAttitude;

	// keep track of updated topics
	bool _desiredTrajectory_valid;
	bool _obstacleDistance_valid;
	bool _vehicleAttitude_valid;

	bool _avoidance_initialized;

	matrix::Dcm<float> _R_att;

	void _check_params(const bool force);

	void _update_state();
};


} // namespace obstacle_avoidance
