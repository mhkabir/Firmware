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

/**
 * @file FlightTaskPrecland.cpp
 */

#include "FlightTaskPrecland.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;

bool FlightTaskPrecland::initializeSubscriptions(SubscriptionArray &subscription_array)
{
	if (!FlightTask::initializeSubscriptions(subscription_array)) {
		return false;
	}

	if (!subscription_array.get(ORB_ID(position_setpoint_triplet), _sub_triplet_setpoint)) {
		return false;
	}

	return true;
}

bool FlightTaskPrecland::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();
	// require a valid triplet
	ret = ret && _sub_triplet_setpoint->get().current.valid;
	// require valid position / velocity in xy
	return ret && PX4_ISFINITE(_position(0))
	       && PX4_ISFINITE(_position(1))
	       && PX4_ISFINITE(_velocity(0))
	       && PX4_ISFINITE(_velocity(1));
}
	
bool FlightTaskPrecland::activate()
{
	bool ret = FlightTask::activate();
	_position_setpoint = _position;
	_velocity_setpoint *= 0.0f;
	_position_lock *= NAN;
	return ret;
}

bool FlightTaskPrecland::update()
{
	if (!_sub_triplet_setpoint->get().current.valid) {
		_resetSetpoints();
		_position_setpoint = _position;
		return false;
	}

	// reset setpoint for every loop
	_resetSetpoints();

	// Yaw / Yaw-speed
	if (_sub_triplet_setpoint->get().current.yaw_valid) {
		// yaw control required
		_yaw_setpoint = _sub_triplet_setpoint->get().current.yaw;

		if (_sub_triplet_setpoint->get().current.yawspeed_valid) {
			// yawspeed is used as feedforward
			_yawspeed_setpoint = _sub_triplet_setpoint->get().current.yawspeed;
		}

	} else if (_sub_triplet_setpoint->get().current.yawspeed_valid) {
		// only yawspeed required
		_yawspeed_setpoint = _sub_triplet_setpoint->get().current.yawspeed;
		// set yaw setpoint to NAN since not used
		_yaw_setpoint = NAN;

	}

	// Position
	if (_sub_triplet_setpoint->get().current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
		// Horizontal control
		if (_sub_triplet_setpoint->get().current.position_valid) {
			_position_setpoint(0) = _sub_triplet_setpoint->get().current.x;
			_position_setpoint(1) = _sub_triplet_setpoint->get().current.y;

		} else {
			return false;
		}

		// Vertical control
		if (_sub_triplet_setpoint->get().current.alt_valid) {
			_position_setpoint(2) = _sub_triplet_setpoint->get().current.z;

		} else {
			return false;
		}

		return true;

	} else {
		_position_lock *= NAN;
	}

	// Land
	if (_sub_triplet_setpoint->get().current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
		if (_sub_triplet_setpoint->get().current.position_valid) {
			// Land with landing speed, keep tracking target
			_position_setpoint(0) = _position_lock(0) = _sub_triplet_setpoint->get().current.x;
			_position_setpoint(1) = _position_lock(1) = _sub_triplet_setpoint->get().current.y;
			_position_setpoint(2) = _position_lock(2) = NAN;
			_velocity_setpoint(2) = MPC_LAND_SPEED.get();

		} else if (!PX4_ISFINITE(_position_lock(0))) {
			_position_setpoint = _position_lock = _position;
			_position_setpoint(2) = _position_lock(2) = NAN;
			_velocity_setpoint(2) = MPC_LAND_SPEED.get();

		} else {
			// If position is not valid anymore, continue descending with last known position
			_position_setpoint = _position_lock;
			_velocity_setpoint(2) = MPC_LAND_SPEED.get();
		}

		return true;

	} else {
		_position_lock *= NAN;
	}

	// Idle
	if (_sub_triplet_setpoint->get().current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
		_thrust_setpoint.zero();
		return true;
	} else {
		_position_lock *= NAN;
	}

	return true;
}
