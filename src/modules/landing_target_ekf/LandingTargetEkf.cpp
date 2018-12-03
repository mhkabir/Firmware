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
 * @file LandingTargetEkf.cpp
 *
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 *
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <drivers/drv_hrt.h>

#include "LandingTargetEkf.h"

#define SEC2USEC 1000000.0f

namespace landing_target_ekf
{

LandingTargetEkf::LandingTargetEkf() :
	_targetPosePub(nullptr),
	_targetInnovationsPub(nullptr),
	_paramHandle(),
	_vehicleLocalPositionValid(false),
	_vehicleAttitudeValid(false),
	_sensorBiasValid(false),
	_targetDetectionValid(false),
	_estimatorInitialized(false),
	_faultDetected(false),
	_lastPredict(0),
	_lastUpdate(0)
{
	_paramHandle.acc_unc = param_find("LTEKF_ACC_UNC");
	_paramHandle.meas_unc = param_find("LTEKF_MEAS_UNC");
	_paramHandle.pos_unc_init = param_find("LTEKF_POS_UNC_IN");
	_paramHandle.vel_unc_init = param_find("LTEKF_VEL_UNC_IN");

	// Initialize uORB topics.
	_initialize_topics();

	_check_params(true);
}

void LandingTargetEkf::update()
{
	_check_params(false);

	_update_topics();

	/* predict */
	if (_estimatorInitialized) {
		if (hrt_absolute_time() - _lastUpdate > landing_target_ekf_TIMEOUT_US) {
			PX4_WARN("Timeout");
			_estimatorInitialized = false;

		} else {
			float dt = (hrt_absolute_time() - _lastPredict) / SEC2USEC;

			// predict target position with the help of accel data
			matrix::Vector3f a;

			if (_vehicleAttitudeValid && _sensorBiasValid) {
				matrix::Quaternion<float> q_att(&_vehicleAttitude.q[0]);
				_R_att = matrix::Dcm<float>(q_att);
				a(0) = _sensorBias.accel_x;
				a(1) = _sensorBias.accel_y;
				a(2) = _sensorBias.accel_z;
				a = _R_att * a;

			} else {
				a.zero();
			}

			_kalman_filter_x.predict(dt, -a(0), _params.acc_unc);
			_kalman_filter_y.predict(dt, -a(1), _params.acc_unc);
			_kalman_filter_z.predict(dt, -a(2) - CONSTANTS_ONE_G, _params.acc_unc);

			_lastPredict = hrt_absolute_time();
		}
	}

	if (!_targetDetectionValid) {
		// nothing to do
		return;
	}

	// mark this sensor measurement as consumed
	_targetDetectionValid = false;

	if (!_vehicleAttitudeValid || !_vehicleLocalPositionValid) {
		// don't have the data needed for an update
		return;
	}

	if (!PX4_ISFINITE(_targetDetection.x) ||
	    !PX4_ISFINITE(_targetDetection.y) ||
	    !PX4_ISFINITE(_targetDetection.z)) {
		return;
	}

	// Position vector of target in body frame
	matrix::Vector3f targetPos(_targetDetection.x,
				   _targetDetection.y,
				   _targetDetection.z);

	// Rotate the position into the navigation frame
	matrix::Quaternionf q_att(&_vehicleAttitude.q[0]); // TODO use buffering and time delay compensation
	_R_att = matrix::Dcm<float>(q_att);
	targetPos = _R_att * targetPos;

	if (!_estimatorInitialized) {
		PX4_INFO("Init");
		float vx_init = _vehicleLocalPosition.v_xy_valid ? -_vehicleLocalPosition.vx : 0.0f;
		float vy_init = _vehicleLocalPosition.v_xy_valid ? -_vehicleLocalPosition.vy : 0.0f;
		float vz_init = _vehicleLocalPosition.v_z_valid ? -_vehicleLocalPosition.vz : 0.0f;
		// TODO : Use dense KF instead of separate 1D Kalmans to handle cross terms
		_kalman_filter_x.init(targetPos(0), vx_init, _params.pos_unc_init, _params.vel_unc_init);
		_kalman_filter_y.init(targetPos(1), vy_init, _params.pos_unc_init, _params.vel_unc_init);
		_kalman_filter_z.init(targetPos(2), vz_init, _params.pos_unc_init, _params.vel_unc_init);

		_estimatorInitialized = true;
		_lastUpdate = hrt_absolute_time();
		_lastPredict = _lastUpdate;

	} else {
		// Perform KF update
		float dist = _vehicleLocalPosition.dist_bottom_valid ? _vehicleLocalPosition.dist_bottom : 1.0f;
		bool update_x = _kalman_filter_x.update(targetPos(0), _params.meas_unc * dist * dist);
		bool update_y = _kalman_filter_y.update(targetPos(1), _params.meas_unc * dist * dist);
		bool update_z = _kalman_filter_z.update(targetPos(2), _params.meas_unc * dist * dist);

		if (!update_x || !update_y || !update_z) {
			if (!_faultDetected) {
				_faultDetected = true;
				PX4_WARN("Measurement rejected:%s%s%s", update_x ? "" : " x",
					 update_y ? "" : " y",
					 update_z ? "" : " z");
			}

		} else {
			_faultDetected = false;
		}

		if (!_faultDetected) {

			landing_target_pose_s target_pose {};

			// only publish if measurements were good
			target_pose.timestamp = _targetDetection.timestamp;

			float x, xvel, y, yvel, z, zvel, covx, covx_v, covy, covy_v, covz, covz_v;
			_kalman_filter_x.getState(x, xvel);
			_kalman_filter_x.getCovariance(covx, covx_v);

			_kalman_filter_y.getState(y, yvel);
			_kalman_filter_y.getCovariance(covy, covy_v);

			_kalman_filter_z.getState(z, zvel);
			_kalman_filter_z.getCovariance(covz, covz_v);

			target_pose.is_static = true;

			target_pose.rel_pos_valid = true;
			target_pose.rel_vel_valid = true;
			target_pose.x_rel = x;
			target_pose.y_rel = y;
			target_pose.z_rel = z;
			target_pose.vx_rel = xvel;
			target_pose.vy_rel = yvel;
			target_pose.vz_rel = zvel;

			target_pose.cov_x_rel = covx;
			target_pose.cov_y_rel = covy;
			target_pose.cov_z_rel = covz;

			target_pose.cov_vx_rel = covx_v;
			target_pose.cov_vy_rel = covy_v;
			target_pose.cov_vz_rel = covz_v;

			if (_vehicleLocalPositionValid && _vehicleLocalPosition.xy_valid) {
				target_pose.x_abs = x + _vehicleLocalPosition.x;
				target_pose.y_abs = y + _vehicleLocalPosition.y;
				target_pose.z_abs = z + _vehicleLocalPosition.z;
				target_pose.abs_pos_valid = true;

			} else {
				target_pose.abs_pos_valid = false;
			}

			if (_targetPosePub == nullptr) {
				_targetPosePub = orb_advertise(ORB_ID(landing_target_pose), &target_pose);

			} else {
				orb_publish(ORB_ID(landing_target_pose), _targetPosePub, &target_pose);
			}

			_lastUpdate = hrt_absolute_time();
			_lastPredict = _lastUpdate;
		}

		landing_target_innovations_s target_innovations {};

		float innov_x, innov_cov_x, innov_y, innov_cov_y, innov_z, innov_cov_z;
		_kalman_filter_x.getInnovations(innov_x, innov_cov_x);
		_kalman_filter_y.getInnovations(innov_y, innov_cov_y);
		_kalman_filter_z.getInnovations(innov_z, innov_cov_z);

		target_innovations.timestamp = _targetDetection.timestamp;
		target_innovations.innov_x = innov_x;
		target_innovations.innov_cov_x = innov_cov_x;
		target_innovations.innov_y = innov_y;
		target_innovations.innov_cov_y = innov_cov_y;
		target_innovations.innov_z = innov_z;
		target_innovations.innov_cov_z = innov_cov_z;

		if (_targetInnovationsPub == nullptr) {
			_targetInnovationsPub = orb_advertise(ORB_ID(landing_target_innovations), &target_innovations);

		} else {
			orb_publish(ORB_ID(landing_target_innovations), _targetInnovationsPub, &target_innovations);
		}
	}

}

void LandingTargetEkf::_check_params(const bool force)
{
	bool updated;
	parameter_update_s paramUpdate;

	orb_check(_parameterSub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _parameterSub, &paramUpdate);
	}

	if (updated || force) {
		_update_params();
	}
}

void LandingTargetEkf::_initialize_topics()
{
	_vehicleLocalPositionSub = orb_subscribe(ORB_ID(vehicle_local_position));
	_attitudeSub = orb_subscribe(ORB_ID(vehicle_attitude));
	_sensorBiasSub = orb_subscribe(ORB_ID(sensor_bias));
	_targetDetectionSub = orb_subscribe(ORB_ID(landing_target_detection));
	_parameterSub = orb_subscribe(ORB_ID(parameter_update));
}

void LandingTargetEkf::_update_topics()
{
	_vehicleLocalPositionValid = _orb_update(ORB_ID(vehicle_local_position), _vehicleLocalPositionSub,
				     &_vehicleLocalPosition);
	_vehicleAttitudeValid = _orb_update(ORB_ID(vehicle_attitude), _attitudeSub, &_vehicleAttitude);
	_sensorBiasValid = _orb_update(ORB_ID(sensor_bias), _sensorBiasSub, &_sensorBias);

	_targetDetectionValid = _orb_update(ORB_ID(landing_target_detection), _targetDetectionSub, &_targetDetection);
}


bool LandingTargetEkf::_orb_update(const struct orb_metadata *meta, int handle, void *buffer)
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

void LandingTargetEkf::_update_params()
{
	param_get(_paramHandle.acc_unc, &_params.acc_unc);
	param_get(_paramHandle.meas_unc, &_params.meas_unc);
	param_get(_paramHandle.pos_unc_init, &_params.pos_unc_init);
	param_get(_paramHandle.vel_unc_init, &_params.vel_unc_init);
}


} // namespace landing_target_ekf
