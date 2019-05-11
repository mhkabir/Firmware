/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @author Mohammed Kabir <mhkabir@mit.edu>
 */

#include "GroundRoverAckermannControl.hpp"

extern "C" __EXPORT int gnd_ackermann_control_main(int argc, char *argv[]);

namespace gnd_ackermann_control
{
GroundRoverAckermannControl	*g_control = nullptr;
}

GroundRoverAckermannControl::GroundRoverAckermannControl() :
	ModuleParams(nullptr),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "gnda_dt")),
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "gnda_nani")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "gnda_nano"))
{
}

GroundRoverAckermannControl::~GroundRoverAckermannControl()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	perf_free(_loop_perf);
	perf_free(_nonfinite_input_perf);
	perf_free(_nonfinite_output_perf);

	gnd_ackermann_control::g_control = nullptr;
}

void GroundRoverAckermannControl::parameters_update(int parameter_update_sub, bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	// Check if any parameter updated
	orb_check(parameter_update_sub, &updated);

	// If any parameter updated copy it to: param_upd
	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
	}

	if (force || updated) {
		updateParams();

		pid_init(&_steering_rate_ctrl, PID_MODE_DERIVATIV_CALC, 0.01f);
		pid_set_parameters(&_steering_rate_ctrl,
				   _param_sr_p.get(),
				   _param_sr_i.get(),
				   _param_sr_d.get(),
				   1.0f,
				   1.0f);
	}
}


void
GroundRoverAckermannControl::vehicle_ackermann_setpoint_poll()
{
	bool updated = false;
	orb_check(_ackermann_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_ackermann_setpoint), _ackermann_sp_sub, &_ackermann_sp);
	}
}

int
GroundRoverAckermannControl::task_main_trampoline(int argc, char *argv[])
{
	gnd_ackermann_control::g_control->task_main();
	return 0;
}

void
GroundRoverAckermannControl::task_main()
{
	// Control demands
	_ackermann_sp_sub = orb_subscribe(ORB_ID(vehicle_ackermann_setpoint));

	// State feedback
	_state_sub = orb_subscribe(ORB_ID(vehicle_body_state));

	_params_sub = orb_subscribe(ORB_ID(parameter_update));

	// Get initial parameter set
	parameters_update(_params_sub, true);

	// Get an initial update for all sensor and status data
	vehicle_ackermann_setpoint_poll();

	// Wakeup sources
	px4_pollfd_struct_t fds[1];
	fds[0].fd = _state_sub;
	fds[0].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			PX4_ERR("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		/* only run controller if body state changed */
		if (fds[0].revents & POLLIN) {

			parameters_update(_params_sub, false);

			float delta_t = (hrt_absolute_time() - _timestamp_last) / 1000000.0f;
			_timestamp_last = hrt_absolute_time();

			// Guard against extreme dts
			if (delta_t > 1.0f || fabsf(delta_t) < 0.00001f || !PX4_ISFINITE(delta_t)) {
				PX4_WARN("Extreme dt!");
				delta_t = 0.01f;
			}

			/* load local copies */
			vehicle_body_state_s state;
			orb_copy(ORB_ID(vehicle_body_state), _state_sub, &state);

			vehicle_ackermann_setpoint_poll();

			actuator_controls_s	actuator_outputs{};

			// Run controller if under autonomous control
			if (!_ackermann_sp.manual_passthrough &&
			    PX4_ISFINITE(state.vx) &&
			    PX4_ISFINITE(state.yawspeed)) {

				// Hack for using torque setpoints in manual mode
				actuator_outputs.control[actuator_controls_s::INDEX_SPOILERS] = 0.0f;

				// Run yawrate controller
				float steering_cmd = 0.0f;

				// Only control steering if vehicle is in motion
				if (fabsf(state.vx) > 0.1f) {
					// Model-based feedforward controller
					steering_cmd = _param_mdl_sk.get() * (atanf(_param_mdl_wb.get() * _ackermann_sp.steering_angle_velocity / state.vx));

					// Run feedback controller on top
					steering_cmd += pid_calculate(&_steering_rate_ctrl, _ackermann_sp.steering_angle_velocity, state.yawspeed, 0.0f,
								      delta_t);

					////////////////////////////////////////////////
					// Publish controller performance
					debug_vect_s perf{};
					perf.x = _ackermann_sp.steering_angle_velocity; // Setpoint (theta_dot_des)
					perf.y = state.yawspeed; 				 			// Actual (theta_dot)
					perf.z = steering_cmd; 								// Controller output (u)

					if (_debug_vect_pub != nullptr) {
						orb_publish(ORB_ID(debug_vect), _debug_vect_pub, &perf);

					} else {
						_debug_vect_pub = orb_advertise(ORB_ID(debug_vect), &perf);
					}
					////////////////////////////////////////////////
				}

				// Steering command
				actuator_outputs.control[actuator_controls_s::INDEX_YAW] = steering_cmd;
				
				// Model-based feedforward controller
				float erpm_cmd = (_ackermann_sp.speed / _param_mdl_wr.get()) * 		// omega_wheel
								 (60.0f/(2.0f*M_PI_F)) * 							// rpm_wheel
								 _param_mdl_wm_ratio.get() *						// rpm_motor
								 _param_mdl_motor_poles.get() / 					// erpm_motor
								 _param_mdl_erpm_scaler.get();						// erpm_scaled [-1,1]

				////////////////////////////////////////////////
				// Publish controller performance
				/*
				debug_vect_s perf{};
				perf.x = _ackermann_sp.speed; 		// Setpoint (v_des)
				perf.y = state.vx; 				 	// Actual (v)
				perf.z = erpm_cmd; 					// Controller output (u)

				if (_debug_vect_pub != nullptr) {
					orb_publish(ORB_ID(debug_vect), _debug_vect_pub, &perf);

				} else {
					_debug_vect_pub = orb_advertise(ORB_ID(debug_vect), &perf);
				}*/
				////////////////////////////////////////////////

				// ERPM command
				actuator_outputs.control[actuator_controls_s::INDEX_THROTTLE] = erpm_cmd;

			} else if (_ackermann_sp.manual_passthrough) {

				// Hack for using torque setpoints
				actuator_outputs.control[actuator_controls_s::INDEX_SPOILERS] = 1.0f;

				// Steering
				actuator_outputs.control[actuator_controls_s::INDEX_YAW] = _ackermann_sp.steering_angle;

				// Torque
				if (PX4_ISFINITE(_ackermann_sp.acceleration)) {
					actuator_outputs.control[actuator_controls_s::INDEX_THROTTLE] = _ackermann_sp.acceleration;

					// Braking
					if (fabsf(_ackermann_sp.acceleration) < FLT_EPSILON) {
						actuator_outputs.control[actuator_controls_s::INDEX_AIRBRAKES] = 1.0f;
					}
				}
			}

			// If setpoints are stale, stop and brake
			if (hrt_absolute_time() - _ackermann_sp.timestamp > 100000) { // 100ms
				actuator_outputs.control[actuator_controls_s::INDEX_YAW] = 0.0f;
				actuator_outputs.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
				actuator_outputs.control[actuator_controls_s::INDEX_AIRBRAKES] = 1.0f;
			}

			actuator_outputs.timestamp = hrt_absolute_time();
			actuator_outputs.timestamp_sample = state.timestamp;

			/* publish the actuator controls */
			if (_actuators_pub != nullptr) {
				orb_publish(ORB_ID(actuator_controls_0), _actuators_pub, &actuator_outputs);

			} else {
				_actuators_pub = orb_advertise(ORB_ID(actuator_controls_0), &actuator_outputs);
			}
		}

		perf_end(_loop_perf);
	}

	_control_task = -1;
	_task_running = false;

}

int
GroundRoverAckermannControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("gnd_ackermann_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&GroundRoverAckermannControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		PX4_ERR("Task start failed");
		return -errno;
	}

	return PX4_OK;
}

int gnd_ackermann_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_INFO("Usage: gnd_ackermann_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (gnd_ackermann_control::g_control != nullptr) {
			PX4_ERR("Already running");
			return 1;
		}

		gnd_ackermann_control::g_control = new GroundRoverAckermannControl;

		if (gnd_ackermann_control::g_control == nullptr) {
			PX4_ERR("Alloc failed");
			return 1;
		}

		if (PX4_OK != gnd_ackermann_control::g_control->start()) {
			delete gnd_ackermann_control::g_control;
			gnd_ackermann_control::g_control = nullptr;
			PX4_ERR("Start failed");
			return 1;
		}

		/* check if the waiting is necessary at all */
		if (gnd_ackermann_control::g_control == nullptr || !gnd_ackermann_control::g_control->task_running()) {

			/* avoid memory fragmentation by not exiting start handler until the task has fully started */
			while (gnd_ackermann_control::g_control == nullptr || !gnd_ackermann_control::g_control->task_running()) {
				usleep(50000);
				printf(".");
				fflush(stdout);
			}

			printf("\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (gnd_ackermann_control::g_control == nullptr) {
			PX4_ERR("Not running");
			return 1;
		}

		delete gnd_ackermann_control::g_control;
		gnd_ackermann_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (gnd_ackermann_control::g_control) {
			PX4_INFO("Running");
			return 0;

		} else {
			PX4_ERR("Not running");
			return 1;
		}
	}

	PX4_ERR("Unrecognized command");
	return 1;
}
