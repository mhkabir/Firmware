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
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "gnda_dt")),
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "gnda_nani")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "gnda_nano"))
{
	// Vehicle dynamic limits
	_parameter_handles.v_max = param_find("RAC_VEL_MAX");
	_parameter_handles.a_max = param_find("RAC_ACC_MAX");
	_parameter_handles.s_max = param_find("RAC_STEER_MAX");

	// Velocity control
	_parameter_handles.v_p = param_find("RAC_VEL_P");

	// Acceleration control
	_parameter_handles.a_t_tf = param_find("RAC_ACC_TQ");

	// Battery drop compensation
	_parameter_handles.bat_scale_en = param_find("RAC_BAT_SCALE");

	// Fetch initial parameter values
	parameters_update();
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

void
GroundRoverAckermannControl::parameters_update()
{

	param_get(_parameter_handles.v_max, &(_parameters.v_max));
	param_get(_parameter_handles.a_max, &(_parameters.a_max));
	param_get(_parameter_handles.s_max, &(_parameters.s_max));

	param_get(_parameter_handles.v_p, &(_parameters.v_p));

	param_get(_parameter_handles.a_t_tf, &(_parameters.a_t_tf));

	param_get(_parameter_handles.bat_scale_en, &_parameters.bat_scale_en);

	pid_init(&_velocity_ctrl, PID_MODE_DERIVATIV_SET, 0.01f);
	pid_set_parameters(&_velocity_ctrl,
			   _parameters.v_p,
			   0.0f,
			   0.0f,
			   0.0f,
			   _parameters.a_max);
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

void
GroundRoverAckermannControl::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
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
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	parameters_update();

	// Get an initial update for all sensor and status data
	vehicle_ackermann_setpoint_poll();
	battery_status_poll();

	// Wakeup sources
	px4_pollfd_struct_t fds[2];
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _state_sub;
	fds[1].events = POLLIN;

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

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if body state changed */
		if (fds[1].revents & POLLIN) {

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
			battery_status_poll();

			actuator_controls_s	actuator_outputs{};

			// Run controller if under autonomous control
			if (!_ackermann_sp.manual_passthrough &&
			    PX4_ISFINITE(_ackermann_sp.acceleration) &&
			    PX4_ISFINITE(state.ax)) {

				float acceleration_sp = _ackermann_sp.acceleration;

				// If needed, run velocity controller first to generate acceleration setpoint
				if (PX4_ISFINITE(_ackermann_sp.speed) && PX4_ISFINITE(state.vx)) {

					// TODO constrain velocity setpoint before use
					acceleration_sp = pid_calculate(&_velocity_ctrl, _ackermann_sp.speed, state.vx, state.ax, delta_t);

					// TODO : Limit acceleration demand
				}

				// Generate torque setpoint using transfer function
				float torque_sp = acceleration_sp * _parameters.a_t_tf;

				// Compensate control effort for battery voltage drop if enabled
				if(_parameters.bat_scale_en && _battery_status.scale > 0.0f) {
					torque_sp *= _battery_status.scale;
				}

				// Steering
				actuator_outputs.control[actuator_controls_s::INDEX_YAW] = _ackermann_sp.steering_angle; // TODO

				// Torque
				actuator_outputs.control[actuator_controls_s::INDEX_THROTTLE] = torque_sp;

			} else if (_ackermann_sp.manual_passthrough) {

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
