/****************************************************************************
 *
 *   Copyright (c) 2018 Mohammed Kabir. All rights reserved.
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

#include <stdint.h>

#include <px4_defines.h>
#include <px4_module.h>
#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>

#include <cmath>
#include <cstring>

#include <lib/mathlib/mathlib.h>
#include <lib/cdev/CDev.hpp>
#include <perf/perf_counter.h>
#include <px4_module_params.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vesc_state.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <mixer/mixer.h>
#include <pwm_limit/pwm_limit.h>

#include "vesc_common.h"

#include "drv_vesc.h"

#if !defined(DEVICE_ARGUMENT_MAX_LENGTH)
#  define DEVICE_ARGUMENT_MAX_LENGTH 32
#endif

// uorb update rate for control groups in miliseconds
#if !defined(VESC_CTRL_UORB_UPDATE_INTERVAL)
#  define VESC_CTRL_UORB_UPDATE_INTERVAL 2  // [ms] min: 2, max: 100
#endif

using namespace time_literals;
/*
 * This driver connects to VESCs via serial.
 */
class VESC : public cdev::CDev, public ModuleBase<VESC>, public ModuleParams
{
public:
	VESC(char const *const device);
	virtual ~VESC();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static VESC *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	virtual int init();
	void cycle();

private:
	char 			_device[DEVICE_ARGUMENT_MAX_LENGTH];
	int 			_uart_fd = -1;
	UARTBuffer 		_uart_buffer = {};
	bool 			_is_armed = false;
	int				_armed_sub = -1;
	int 			_params_sub = -1;
	actuator_armed_s	_armed = {};

	int			_control_sub;
	actuator_controls_s	_control;
	px4_pollfd_struct_t	_poll_fds[2];

	orb_advert_t _state_pub = nullptr;

	hrt_abstime _last_request_time = 0;

	void subscribe();
	void send_erpm_sp(int32_t erpm);
	void send_current_sp(float current);
	void send_brake_current_sp(float current);
	void send_steering_sp(float angle);
	void send_feedback_request();

	void update_params(bool force = false);

	/** @note Declare local parameters using defined parameters. */
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VESC_CTL_MODE>)  _param_control_mode,
		(ParamInt<px4::params::VESC_MAX_ERPM>)  _param_max_erpm,
		(ParamFloat<px4::params::VESC_MAX_CURRENT>)  _param_max_current,
		(ParamFloat<px4::params::VESC_BRK_CURRENT>)  _param_max_braking_current,
		(ParamFloat<px4::params::VESC_ANGLE_TRIM>)  _param_angle_trim,
		(ParamFloat<px4::params::VESC_ANGLE_MIN>)  _param_angle_min,
		(ParamFloat<px4::params::VESC_ANGLE_MAX>)  _param_angle_max
	)

};

VESC::VESC(char const *const device):
	CDev(VESC_DEVICE_PATH),
	ModuleParams(nullptr),
	_control_sub(-1)
{
	strncpy(_device, device, sizeof(_device));
	_device[sizeof(_device) - 1] = '\0';  // Fix in case of overflow

}

VESC::~VESC()
{
	orb_unsubscribe(_control_sub);
	orb_unsubscribe(_armed_sub);
	orb_unsubscribe(_params_sub);

	vesc_common::deinitialise_uart(_uart_fd);

	PX4_INFO("stopping");
}

/** @see ModuleBase */
VESC *VESC::instantiate(int argc, char *argv[])
{
	/* Parse arguments */
	const char *device = nullptr;

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	if (argc < 1) {
		print_usage("not enough arguments");
		return nullptr;
	}

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;
		}
	}

	if (device == nullptr || strlen(device) == 0) {
		print_usage("no device specified");
		return nullptr;
	}

	VESC *vesc = new VESC(device);

	if (vesc == nullptr) {
		PX4_ERR("failed to instantiate module");
		return nullptr;
	}

	if (vesc->init() != 0) {
		PX4_ERR("failed to initialize module");
		delete vesc;
		return nullptr;
	}

	return vesc;
}

/** @see ModuleBase */
int VESC::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VESC::init()
{
	int ret;

	ret = vesc_common::initialise_uart(_device, _uart_fd);

	if (ret != 0) {
		PX4_ERR("failed to initialise UART.");
		return ret;
	}

	/* Respect boot time required by the VESC firmware */
	hrt_abstime uptime_us = hrt_absolute_time();

	if (uptime_us < MAX_BOOT_TIME_MS * 1000) {
		usleep((MAX_BOOT_TIME_MS * 1000) - uptime_us);
	}

	/* do regular cdev init */
	ret = CDev::init();

	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_control_sub = orb_subscribe(ORB_ID(actuator_controls_0));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));

	orb_set_interval(_control_sub, VESC_CTRL_UORB_UPDATE_INTERVAL);

	update_params(true);

	// Primary wakeup source
	_poll_fds[0].fd = _control_sub;
	_poll_fds[0].events = POLLIN;

	// Secondary wakeup source
	_poll_fds[1].fd = _uart_fd;
	_poll_fds[1].events = POLLIN;

	return ret;
}

void VESC::send_erpm_sp(int32_t erpm)
{
	// Create packet payload
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_RPM;
	vesc_common::buffer_append_int32(payload, erpm, &index);

	// Send packet
	int ret = vesc_common::send_payload(_uart_fd, payload, 5);

	if (ret < 1) {
		PX4_ERR("TX ERROR: ret: %d, errno: %d", ret, errno);
	}
}

void VESC::send_current_sp(float current)
{
	// Create packet payload
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT;
	vesc_common::buffer_append_int32(payload, int32_t(current * 1000), &index);

	// Send packet
	int ret = vesc_common::send_payload(_uart_fd, payload, 5);

	if (ret < 1) {
		PX4_ERR("TX ERROR: ret: %d, errno: %d", ret, errno);
	}
}

void VESC::send_brake_current_sp(float current)
{
	// Create packet payload
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT_BRAKE;
	vesc_common::buffer_append_int32(payload, int32_t(current * 1000), &index);

	// Send packet
	int ret = vesc_common::send_payload(_uart_fd, payload, 5);

	if (ret < 1) {
		PX4_ERR("TX ERROR: ret: %d, errno: %d", ret, errno);
	}
}

void VESC::send_steering_sp(float angle)
{
	// Create packet payload
	int32_t index = 0;
	uint8_t payload[3];

	payload[index++] = COMM_SET_SERVO_POS;
	vesc_common::buffer_append_int16(payload, int16_t(angle * 1000), &index);

	// Send packet
	int ret = vesc_common::send_payload(_uart_fd, payload, 3);

	if (ret < 1) {
		PX4_ERR("TX ERROR: ret: %d, errno: %d", ret, errno);
	}
}

void VESC::send_feedback_request()
{
	// Create packet payload
	uint8_t payload[1];

	payload[0] = COMM_GET_VALUES;

	// Send packet
	int ret = vesc_common::send_payload(_uart_fd, payload, 1);

	if (ret < 1) {
		PX4_ERR("TX ERROR: ret: %d, errno: %d", ret, errno);
	}
}

void VESC::cycle()
{
	/* check if anything updated */
	int ret = px4_poll(_poll_fds, 2, 100);

	/* this would be bad... */
	if (ret < 0) {
		PX4_ERR("poll error %d", errno);
		return;

	} else if (_poll_fds[0].revents & POLLIN) { // Control data
		orb_copy(ORB_ID(actuator_controls_0), _control_sub, &_control);

		// Yaw is normalized [-1, 1]
		float yaw_target = -math::constrain(_control.control[actuator_controls_s::INDEX_YAW], -1.0f, 1.0f);
		float steering_sp = _param_angle_trim.get();

		if (yaw_target < 0.0f) {
			// Map -1 to 0 from angle_min to angle_trim
			steering_sp = _param_angle_min.get() + ((_param_angle_trim.get() - _param_angle_min.get()) * (yaw_target + 1.0f));

		} else if (yaw_target > 0.0f) {
			// Map 0 to 1 from angle_trim to angle_max
			steering_sp = _param_angle_trim.get() + ((_param_angle_max.get() - _param_angle_trim.get()) * yaw_target);
		}

		send_steering_sp(steering_sp);

		if (_param_control_mode.get() == 0) { // Current control mode
			// Throttle is normalized [-1, 1]
			float current_sp = math::constrain(_control.control[actuator_controls_s::INDEX_THROTTLE], -1.0f,
							   1.0f) * _param_max_current.get();
			send_current_sp(current_sp);

			// Brake is normalized [0, 1]
			if (_control.control[actuator_controls_s::INDEX_AIRBRAKES] > 0.0f) {
				float braking_current_sp = -_control.control[actuator_controls_s::INDEX_AIRBRAKES] * _param_max_braking_current.get();
				send_brake_current_sp(braking_current_sp);
			}

		} else if (_param_control_mode.get() == 1) { // ERPM control mode
			// Throttle is normalized [-1, 1]
			int32_t erpm_sp = math::constrain(_control.control[actuator_controls_s::INDEX_THROTTLE], -1.0f,
							  1.0f) * _param_max_erpm.get();

			// Brake is normalized [0, 1]
			if (_control.control[actuator_controls_s::INDEX_AIRBRAKES] > 0.0f) {
				erpm_sp = 0;
			}

			send_erpm_sp(erpm_sp);
		}

	} else if (_poll_fds[1].revents & POLLIN) { // ESC feedback

		// Read UART bytes into ringbuffer
		if (vesc_common::read_uart(_uart_fd, &_uart_buffer) < 0) {
			PX4_ERR("RX ERR failed");
		}

		// Try to parse message payload
		uint8_t payload[256 - MIN_PACKET_SIZE];
		uint8_t payload_len = vesc_common::parse_packet(&_uart_buffer, payload);

		if (payload_len > 0) {
			float erpm = 0.0f;
			if (vesc_common::unpack_payload(payload, payload_len, erpm) >= 0) {

				vesc_state_s state{};
				state.timestamp = hrt_absolute_time();
				state.erpm = fabsf(erpm) > 50.0f ? erpm : 0.0f;
				state.steering_angle = _control.control[actuator_controls_s::INDEX_YAW]; // _param_model_sk;

				int instance;
				orb_publish_auto(ORB_ID(vesc_state), &_state_pub, &state, &instance, ORB_PRIO_HIGH);

			}
		}
	}

	// Request ESC feedback
	if (hrt_elapsed_time(&_last_request_time) > 10_ms) {
		send_feedback_request();
		_last_request_time = hrt_absolute_time();
	}

	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);

		_is_armed = _armed.armed;
	}

	update_params(false);

}

void VESC::update_params(const bool force)
{
	bool updated;
	parameter_update_s param_update;

	orb_check(_params_sub, &updated);

	if (updated || force) {
		ModuleParams::updateParams();
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
	}
}

/** @see ModuleBase */
void VESC::run()
{
	// Main loop
	while (!should_exit()) {
		cycle();
	}
}

/** @see ModuleBase */
int VESC::task_spawn(int argc, char *argv[])
{
	/* start the task */
	_task_id = px4_task_spawn_cmd("vesc",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_ACTUATOR_OUTPUTS,
				      1180,
				      (px4_main_t)&run_trampoline,
				      argv);

	if (_task_id < 0) {
		PX4_ERR("task start failed");
		_task_id = -1;
		return PX4_ERROR;
	}

	// wait until task is up & running
	if (wait_until_running() < 0) {
		_task_id = -1;
		return -1;
	}

	return PX4_OK;
}

/** @see ModuleBase */
int VESC::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module controls a single VESC unit via UART. It listens on the
actuator_controls topics, and sends current (torque) setpoints to the ESC.

### Implementation
Currently the module is implementd as a threaded version only, meaning that it
runs in its own thread instead of on the work queue.

### Example
The module is typically started with:
vesc start -d /dev/ttyS2

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vesc", "driver");

	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<device>", "Device used to talk to the VESC", true);
	return PX4_OK;
}

extern "C" __EXPORT int vesc_main(int argc, char *argv[]);

int vesc_main(int argc, char *argv[])
{
	return VESC::main(argc, argv);
}
