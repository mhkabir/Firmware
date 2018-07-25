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
 * @file camera_capture.cpp
 *
 * Online and offline geotagging from camera feedback
 *
 * @author Mohammed Kabir <kabir@uasys.io>
 */

#include "camera_capture.hpp"

#define commandParamToInt(n) static_cast<int>(n >= 0 ? n + 0.5f : n - 0.5f)

namespace camera_capture
{
CameraCapture	*g_camera_capture;
}

CameraCapture::CameraCapture() :
	_trigger_pub(nullptr),
	_command_ack_pub(nullptr),
	_command_sub(-1),
	_capture_enabled{},
	_capture_overflows{},
	_capture_seq{},
	_last_fall_time{},
	_last_exposure_time{},
	_time_offset(0.0) // in seconds
{

	memset(&_work, 0, sizeof(_work));

	struct camera_trigger_s trigger = {};
	_trigger_pub = orb_advertise_queue(ORB_ID(camera_trigger), &trigger, 3);
}

CameraCapture::~CameraCapture()
{
	camera_capture::g_camera_capture = nullptr;
}

void
CameraCapture::capture_callback(uint32_t chan_index,
				hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	uint8_t cam_id = chan_index - PIN_BASE;

	if (edge_state == 0) {											// Falling edge
		// Timestamp it
		_last_fall_time[cam_id] = edge_time;

	} else if (edge_state == 1 && _last_fall_time[cam_id] > 0) {			// Rising edge and got falling before
		struct camera_trigger_s	trigger {};

		// Calculate exposure time
		_last_exposure_time[cam_id] = edge_time - _last_fall_time[cam_id];

		trigger.timestamp = edge_time - (_last_exposure_time[cam_id] / 2); //+ uint64_t(_time_offset * 1000000.0f);	// Get timestamp of mid-exposure
		trigger.camera_id = cam_id;
		trigger.seq = _capture_seq[cam_id]++;

		orb_publish(ORB_ID(camera_trigger), _trigger_pub, &trigger);

	}

	_capture_overflows[cam_id] = overflow;

}

void
CameraCapture::capture_trampoline(void *context, uint32_t chan_index,
				  hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	camera_capture::g_camera_capture->capture_callback(chan_index, edge_time, edge_state, overflow);
}

void
CameraCapture::cycle_trampoline(void *arg)
{

	CameraCapture *cap = reinterpret_cast<CameraCapture *>(arg);

	if (cap->_command_sub < 0) {
		cap->_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	}

	bool updated = false;
	orb_check(cap->_command_sub, &updated);

	// Command handling
	if (updated) {

		vehicle_command_s cmd;
		orb_copy(ORB_ID(vehicle_command), cap->_command_sub, &cmd);

		// TODO : this should eventuallly be a capture control command
		if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL) {

			// get camera ID
			uint8_t cam_id = commandParamToInt(cmd.param4);

			// Reset frame sequence
			bool reset_seq = commandParamToInt(cmd.param2) == 1;

			// Enable/disable signal capture
			if (commandParamToInt(cmd.param1) == 1) {
				cap->set_capture_control(cam_id, true, reset_seq);

			} else if (commandParamToInt(cmd.param1) == 0) {
				cap->set_capture_control(cam_id, false, reset_seq);

			}

			// Acknowledge the command
			vehicle_command_ack_s command_ack = {
				.timestamp = 0,
				.result_param2 = 0,
				.command = cmd.command,
				.result = (uint8_t)vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED,
				.from_external = false,
				.result_param1 = 0,
				.target_system = cmd.source_system,
				.target_component = cmd.source_component
			};

			if (cap->_command_ack_pub == nullptr) {
				cap->_command_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &command_ack,
							vehicle_command_ack_s::ORB_QUEUE_LENGTH);

			} else {
				orb_publish(ORB_ID(vehicle_command_ack), cap->_command_ack_pub, &command_ack);

			}
		}

	}

	work_queue(LPWORK, &_work, (worker_t)&CameraCapture::cycle_trampoline, camera_capture::g_camera_capture,
		   USEC2TICK(100000)); // 100ms
}

void
CameraCapture::set_capture_control(uint8_t cam_id, bool enable, bool reset_seq)
{
	uint32_t pin = PIN_BASE + cam_id;

	if (enable && !_capture_enabled[cam_id]) {
		// Enable signal capture
		reset_statistics(cam_id, reset_seq);
		up_input_capture_set(pin, Both, 0, &CameraCapture::capture_trampoline, this);

		_capture_enabled[cam_id] = true;

	} else if (!enable && _capture_enabled[cam_id]) {
		// Disable signal capture
		up_input_capture_set(pin, Disabled, 0, NULL, NULL);
		reset_statistics(cam_id, reset_seq);

		_capture_enabled[cam_id] = false;

	}

}

void
CameraCapture::reset_statistics(uint8_t cam_id, bool reset_seq)
{

	if (reset_seq) { _capture_seq[cam_id] = 0; }

	_last_fall_time[cam_id] = 0;
	_last_exposure_time[cam_id] = 0;

}

void
CameraCapture::start()
{
	// start to monitor at low rates for capture control commands
	work_queue(LPWORK, &_work, (worker_t)&CameraCapture::cycle_trampoline, this, USEC2TICK(1));
}

void
CameraCapture::stop()
{

	work_cancel(LPWORK, &_work);

	if (camera_capture::g_camera_capture != nullptr) {
		delete (camera_capture::g_camera_capture);
	}
}

void
CameraCapture::status()
{
	for (uint8_t cam_id = 0; cam_id < NUM_CAMERAS; cam_id++) {
		PX4_INFO("Camera %u status :", cam_id);
		PX4_INFO("  Capture enabled : %s", _capture_enabled[cam_id] ? "YES" : "NO");
		PX4_INFO("  Number of overflows : %u", _capture_overflows[cam_id]);
		PX4_INFO("  Frame sequence : %u", _capture_seq[cam_id]);
		PX4_INFO("  Last fall timestamp : %llu", _last_fall_time[cam_id]);
		PX4_INFO("  Last exposure time : %0.2f ms", double(_last_exposure_time[cam_id]) / 1000.0);
	}
}

static int usage()
{
	PX4_INFO("usage: camera_capture {start|stop|status}\n");
	return 1;
}

extern "C" __EXPORT int camera_capture_main(int argc, char *argv[]);

int camera_capture_main(int argc, char *argv[])
{
	if (argc < 2) {
		return usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (camera_capture::g_camera_capture != nullptr) {
			PX4_WARN("already running");
			return 0;
		}

		camera_capture::g_camera_capture = new CameraCapture();

		if (camera_capture::g_camera_capture == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}

		camera_capture::g_camera_capture->start();
		return 0;
	}

	if (camera_capture::g_camera_capture == nullptr) {
		PX4_WARN("not running");
		return 1;

	} else if (!strcmp(argv[1], "stop")) {
		camera_capture::g_camera_capture->stop();

	} else if (!strcmp(argv[1], "status")) {
		camera_capture::g_camera_capture->status();

	} else {
		return usage();
	}

	return 0;
}
