/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file landing_target_ekf_main.cpp
 * Landing target position estimator. Filter and publish the position of a landing target on the ground as observed by an onboard sensor.
 *
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 *
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>

#include "LandingTargetEkf.h"

namespace landing_target_ekf
{

static bool thread_should_exit = false;	/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;			/**< Handle of daemon task / thread */

/* Run main loop at this rate in Hz. */
static constexpr uint32_t landing_target_ekf_UPDATE_RATE_HZ = 50;

/**
 * Landing target position estimator app start / stop handling function
 * @ingroup apps
 */
extern "C" __EXPORT int landing_target_ekf_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int landing_target_ekf_thread_main(int argc, char *argv[]);

/**
* Main entry point for this module
**/
int landing_target_ekf_main(int argc, char *argv[])
{

	if (argc < 2) {
		goto exiterr;
	}

	if (argc >= 2 && !strcmp(argv[1], "start")) {
		if (thread_running) {
			PX4_INFO("already running");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("landing_target_ekf",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 landing_target_ekf_thread_main,
						 (argv) ? (char *const *)&argv[2] : nullptr);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;

		if (!thread_running) {
			PX4_WARN("landing_target_ekf not running");
		}

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			PX4_INFO("running");

		} else {
			PX4_INFO("not started");
		}

		return 0;
	}

exiterr:
	PX4_WARN("usage: landing_target_ekf {start|stop|status}");
	return 1;
}

int landing_target_ekf_thread_main(int argc, char *argv[])
{
	PX4_DEBUG("starting");

	thread_running = true;

	LandingTargetEkf est;

	while (!thread_should_exit) {
		est.update();
		// TODO : proper scheduling and polling on IMU
		usleep(1000000 / landing_target_ekf_UPDATE_RATE_HZ);
	}

	PX4_DEBUG("exiting");

	thread_running = false;

	return 0;
}

}
