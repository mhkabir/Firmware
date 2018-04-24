/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_parameters.h
 * Mavlink parameters manager definition.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Beat Kueng <beat@px4.io>
 */

#pragma once

#include "mavlink_bridge_header.h"

#include <uORB/uORB.h>
#include <uORB/topics/timesync_status.h>

#include <drivers/drv_hrt.h>

#include <math.h>
#include <float.h>

#define PX4_EPOCH_SECS 1234567890ULL

// Filter gains
#define ALPHA_GAIN_INITIAL 				0.05
#define BETA_GAIN_INITIAL 				0.05
#define ALPHA_GAIN_FINAL 				0.003
#define BETA_GAIN_FINAL 				0.003

// Filter gain scheduling
#define CONVERGENCE_WINDOW 				500

// Outlier rejection and filter reset
// TODO : we should be able to automatically determine these using ping statistics
#define MAX_RTT_SAMPLE 					10000ULL 	// 10ms
#define MAX_DEVIATION_SAMPLE 			100000ULL 	// 100ms
#define MAX_CONSECUTIVE_BAD_ESTIMATES 	5

class Mavlink;

class MavlinkTimesync
{
public:
	explicit MavlinkTimesync(Mavlink *mavlink);
	~MavlinkTimesync();

	void handle_message(const mavlink_message_t *msg);

	/**
	 * Convert remote timestamp to local hrt time (usec)
	 * Use synchronised time if available, monotonic boot time otherwise
	 */
	uint64_t sync_stamp(uint64_t usec);

private:

	/* do not allow top copying this class */
	MavlinkTimesync(MavlinkTimesync &);
	MavlinkTimesync &operator = (const MavlinkTimesync &);

protected:

	/**
	 * Online exponential filter to smooth time offset
	 */
	void add_sample(int64_t offset_us);

	/**
	 * Return true if the timesync algorithm converged to a good estimate,
	 * return false otherwise
	 */
	bool sync_converged();

	/**
	 * Reset the exponential filter and its states
	 */
	void reset_filter();

	orb_advert_t _timesync_status_pub;

	// Gain scheduling
	uint32_t _sequence;

	// Outlier rejection and filter reset
	uint32_t _bad_estimate_count;

	// Timesync statistics
	double _filter_alpha;
	double _filter_beta;
	double _time_offset;
	double _time_skew;

	Mavlink *_mavlink;
};
