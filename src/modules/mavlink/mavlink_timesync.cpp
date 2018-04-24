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
 * @file mavlink_timesync.cpp
 * Mavlink timesync implementation.
 *
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 */

#include "mavlink_timesync.h"
#include "mavlink_main.h"

MavlinkTimesync::MavlinkTimesync(Mavlink *mavlink) :
	_timesync_status_pub(nullptr),
	_sequence(0),
	_bad_estimate_count(0),
	_filter_alpha(ALPHA_GAIN_INITIAL),
	_filter_beta(BETA_GAIN_INITIAL),
	_time_offset(0.0),
	_time_skew(0.0),
	_mavlink(mavlink)
{
}
MavlinkTimesync::~MavlinkTimesync()
{
	if (_timesync_status_pub) {
		orb_unadvertise(_timesync_status_pub);
	}
}

void
MavlinkTimesync::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_TIMESYNC: {

			mavlink_timesync_t tsync = {};
			mavlink_msg_timesync_decode(msg, &tsync);

			const uint64_t now = hrt_absolute_time();

			if (tsync.tc1 == 0) {			// Message originating from remote system, timestamp and return it

				mavlink_timesync_t rsync;

				rsync.tc1 = now * 1000ULL;
				rsync.ts1 = tsync.ts1;

				mavlink_msg_timesync_send_struct(_mavlink->get_channel(), &rsync);

				return;

			} else if (tsync.tc1 > 0) {		// Message originating from this system, compute time offset from it

				// Calculate time offset between this system and the remote system, assuming RTT for
				// the timesync packet is roughly equal both ways.
				int64_t offset_us = (int64_t)((tsync.ts1 / 1000ULL) + now - (tsync.tc1 / 1000ULL) * 2) / 2 ;

				// TODO : where does the 0 RTT come from???
				// Calculate the time it took the timesync packet to bounce back to us from remote system (RTT)
				uint64_t rtt_us = now - (tsync.ts1 / 1000ULL);

				// Calculate the difference of this sample from the current estimate
				uint64_t deviation = llabs((int64_t)_time_offset - offset_us);

				if (rtt_us < MAX_RTT_SAMPLE) {	// Only use samples with low RTT

					if (sync_converged() && (deviation > MAX_DEVIATION_SAMPLE)) {

						// Increment the counter if we have a good estimate and are getting samples far from the estimate
						_bad_estimate_count++;

						// We reset the filter if we received 5 consecutive samples which violate our present estimate.
						// This is most likely due to a time jump on the offboard system.
						if (_bad_estimate_count > MAX_CONSECUTIVE_BAD_ESTIMATES) {
							// Reset the filter
							reset_filter();
							PX4_ERR("[timesync] Time jump detected. Resetting time synchroniser.");
						}

					} else {

						// Filter gain scheduling
						if (!sync_converged()) {
							// Interpolate with a sigmoid function
							float progress = ((float)_sequence) / CONVERGENCE_WINDOW;
							float p = 1.0f - expf(0.5f * (1.0f - 1.0f / (1.0f - progress)));
							_filter_alpha = p * (float)ALPHA_GAIN_FINAL + (1.0f - p) * (float)ALPHA_GAIN_INITIAL;
							_filter_beta = p * (float)BETA_GAIN_FINAL + (1.0f - p) * (float)BETA_GAIN_INITIAL;

						} else {
							_filter_alpha = ALPHA_GAIN_FINAL;
							_filter_beta = BETA_GAIN_FINAL;
						}

						// Perform filter update
						add_sample(offset_us);

						// Increment sequence counter after filter update
						_sequence++;

						// Reset bad estimate count after successful filter update
						_bad_estimate_count = 0;
					}

				} else {
					PX4_WARN("[timesync] Dropping sample - High RTT : %llu ms", rtt_us / 1000ULL);
				}

				// Publish status message
				struct timesync_status_s tsync_status = {};

				tsync_status.timestamp = hrt_absolute_time();
				tsync_status.remote_timestamp = tsync.tc1 / 1000ULL;
				tsync_status.observed_offset = offset_us;
				tsync_status.estimated_offset = (int64_t)_time_offset;
				tsync_status.round_trip_time = rtt_us;

				if (_timesync_status_pub == nullptr) {
					_timesync_status_pub = orb_advertise(ORB_ID(timesync_status), &tsync_status);

				} else {
					orb_publish(ORB_ID(timesync_status), _timesync_status_pub, &tsync_status);
				}

			}

			break;
		}

	case MAVLINK_MSG_ID_SYSTEM_TIME: {

			mavlink_system_time_t time;
			mavlink_msg_system_time_decode(msg, &time);

			timespec tv = {};
			px4_clock_gettime(CLOCK_REALTIME, &tv);

			// date -d @1234567890: Sat Feb 14 02:31:30 MSK 2009
			bool onb_unix_valid = (unsigned long long)tv.tv_sec > PX4_EPOCH_SECS;
			bool ofb_unix_valid = time.time_unix_usec > PX4_EPOCH_SECS * 1000ULL;

			if (!onb_unix_valid && ofb_unix_valid) {
				tv.tv_sec = time.time_unix_usec / 1000000ULL;
				tv.tv_nsec = (time.time_unix_usec % 1000000ULL) * 1000ULL;

				if (px4_clock_settime(CLOCK_REALTIME, &tv)) {
					PX4_ERR("[timesync] Failed setting realtime clock");
				}
			}

			break;
		}

	default:
		break;
	}
}

uint64_t
MavlinkTimesync::sync_stamp(uint64_t usec)
{
	if (abs(_time_offset) > DBL_EPSILON) {
		return (usec + (int64_t)_time_offset);

	} else {
		return hrt_absolute_time();
	}
}


bool
MavlinkTimesync::sync_converged()
{
	return _sequence >= CONVERGENCE_WINDOW;
}


void
MavlinkTimesync::add_sample(int64_t offset_us)
{
	/* Online exponential smoothing filter. The derivative of the estimate is also
	 * estimated in order to produce an estimate without steady state lag:
	 * https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing
	 */

	double time_offset_prev = _time_offset;

	if (abs(_time_offset) < DBL_EPSILON) {			// First offset sample
		_time_offset = offset_us;

	} else {
		// Update the clock offset estimate
		_time_offset = _filter_alpha * offset_us + (1.0 - _filter_alpha) * (_time_offset + _time_skew);

		// Update the clock skew estimate
		_time_skew = _filter_beta * (_time_offset - time_offset_prev) + (1.0 - _filter_beta) * _time_skew;
	}

}

void
MavlinkTimesync::reset_filter()
{
	// Do a full reset of all statistics and parameters
	_sequence = 0;
	_bad_estimate_count = 0;
	_filter_alpha = ALPHA_GAIN_INITIAL;
	_filter_beta = BETA_GAIN_INITIAL;
	_time_offset = 0.0;
	_time_skew = 0.0;

}