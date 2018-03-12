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
	_time_offset_avg_alpha(0.4),
	_time_offset(0),
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

			struct timesync_status_s tsync_status = {};

			uint64_t now_ns = hrt_absolute_time() * 1000LL ;

			if (tsync.tc1 == 0) {			// Message from remote system, timestamp and return it

				mavlink_timesync_t rsync; // return timestamped sync message

				rsync.tc1 = now_ns;
				rsync.ts1 = tsync.ts1;

				mavlink_msg_timesync_send_struct(_mavlink->get_channel(), &rsync);

				return;

			} else if (tsync.tc1 > 0) {		// Message from this system, compute time offset from it

				int64_t offset_ns = (int64_t)(tsync.ts1 + now_ns - tsync.tc1 * 2) / 2 ;
				int64_t dt = _time_offset - offset_ns;

				if (dt > 10000000LL || dt < -10000000LL) { // 10 millisecond skew
					_time_offset = offset_ns;

					// Provide a warning only if not syncing initially
					if (_time_offset != 0) {
						PX4_ERR("[timesync] Hard setting offset.");
					}

				} else {
					smooth_time_offset(offset_ns);
				}
			}

			// Publish status message
			tsync_status.offset_ns = _time_offset ;
			//tsync_status.rtt_ns = rtt_ns ;

			if (_timesync_status_pub == nullptr) {
				_timesync_status_pub = orb_advertise(ORB_ID(timesync_status), &tsync_status);

			} else {
				orb_publish(ORB_ID(timesync_status), _timesync_status_pub, &tsync_status);
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
					PX4_ERR("failed setting clock");
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

	if (_time_offset != 0) {
		return usec + (_time_offset / 1000) ;

	} else {
		return hrt_absolute_time();
	}
}


void
MavlinkTimesync::smooth_time_offset(int64_t offset_ns)
{
	/* The closer alpha is to 1.0, the faster the moving
	 * average updates in response to new offset samples.
	 */

	_time_offset = (_time_offset_avg_alpha * offset_ns) + ((float)1.0 - _time_offset_avg_alpha) * _time_offset;
}