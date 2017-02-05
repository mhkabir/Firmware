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
 * @file skycontroller_interface.cpp
 * @author Mohammed Kabir <kabir@uasys.io>
 *
 * This driver turns a Parrot Skycontroller 2 ino a generic 
 * mavlink RC controller.
 *
 */


#include <stdint.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>
#include <string.h>
#include <termios.h>

#include <net/if.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <drivers/device/device.h>

#include <v1.0/mavlink_types.h>
#include <v1.0/common/mavlink.h>

namespace skycontroller_interface
{

static const uint8_t mavlink_message_lengths[256] = MAVLINK_MESSAGE_LENGTHS;
static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;

pthread_mutex_t		_send_mutex;
static int _socket_fd = -1;
struct sockaddr_in _network_addr = {};	// Skycontroller
struct sockaddr_in _remote_addr = {};	// Vehicle
unsigned short _network_port = 14550;	// Skycontroller
unsigned short _remote_port = 14556;	// Vehicle

volatile bool _task_should_exit = false; // flag indicating if skycontroller_interface task should exit
static bool _is_running = false;         // flag indicating if skycontroller_interface app is running
static px4_task_t _task_handle = -1;     // handle to the task main thread

//hrt_abstime _last_actuator_controls_received = 0;

// Print out the usage information
void usage();

void start();

/** skycontroller_interface stop */
void stop();

void init_udp();

void deinit_udp();

int send_udp(uint8_t *buf, unsigned len);

void send_rc_mavlink();

/** task main trampoline function */
void task_main_trampoline(int argc, char *argv[]);

/** skycontroller_interface thread primary entry point */
void task_main(int argc, char *argv[]);

void task_main(int argc, char *argv[])
{
	//char serial_buf[128];
	//mavlink_status_t serial_status = {};

	//_rc_sub = orb_subscribe(ORB_ID(input_rc));

	init_udp();

	// we wait for uart actuator controls messages from snapdragon
	//px4_pollfd_struct_t fds[1];
	//fds[0].fd = _uart_fd;
	//fds[0].events = POLLIN;

	//param_get(param_find("PWM_DISARMED"), &_pwm_disarmed);

	while (!_task_should_exit) {

		// wait for up to 100ms for data
		//int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		// timed out
		//if (pret == 0) {
			// let's run the loop anyway to send RC
		//}

		//if (pret < 0) {
		//	PX4_WARN("skycontroller_interface poll error");
			// sleep a bit before next try
		//	usleep(100000);
		//	continue;
		//}
		/*
		if (fds[0].revents & POLLIN) {
			int len = ::read(_uart_fd, serial_buf, sizeof(serial_buf));

			if (len > 0) {
				mavlink_message_t msg;

				for (int i = 0; i < len; ++i) {
					if (mavlink_parse_char(MAVLINK_COMM_0, serial_buf[i], &msg, &serial_status)) {
						// have a message, handle it
						handle_message(&msg);
					}
				}
			}
		}*/

		// check if we have new rc data, if yes send it to snapdragon
		//bool rc_updated = false;
		//orb_check(_rc_sub, &rc_updated);

		//if (rc_updated) {
		//	orb_copy(ORB_ID(input_rc), _rc_sub, &_rc);
			// send mavlink message
			send_rc_mavlink();
		//}
	}

	deinit_udp();

}

void send_rc_mavlink()
{
	mavlink_rc_channels_t rc_message;
	//rc_message.time_boot_ms = hrt_absolute_time() / 1000;
	/*
	rc_message.chancount = _rc.channel_count;
	rc_message.chan1_raw = (_rc.channel_count > 0) ? _rc.values[0] : UINT16_MAX;
	rc_message.chan2_raw = (_rc.channel_count > 1) ? _rc.values[1] : UINT16_MAX;
	rc_message.chan3_raw = (_rc.channel_count > 2) ? _rc.values[2] : UINT16_MAX;
	rc_message.chan4_raw = (_rc.channel_count > 3) ? _rc.values[3] : UINT16_MAX;
	rc_message.chan5_raw = (_rc.channel_count > 4) ? _rc.values[4] : UINT16_MAX;
	rc_message.chan6_raw = (_rc.channel_count > 5) ? _rc.values[5] : UINT16_MAX;
	rc_message.chan7_raw = (_rc.channel_count > 6) ? _rc.values[6] : UINT16_MAX;
	rc_message.chan8_raw = (_rc.channel_count > 7) ? _rc.values[7] : UINT16_MAX;
	rc_message.chan9_raw = (_rc.channel_count > 8) ? _rc.values[8] : UINT16_MAX;
	rc_message.chan10_raw = (_rc.channel_count > 9) ? _rc.values[9] : UINT16_MAX;
	rc_message.chan11_raw = (_rc.channel_count > 10) ? _rc.values[10] : UINT16_MAX;
	rc_message.chan12_raw = (_rc.channel_count > 11) ? _rc.values[11] : UINT16_MAX;
	rc_message.chan13_raw = (_rc.channel_count > 12) ? _rc.values[12] : UINT16_MAX;
	rc_message.chan14_raw = (_rc.channel_count > 13) ? _rc.values[13] : UINT16_MAX;
	rc_message.chan15_raw = (_rc.channel_count > 14) ? _rc.values[14] : UINT16_MAX;
	rc_message.chan16_raw = (_rc.channel_count > 15) ? _rc.values[15] : UINT16_MAX;
	rc_message.chan17_raw = (_rc.channel_count > 16) ? _rc.values[16] : UINT16_MAX;
	rc_message.chan18_raw = (_rc.channel_count > 17) ? _rc.values[17] : UINT16_MAX;
	rc_message.rssi = _rc.rssi;
	*/
	const uint8_t msgid = MAVLINK_MSG_ID_RC_CHANNELS;
	uint8_t component_ID = 0;
	uint8_t payload_len = mavlink_message_lengths[msgid];
	unsigned packet_len = payload_len + MAVLINK_NUM_NON_PAYLOAD_BYTES;

	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	/* header */
	buf[0] = MAVLINK_STX;
	buf[1] = payload_len;
	/* no idea which numbers should be here*/
	buf[2] = 100;
	buf[3] = 0;
	buf[4] = component_ID;
	buf[5] = msgid;

	/* payload */
	memcpy(&buf[MAVLINK_NUM_HEADER_BYTES], (const void *)&rc_message, payload_len);

	/* checksum */
	uint16_t checksum;
	crc_init(&checksum);
	crc_accumulate_buffer(&checksum, (const char *) &buf[1], MAVLINK_CORE_HEADER_LEN + payload_len);
	crc_accumulate(mavlink_message_crcs[msgid], &checksum);

	buf[MAVLINK_NUM_HEADER_BYTES + payload_len] = (uint8_t)(checksum & 0xFF);
	buf[MAVLINK_NUM_HEADER_BYTES + payload_len + 1] = (uint8_t)(checksum >> 8);

	int ret = send_udp(&buf[0], packet_len);

	if (ret < 1) {
		PX4_WARN("Failed sending rc mavlink message");
	}
}

void init_udp()
{

	PX4_DEBUG("Setting up UDP with port %d", _network_port);

	_network_addr.sin_family = AF_INET;
	_network_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	_network_addr.sin_port = htons(_network_port);

	if ((_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		PX4_WARN("create socket failed: %s", strerror(errno));
		return;
	}

	if (bind(_socket_fd, (struct sockaddr *)&_network_addr, sizeof(_network_addr)) < 0) {
		PX4_WARN("bind failed: %s", strerror(errno));
		return;
	}

}

void deinit_udp()
{
	close(_socket_fd);
}

int send_udp(uint8_t *buf, unsigned len)
{
	int ret = -1;
 
 	// Lock
 	pthread_mutex_lock(&_send_mutex);
 
 	// Write packet via udp link
 	if(_socket_fd > 0){
 
 		ret = sendto(_socket_fd, buf, len, 0,
			     (struct sockaddr *)&_remote_addr, sizeof(_remote_addr));
 	}
 
 	// Unlock
 	pthread_mutex_unlock(&_send_mutex);
 
 	return ret;
}

// skycontroller_interface main entrance
void task_main_trampoline(int argc, char *argv[])
{
	task_main(argc, argv);
}

void start()
{
	ASSERT(_task_handle == -1);

	/* start the task */
	_task_handle = px4_task_spawn_cmd("skycontroller_interface_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX,
					  2000,
					  (px4_main_t)&task_main_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		warn("task start failed");
		return;
	}

	_is_running = true;
}

void stop()
{
	// TODO - set thread exit signal to terminate the task main thread

	_task_should_exit = true;

	_is_running = false;
	_task_handle = -1;
}

void usage()
{
	PX4_WARN("missing command: try 'start', 'stop', 'status'");
	PX4_WARN("options:");
	PX4_WARN("    -i ip");
	PX4_WARN("    -p port");
}


}

extern "C" __EXPORT int skycontroller_interface_main(int argc, char *argv[]);

int skycontroller_interface_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = NULL;
	char *eptr;
	int temp_int_arg;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			temp_int_arg = strtoul(myoptarg, &eptr, 10);

			if (*eptr == '\0') {
				skycontroller_interface::_remote_port = temp_int_arg;

			} else {
				warnx("invalid UDP port '%s'", myoptarg);
				return 1;
				//err_flag = true;
			}

			break;

		case 'i':
			skycontroller_interface::_remote_addr.sin_family = AF_INET;

			if (inet_aton(myoptarg, &skycontroller_interface::_remote_addr.sin_addr)) {

			} else {
				warnx("invalid remote address '%s'", myoptarg);
				return 1;
				//err_flag = true;
			}

			break;

		default:
			skycontroller_interface::usage();
			return 1;
		}
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
/*
		// Check on required arguments
		if (device == NULL || strlen(device) == 0) {
			skycontroller_interface::usage();
			return 1;
		}

		memset(skycontroller_interface::_device, 0, MAX_LEN_DEV_PATH);
		strncpy(skycontroller_interface::_device, device, strlen(device));

*/
		if (skycontroller_interface::_is_running) {
			PX4_WARN("skycontroller_interface already running");
			return 1;
		}

		skycontroller_interface::start();
	}

	else if (!strcmp(verb, "stop")) {
		if (!skycontroller_interface::_is_running) {
			PX4_WARN("skycontroller_interface is not running");
			return 1;
		}

		skycontroller_interface::stop();
	}

	else if (!strcmp(verb, "status")) {
		PX4_WARN("skycontroller_interface is %s", skycontroller_interface::_is_running ? "running" : "stopped");
		return 0;

	} else {
		skycontroller_interface::usage();
		return 1;
	}

	return 0;
}
