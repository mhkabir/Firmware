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


#include "vesc_common.h"

#include <fcntl.h>
#include <termios.h>

#include <systemlib/px4_macros.h>
#include <px4_config.h>
#include <px4_defines.h>

namespace vesc_common
{
static uint16_t crc16(unsigned char *buf, uint8_t len);

int initialise_uart(const char *const device, int &uart_fd)
{
	// open uart
	uart_fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	int termios_state = -1;

	if (uart_fd < 0) {
		PX4_ERR("failed to open uart device!");
		return -1;
	}

	// set baud rate
	int speed = B115200;
	struct termios uart_config;
	tcgetattr(uart_fd, &uart_config);

	// clear ONLCR flag (which appends a CR for every LF)
	uart_config.c_oflag &= ~ONLCR;

	// set baud rate
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("failed to set baudrate for %s: %d\n", device, termios_state);
		close(uart_fd);
		return -1;
	}

	if ((termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("tcsetattr failed for %s\n", device);
		close(uart_fd);
		return -1;
	}

	// setup output flow control
	if (enable_flow_control(uart_fd, false)) {
		PX4_WARN("hardware flow disable failed");
	}

	return 0;
}

int deinitialise_uart(int &uart_fd)
{
	int ret = close(uart_fd);

	if (ret == 0) {
		uart_fd = -1;
	}

	return ret;
}

int enable_flow_control(int uart_fd, bool enabled)
{
	struct termios uart_config;

	int ret = tcgetattr(uart_fd, &uart_config);

	if (ret != 0) {
		PX4_ERR("error getting uart configuration");
		return ret;
	}

	if (enabled) {
		uart_config.c_cflag |= CRTSCTS;

	} else {
		uart_config.c_cflag &= ~CRTSCTS;
	}

	return tcsetattr(uart_fd, TCSANOW, &uart_config);
}

int send_payload(int uart_fd, uint8_t *payload, uint32_t len)
{
	uint8_t packet_len = 0; //TODO for larger messages
	uint8_t packet[256];

	if (len <= 256) {
		packet[packet_len++] = 2;		// Small packet type
		packet[packet_len++] = len;		// Payload length

	} else {
		// TODO : this doesn't handle larger packets yet
		PX4_ERR("Message too large!");
	}

	memcpy(&packet[packet_len], payload, len);

	packet_len += len;

	// Checksum
	uint16_t crc_payload = crc16(payload, len);
	packet[packet_len++] = (uint8_t)(crc_payload >> 8);
	packet[packet_len++] = (uint8_t)(crc_payload & 0xFF);

	packet[packet_len++] = 3; 			// End byte
	packet[packet_len] = NULL;

	int ret = ::write(uart_fd, &packet[0], packet_len);

	if (ret != packet_len) {
		PX4_WARN("TX ERROR: ret: %d, errno: %d", ret, errno);
	}

	return ret;
}

/*
int read_data_from_uart(int uart_fd, ESC_UART_BUF *const uart_buf)
{
	uint8_t tmp_serial_buf[UART_BUFFER_SIZE];

	int len =::read(uart_fd, tmp_serial_buf, arraySize(tmp_serial_buf));

	if (len > 0 && (uart_buf->dat_cnt + len < UART_BUFFER_SIZE)) {
		for (int i = 0; i < len; i++) {
			uart_buf->esc_feedback_buf[uart_buf->tail++] = tmp_serial_buf[i];
			uart_buf->dat_cnt++;

			if (uart_buf->tail >= UART_BUFFER_SIZE) {
				uart_buf->tail = 0;
			}
		}

	} else if (len < 0) {
		return len;
	}

	return 0;
}

int parse_tap_esc_feedback(ESC_UART_BUF *const serial_buf, EscPacket *const packetdata)
{
	static PARSR_ESC_STATE state = HEAD;
	static uint8_t data_index = 0;
	static uint8_t crc_data_cal;

	if (serial_buf->dat_cnt > 0) {
		int packet_len = serial_buf->dat_cnt;

		for (int i = 0; i < packet_len; i++) {
			switch (state) {
			case HEAD:
				if (serial_buf->esc_feedback_buf[serial_buf->head] == PACKET_HEAD) {
					packetdata->head = PACKET_HEAD; //just_keep the format
					state = LEN;
				}

				break;

			case LEN:
				if (serial_buf->esc_feedback_buf[serial_buf->head] < sizeof(packetdata->d)) {
					packetdata->len = serial_buf->esc_feedback_buf[serial_buf->head];
					state = ID;

				} else {
					state = HEAD;
				}

				break;

			case ID:
				if (serial_buf->esc_feedback_buf[serial_buf->head] < ESCBUS_MSG_ID_MAX_NUM) {
					packetdata->msg_id = serial_buf->esc_feedback_buf[serial_buf->head];
					data_index = 0;
					state = DATA;

				} else {
					state = HEAD;
				}

				break;

			case DATA:
				packetdata->d.bytes[data_index++] = serial_buf->esc_feedback_buf[serial_buf->head];

				if (data_index >= packetdata->len) {

					crc_data_cal = crc8_esc((uint8_t *)(&packetdata->len), packetdata->len + 2);
					state = CRC;
				}

				break;

			case CRC:
				if (crc_data_cal == serial_buf->esc_feedback_buf[serial_buf->head]) {
					packetdata->crc_data = serial_buf->esc_feedback_buf[serial_buf->head];

					if (++serial_buf->head >= UART_BUFFER_SIZE) {
						serial_buf->head = 0;
					}

					serial_buf->dat_cnt--;
					state = HEAD;
					return 0;
				}

				state = HEAD;
				break;

			default:
				state = HEAD;
				break;

			}

			if (++serial_buf->head >= UART_BUFFER_SIZE) {
				serial_buf->head = 0;
			}

			serial_buf->dat_cnt--;
		}
	}

	return -1;
}
*/

static uint16_t crc16(unsigned char *buf, uint8_t len)
{
	uint16_t checksum = 0;

	for (uint8_t i = 0; i < len; i++) {
		checksum = crc_table[(((checksum >> 8) ^ *buf++) & 0xFF)] ^ (checksum << 8);
	}

	return checksum;
}

// CRC Table
const uint16_t crc_table[256] = { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
				  0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
				  0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
				  0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
				  0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
				  0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
				  0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
				  0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
				  0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
				  0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
				  0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
				  0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
				  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
				  0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
				  0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
				  0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
				  0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
				  0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
				  0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
				  0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
				  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
				  0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
				  0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
				  0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
				  0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
				  0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
				  0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
				  0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
				  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
				};

// Buffer functions
void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index)
{
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index)
{
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

} /* vesc_common */
