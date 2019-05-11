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

	// fetch bytes as they become available
	uart_config.c_cc[VMIN] = 1;
	uart_config.c_cc[VTIME] = 1;

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

int read_uart(int uart_fd, UARTBuffer *const uart_buffer)
{
	// Read raw UART data into a persistent ringbuffer
	uint8_t temp_buffer[UART_BUFFER_SIZE];
	int len = ::read(uart_fd, temp_buffer, arraySize(temp_buffer));

	if (len > 0 && (uart_buffer->length + len < UART_BUFFER_SIZE)) {
		for (int i = 0; i < len; i++) {
			uart_buffer->data[uart_buffer->tail++] = temp_buffer[i];
			uart_buffer->length++;

			if (uart_buffer->tail >= UART_BUFFER_SIZE) {
				uart_buffer->tail = 0;
			}
		}

	} else if (len < 0) {
		return len;
	}

	return 0;

}

int parse_packet(UARTBuffer *const uart_buffer
	, uint8_t *payload)
{
	static ParserState state = HEAD;
	static uint8_t payload_data_len = 0;
	static uint8_t payload_data[256 - MIN_PACKET_SIZE];
	static uint8_t data_idx = 0;
	static uint8_t crc[2];

	static uint16_t crc_message;
	static uint16_t crc_payload;

	int bytes_available = uart_buffer->length;

	if (bytes_available > MIN_PACKET_SIZE) {

		for (int i = 0; i < bytes_available; i++) {
			// Run parser
			switch (state) {
			case HEAD:
				if (uart_buffer->data[uart_buffer->head] == SOF_VAL) {
					state = LEN;
				}

				break;

			case LEN:
				payload_data_len = uart_buffer->data[uart_buffer->head];
				state = PAYLOAD;
				break;

			case PAYLOAD:
				payload_data[data_idx++] = uart_buffer->data[uart_buffer->head];

				if (data_idx == payload_data_len) {
					// We have the full payload, compute CRC
					crc_payload = crc16(payload_data, payload_data_len);
					data_idx = 0;
					state = CRC;
				}

				break;

			case CRC:
				crc[data_idx++] = uart_buffer->data[uart_buffer->head];

				if (data_idx == 2) {
					// We have the full CRC
					crc_message = crc[0] << 8;
					crc_message &= 0xFF00;
					crc_message += crc[1];

					data_idx = 0;
					state = TAIL;
				}

				break;

			case TAIL:

				// Check for end of frame
				if (uart_buffer->data[uart_buffer->head] == EOF_VAL) {

					if (crc_message == crc_payload) {

						// Parsed payload
						memcpy(payload, payload_data, payload_data_len);

						// Handle ringbuffer loopback
						if (++uart_buffer->head >= UART_BUFFER_SIZE) {
							uart_buffer->head = 0;
						}

						uart_buffer->length--;

						state = HEAD;
						return payload_data_len;

					} else {
						PX4_ERR("CRC incorrect");
					}

				} else {
					PX4_ERR("Parser out of sync");
				}
				clear_buffer(uart_buffer);
				payload_data_len = 0;
				data_idx = 0;
				state = HEAD;
				return 0;
				break;
			}

			// Handle ringbuffer loopback
			if (++uart_buffer->head >= UART_BUFFER_SIZE) {
				uart_buffer->head = 0;
			}

			uart_buffer->length--; // TODO : this should never go negative!
		}
	}

	return 0;
}

void clear_buffer(UARTBuffer *const uart_buffer)
{
	uart_buffer->head = 0;
	uart_buffer->tail = 0;
	uart_buffer->length = 0;
	memset(uart_buffer->data, 0, sizeof(uint8_t)*UART_BUFFER_SIZE);
}

int unpack_payload(uint8_t *payload, uint8_t payload_len, float &erpm)
{
	uint8_t packet_id = payload[0];
	int32_t index = 0;

	switch (packet_id) {
	case COMM_GET_VALUES:
		index = 25; // Skip the first 25 bits.
		erpm = buffer_get_float32(payload, 1.0f, &index);
		return 0;
		break;

	default:
		PX4_WARN("Got unexpected packet %d", packet_id);
		break;
	}

	return -1;
}

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

// Reading functions
uint8_t buffer_get_uint8(const uint8_t *buffer, int32_t *index)
{
	uint8_t res = buffer[*index];
	*index += 1;
	return res;
}

// Reading functions
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index)
{
	int16_t res =	((uint16_t) buffer[*index]) << 8 |
			((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index)
{
	uint16_t res =	((uint16_t) buffer[*index]) << 8 |
			((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index)
{
	int32_t res =	((uint32_t) buffer[*index]) << 24 |
			((uint32_t) buffer[*index + 1]) << 16 |
			((uint32_t) buffer[*index + 2]) << 8 |
			((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index)
{
	uint32_t res =	((uint32_t) buffer[*index]) << 24 |
			((uint32_t) buffer[*index + 1]) << 16 |
			((uint32_t) buffer[*index + 2]) << 8 |
			((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}
float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index)
{
	return (float)buffer_get_int16(buffer, index) / scale;
}

float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index)
{
	return (float)buffer_get_int32(buffer, index) / scale;
}

// Addition functions
void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index)
{
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_uint16(uint8_t *buffer, uint16_t number, int32_t *index)
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

void buffer_append_uint32(uint8_t *buffer, uint32_t number, int32_t *index)
{
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_float16(uint8_t *buffer, float number, float scale, int32_t *index)
{
	buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index)
{
	buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

} /* vesc_common */
