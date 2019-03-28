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

#pragma once

#include <stdint.h>

#include "drv_vesc.h"

namespace vesc_common
{

/**
 *  Opens a device for use as UART.
 *  @param device UNIX path of UART device
 *  @param uart_fd file-descriptor of UART device
 *  @return 0 on success, -1 on error
 */
int initialise_uart(const char *const device, int &uart_fd);

/**
 *  Closes a device previously opened with initialise_uart().
 *  @param uart_fd file-descriptor of UART device as provided by initialise_uart()
 *  @return 0 on success, -1 on error
 */
int deinitialise_uart(int &uart_fd);

/**
 *  Enables/disables flow control for open UART device.
 *  @param uart_fd file-descriptor of UART device
 *  @param enabled Set true for enabling and false for disabling flow control
 *  @return 0 on success, -1 on error
 */
int enable_flow_control(int uart_fd, bool enabled);

/**
 *  Sends a packet to the ESC
 *  @param uart_fd file-descriptor of UART device
 *  @param payload Payload to be sent to ESCs. CRC information will be added.
 *  @param len Length of payload
 *  @return On success number of bytes written, on error -1
 */
int send_payload(int uart_fd, uint8_t *payload, uint32_t len);

/**
 *  Read data from the UART into a buffer
 *  @param uart_fd file-descriptor of UART device
 *  @param uart_buffer Buffer where incoming data will be stored
 *  @return 0 on success, -1 on error
 */
int read_uart(int uart_fd, UARTBuffer *const uart_buffer);

/**
 *  Parse packets from ESC
 *  @param uart_buffer Buffer where incoming data will be stored
 *  @param payload
 *  @param payload_len
 *  @return On success length of payload, on error -1
 */
int parse_packet(UARTBuffer *const uart_buffer, uint8_t *payload);

/**
 *  Parse status message payload
 *  @param payload
 *  @param payload_len
 *  @param feedback
 *  @return 0 on success, -1 on error
 */
int unpack_payload(uint8_t *payload, uint8_t payload_len, float &erpm);

/**
 *  Lookup-table for faster CRC computation when sending ESC packets.
 */
extern const uint16_t crc_table[];

int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index);
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index);
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index);
uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index);
float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index);
float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index);

void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index);
void buffer_append_uint16(uint8_t *buffer, uint16_t number, int32_t *index);
void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index);
void buffer_append_uint32(uint8_t *buffer, uint32_t number, int32_t *index);
void buffer_append_float16(uint8_t *buffer, float number, float scale, int32_t *index);
void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index);

} /* vesc_common */