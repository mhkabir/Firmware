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
 * @file vesc_params.c
 * VESC parameters.
 *
 * @author Mohammed Kabir
 */

/**
 * Maximum current to deliver to the motor
 *
 * @group VESC
 * @min 0.0
 * @max 100.0
 * @unit A
 */
PARAM_DEFINE_FLOAT(VESC_MAX_CURRENT, 10.0f);

/**
 * Maximum braking current to deliver to the motor
 *
 * @group VESC
 * @min 0.0
 * @max 100.0
 * @unit A
 */
PARAM_DEFINE_FLOAT(VESC_BRK_CURRENT, 10.0f);

/**
 * Normalized angle setpoint for which servo is centered
 *
 * @group VESC
 * @min 0.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(VESC_ANGLE_TRIM, 0.5f);

/**
 * Normalized angle setpoint for which servo is at left extreme
 *
 * @group VESC
 * @min 0.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(VESC_ANGLE_MIN, 0.0f);

/**
 * Normalized angle setpoint for which servo is at right extreme
 *
 * @group VESC
 * @min 0.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(VESC_ANGLE_MAX, 1.0f);