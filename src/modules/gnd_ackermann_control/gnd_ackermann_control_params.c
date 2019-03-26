/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * Maximum lateral velocity that can be commanded (magnitude)
 *
 * @unit m/s
 * @min 0.0
 * @max 30.0
 * @group Ground Rover Ackermann Control
 */
PARAM_DEFINE_FLOAT(RAC_VEL_MAX, 10.0f);

/**
 * Velocity controller proportional gain
 *
 * This defines how much acceleration will be commanded depending on the
 * current velocity error.
 *
 * @unit (m/s^2)/(m/s)
 * @min 0.005
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Ackermann Control
 */
PARAM_DEFINE_FLOAT(RAC_VEL_P, 1.0f);

/**
 * Maximum steering rate that can be commanded (magnitude)
 *
 * @unit rad/s
 * @min 0.0
 * @max 1.0
 * @group Ground Rover Ackermann Control
 */
PARAM_DEFINE_FLOAT(RAC_SR_MAX, 1.0f);

/**
 * Steering rate controller proportional gain
 *
 * @unit rad/s
 * @min 0.0
 * @max 1.0
 * @group Ground Rover Ackermann Control
 */
PARAM_DEFINE_FLOAT(RAC_SR_P, 0.1f);

/**
 * Steering rate controller integral gain
 *
 * @unit rad/s
 * @min 0.0
 * @max 1.0
 * @group Ground Rover Ackermann Control
 */
PARAM_DEFINE_FLOAT(RAC_SR_I, 1.0f);

/**
 * Steering rate controller derivative gain
 *
 * @unit rad/s
 * @min 0.0
 * @max 1.0
 * @group Ground Rover Ackermann Control
 */
PARAM_DEFINE_FLOAT(RAC_SR_D, 0.01f);

/**
 * Steering feedforward controller wheelbase model parameter
 *
 * @unit m
 * @min 0.0
 * @max 1.0
 * @group Rover Ackermann Control
 */
PARAM_DEFINE_FLOAT(RAC_MDL_WB, 0.485f);

/**
 * Steering feedforward controller steering model parameter
 *
 * @unit %/rad
 * @min 0.0
 * @max 10.0
 * @group Rover Ackermann Control
 */
PARAM_DEFINE_FLOAT(RAC_MDL_SK, 2.2f);

/**
 * Velocity feedforward controller wheel radius model parameter
 *
 * @unit m
 * @min 0.0
 * @max 1.0
 * @group Rover Ackermann Control
 */
PARAM_DEFINE_FLOAT(RAC_MDL_WR, 0.1015f);

/**
 * Velocity feedforward controller wheel to motor RPM model parameter
 *
 * @min 0.0
 * @max 1000.0
 * @group Rover Ackermann Control
 */
PARAM_DEFINE_FLOAT(RAC_MDL_WMR, 23.75f);

/**
 * Whether to scale control effort by battery voltage level
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The vehicle
 * should constantly behave as if it was fully charged with reduced max torque
 * at lower battery percentages.
 *
 * @boolean
 * @group Rover Ackermann Control
 */
PARAM_DEFINE_INT32(RAC_BAT_SCALE, 1);
