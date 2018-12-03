/****************************************************************************
 *
 *   Copyright (c) 2014-2018 PX4 Development Team. All rights reserved.
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
 * @file landing_target_ekf_params.c
 * Landing Target EKF algorithm parameters.
 *
 * @author Mohammed Kabir <mhkabir98@gmail.io>
 *
 */

/**
 * Acceleration uncertainty
 *
 * Variance of acceleration measurement used for landing target position prediction.
 * Higher values results in tighter following of the measurements and more lenient outlier rejection
 *
 * @unit (m/s^2)^2
 * @min 0.01
 * @decimal 2
 *
 * @group Landing Target EKF
 */
PARAM_DEFINE_FLOAT(LTEKF_ACC_UNC, 10.0f);

/**
 * Landing target measurement uncertainty
 *
 * Variance of the landing target measurement from the driver.
 * Higher values results in less agressive following of the measurement and a smoother output as well as fewer rejected measurements.
 *
 * @unit tan(rad)^2
 * @decimal 4
 *
 * @group Landing Target EKF
 */
PARAM_DEFINE_FLOAT(LTEKF_MEAS_UNC, 0.005f);

/**
 * Initial landing target position uncertainty
 *
 * Initial variance of the relative landing target position in x and y direction
 *
 * @unit m^2
 * @min 0.001
 * @decimal 3
 *
 * @group Landing Target EKF
 */
PARAM_DEFINE_FLOAT(LTEKF_POS_UNC_IN, 0.1f);

/**
 * Initial landing target velocity uncertainty
 *
 * Initial variance of the relative landing target velocity in x and y direction
 *
 * @unit (m/s)^2
 * @min 0.001
 * @decimal 3
 *
 * @group Landing Target EKF
 */
PARAM_DEFINE_FLOAT(LTEKF_VEL_UNC_IN, 0.1f);