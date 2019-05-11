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
 * @author Mohammed Kabir <mhkabir@mit.edu>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_module_params.h>

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <pid/pid.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_ackermann_setpoint.h>
#include <uORB/topics/vehicle_body_state.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/uORB.h>

using matrix::Eulerf;
using matrix::Quatf;

class GroundRoverAckermannControl : public ModuleParams
{
public:
	GroundRoverAckermannControl();
	~GroundRoverAckermannControl();

	int start();
	bool task_running() { return _task_running; }

private:

	bool	_task_should_exit{false};		/**< if true, attitude control task should exit */
	bool	_task_running{false};			/**< if true, task is running in its mainloop */
	int		_control_task{ -1};			/**< task handle */

	int		_state_sub{ -1};			/**< vehicle attitude setpoint */
	int		_battery_status_sub{ -1};		/**< battery status subscription */
	int		_ackermann_sp_sub{ -1};		/**< control state subscription */
	int		_params_sub{ -1};			/**< notification of parameter updates */

	orb_advert_t	_actuators_pub{nullptr};		/**< actuator control group 0 setpoint */
	orb_advert_t	_debug_vect_pub{nullptr};		/**< debug */

	uint64_t _timestamp_last{0};

	battery_status_s				_battery_status {};		/**< battery status */
	vehicle_ackermann_setpoint_s	_ackermann_sp {};		/**< setpoint */

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_nonfinite_input_perf;		/**< performance counter for non finite input */
	perf_counter_t	_nonfinite_output_perf;		/**< performance counter for non finite output */

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RAC_MDL_SK>) _param_mdl_sk,
		(ParamFloat<px4::params::RAC_MDL_WB>) _param_mdl_wb,
		(ParamFloat<px4::params::RAC_SR_P>) _param_sr_p,
		(ParamFloat<px4::params::RAC_SR_I>) _param_sr_i,
		(ParamFloat<px4::params::RAC_SR_D>) _param_sr_d,
		(ParamInt<px4::params::RAC_BAT_SCALE>) _param_bat_scale_en
	);

	//PID_t			_velocity_ctrl{};
	PID_t			_steering_rate_ctrl{};

	void		parameters_update(int parameter_update_sub, bool force);

	void		vehicle_ackermann_setpoint_poll();
	void		battery_status_poll();

	static int	task_main_trampoline(int argc, char *argv[]);
	void		task_main();

};
