/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *   Author: Mohammed Kabir <mhkabir98@gmail.com>
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
 * @file camera_trigger.cpp
 *
 * External camera-IMU synchronisation and triggering via FMU auxillary _pins.
 *
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <uORB/uORB.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_command.h>
#include <poll.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <mavlink/mavlink_log.h>

class CameraTrigger
{
public:
	/**
	 * Constructor
	 */
	CameraTrigger();

	/**
	 * Destructor, also kills task.
	 */
	~CameraTrigger();

	/**
	 * Start the task.
	 */
	void		start();

	/**
	 * Stop the task.
	 */
	void		stop();

	/**
	 * Display info.
	 */
	void		info();
		
	int 		_pin;
	
private:
	
	struct hrt_call		_pollcall;
	struct hrt_call		_firecall;
	
	int 			_gpio_fd0, _gpio_fd1, _gpio_fd2;

	int 			_polarity;
	float 			_activation_time;
	float  			_integration_time;
	float  			_transfer_time;
	uint32_t 		_trigger_seq;		
	bool	 		_trigger_enabled;

	int			_sensor_sub;
	int			_vcommand_sub;

	orb_advert_t		_trigger_pub;

	struct camera_trigger_s		_trigger;
	struct sensor_combined_s	_sensor;
	struct vehicle_command_s	_command;

	param_t polarity ;
	param_t activation_time ;	
	param_t integration_time ;
	param_t transfer_time ;
	
	/**
	 * Topic poller to check for fire info.
	 */
	static void	poll(void *arg);
	/**
	 * Fires trigger
	 */
	static void	engage(void *arg);
	/**
	 * Resets trigger
	 */
	static void	disengage(void *arg);

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int camera_trigger_main(int argc, char *argv[]);

namespace camera_trigger
{

CameraTrigger	*g_camera_trigger;
}

CameraTrigger::CameraTrigger() :
	_pin(1),
	_pollcall{},
	_firecall{},
	_gpio_fd0(-1),
	_gpio_fd1(-1),
	_gpio_fd2(-1),
	_polarity(0),
	_activation_time(0.0f),
	_integration_time(0.0f),
	_transfer_time(0.0f),
	_trigger_seq(0),
	_trigger_enabled(true),
	_sensor_sub(-1),
	_vcommand_sub(-1),
	_trigger_pub(nullptr),
	_trigger{},
	_sensor{},
	_command{}
{
	memset(&_trigger, 0, sizeof(_trigger));
	memset(&_sensor, 0, sizeof(_sensor));
	memset(&_command, 0, sizeof(_command));
	
	memset(&_pollcall, 0, sizeof(_pollcall));
	memset(&_firecall, 0, sizeof(_firecall));	

	/* Parameters */
	polarity = param_find("TRIG_POLARITY");
	activation_time = param_find("TRIG_ACT_TIME");	
	integration_time = param_find("TRIG_INT_TIME");
	transfer_time = param_find("TRIG_TRANS_TIME");
}

CameraTrigger::~CameraTrigger()
{
	camera_trigger::g_camera_trigger = nullptr;
}

void
CameraTrigger::start()
{

	_gpio_fd0 = open(PX4FMU_DEVICE_PATH, 0);

	if (_gpio_fd0 < 0) {
		warnx("GPIO device open fail");
		stop();
	}
	else
	{
		warnx("GPIO device opened");
	}

	_sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	_vcommand_sub = orb_subscribe(ORB_ID(vehicle_command));
	
	param_get(polarity, &_polarity); 
	param_get(activation_time, &_activation_time);	
	param_get(integration_time, &_integration_time); 	
	param_get(transfer_time, &_transfer_time); 

	px4_ioctl(_gpio_fd0, GPIO_SET_OUTPUT, _pin);
	
	if(_polarity == 0)
	{
		px4_ioctl(_gpio_fd0, GPIO_SET, _pin); 	/* GPIO _pin pull high */
	}
	else if(_polarity == 1)
	{
		px4_ioctl(_gpio_fd0, GPIO_CLEAR, _pin); 	/* GPIO _pin pull low */
	}	
	else
	{
		warnx(" invalid trigger polarity setting. stop_ping.");
		stop();
	}
	close(_gpio_fd0);

	poll(this);	/* Trampoline call */

}

void
CameraTrigger::stop()
{	
	hrt_cancel(&_firecall);
	hrt_cancel(&_pollcall);

	if (camera_trigger::g_camera_trigger != nullptr) {
            delete (camera_trigger::g_camera_trigger);
	}
	warnx("Camera Trigger routine stopped");
}

void
CameraTrigger::poll(void *arg)
{

	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	bool updated;
	orb_check(trig->_vcommand_sub, &updated);

	if(trig->_gpio_fd == -1)
		return;
	
	if (updated) {
		
		orb_copy(ORB_ID(vehicle_command), trig->_vcommand_sub, &trig->_command);
		
		if(trig->_command.command == vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL)
		{
			if(trig->_command.param1 < 1)
			{
				if(trig->_trigger_enabled)
				{
					trig->_trigger_enabled = false ; 
				}
			}
			else if(trig->_command.param1 >= 1)
			{
				if(!trig->_trigger_enabled)
				{
					trig->_trigger_enabled = true ;
				} 
			}

			// Set trigger rate from command
			if(trig->_command.param2 > 0)
			{
				trig->_integration_time = trig->_command.param2;
				param_set(trig->integration_time, &(trig->_integration_time));
			}		
		}
	}

	if(!trig->_trigger_enabled)	{	
		hrt_call_after(&trig->_pollcall, 1000, (hrt_callout)&CameraTrigger::poll, trig); 
		return;
	}
	else
	{	
		engage(trig);
		hrt_call_after(&trig->_firecall, trig->_activation_time*1000, (hrt_callout)&CameraTrigger::disengage, trig);		
		
		orb_copy(ORB_ID(sensor_combined), trig->_sensor_sub, &trig->_sensor);
					
		trig->_trigger.timestamp = trig->_sensor.timestamp;	/* get IMU timestamp */
		trig->_trigger.seq = trig->_trigger_seq++;

		if (trig->_trigger_pub != nullptr) {
			orb_publish(ORB_ID(camera_trigger), trig->_trigger_pub, &trig->_trigger);
		} else {
			trig->_trigger_pub = orb_advertise(ORB_ID(camera_trigger), &trig->_trigger);
		}

		hrt_call_after(&trig->_pollcall, (trig->_transfer_time + trig->_integration_time)*1000, (hrt_callout)&CameraTrigger::poll, trig); 
	}
	
}

void
CameraTrigger::engage(void *arg)
{
	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);
	
	trig->_gpio_fd0 = open(PX4FMU_DEVICE_PATH, 0);

	if(trig->_gpio_fd1 == -1)
		return;
	
	if(trig->_polarity == 0)  	// ACTIVE_LOW 
	{
		px4_ioctl(trig->_gpio_fd0, GPIO_CLEAR, trig->_pin);	
	}
	else if(trig->_polarity == 1)	// ACTIVE_HIGH 
	{
		px4_ioctl(trig->_gpio_fd0, GPIO_SET, trig->_pin);		
	}

	close(trig->_gpio_fd0);
	
}

void
CameraTrigger::disengage(void *arg)
{
	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);
	
	trig->_gpio_fd1 = open(PX4FMU_DEVICE_PATH, 0);

	if(trig->_gpio_fd1 == -1)
		return;
	
	if(trig->_polarity == 0)  	// ACTIVE_LOW 
	{
		px4_ioctl(trig->_gpio_fd1, GPIO_SET, trig->_pin);	
	}
	else if(trig->_polarity == 1)	// ACTIVE_HIGH 
	{
		px4_ioctl(trig->_gpio_fd1, GPIO_CLEAR, trig->_pin);		
	}

	close(trig->_gpio_fd1);
}

void
CameraTrigger::info()
{
	warnx("Trigger state : %s", _trigger_enabled ? "enabled" : "disabled");
	warnx("Trigger _pin : %i", _pin);
	warnx("Trigger polarity : %s", _polarity ? "ACTIVE_HIGH" : "ACTIVE_LOW");
	warnx("Shutter integration time : %.2f", (double)_integration_time);
}

static void usage()
{
	errx(1, "usage: camera_trigger {start|stop|info} [-p <n>]\n"
		     "\t-p <n>\tUse specified AUX OUT _pin number (default: 1)"
		    );
}

int camera_trigger_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (camera_trigger::g_camera_trigger != nullptr) {
			errx(0, "already running");
		}
			
		camera_trigger::g_camera_trigger = new CameraTrigger;

		if (camera_trigger::g_camera_trigger == nullptr) {
			errx(1, "alloc failed");
		}
		
		if (argc > 3) {
	
			camera_trigger::g_camera_trigger->_pin = (int)argv[3];
			if (atoi(argv[3]) > 0 && atoi(argv[3]) < 6) {	
				warnx("starting trigger on _pin : %li ", atoi(argv[3]));	
				camera_trigger::g_camera_trigger->_pin = atoi(argv[3]);
			}
			else
			{
				usage(); 
			}
		}
		camera_trigger::g_camera_trigger->start();

		return 0;
	}

	if (camera_trigger::g_camera_trigger == nullptr) {
		errx(1, "not running");
	}

	else if (!strcmp(argv[1], "stop")) {
		camera_trigger::g_camera_trigger->stop(); 

	}
	else if (!strcmp(argv[1], "info")) {
		camera_trigger::g_camera_trigger->info(); 

	} else {
		usage();
	}

	return 0;
}

