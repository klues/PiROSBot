/*
 * Motor Driver for 2 DC motors
 * It uses the PIGPIO Daemon as GPIO-Interface (default socket)
 * 
 * 
 * Copyright © 2017, Thomas Jerabek, UAS Technikum-Wien.
 * This work is free. You can redistribute it and/or modify it under the
 * terms of the Do What The Fuck You Want To Public License, Version 2,
 * as published by Sam Hocevar. See the COPYING file or http://www.wtfpl.net/ 
 * for more details.
 */

#include "rpi_driverd_mot.h"
#include <unistd.h>

/**
 * constructor
 */
RPI_Driverd_Mot::RPI_Driverd_Mot()
{
	initFlag = init();
	if(!initFlag)
	{
		quit();
	}

}

/**
 * destructor
 */
RPI_Driverd_Mot::~RPI_Driverd_Mot()
{
	quit();
}

bool RPI_Driverd_Mot::init(void)
{
	bool ret = false;
	startTimer = false;
	piID = pigpio_start(NULL,NULL);
	if (piID>= 0)	// wait for successed PiGPIO init
	{
		if(IRRC_ENABLE) // enable IR input pin with pull-up
		{
			set_mode(piID,IRRC_IN,PI_INPUT);
			set_pull_up_down(piID,IRRC_IN, PI_PUD_UP);
		}

		// output pins for motor control (for forward, reverse, left, right and stop);
		set_mode(piID,MotA1_OUT,PI_OUTPUT);
		set_mode(piID,MotA2_OUT,PI_OUTPUT);
		set_mode(piID,MotB1_OUT,PI_OUTPUT);
		set_mode(piID,MotB2_OUT,PI_OUTPUT);

		// set pwm outpus
		set_PWM_frequency(piID,PWMA_OUT, PWM_FREQ);
		set_PWM_frequency(piID,PWMB_OUT, PWM_FREQ);
		set_PWM_range(piID,PWMA_OUT, PWM_RANGE);
		set_PWM_range(piID,PWMB_OUT, PWM_RANGE);
		set_PWM_dutycycle(piID,PWMA_OUT,PWM_INIT);
		set_PWM_dutycycle(piID,PWMB_OUT,PWM_INIT);

		stop(); // initially set motors to stop state!
		dc_state = DC_STOP;
		dc_speed = PWM_INIT;
		dc_duration = STOP_TIMER_INIT;

		// timer thread
		pthKeepAlive = true;

		_pth = NULL;

		_pth = start_thread(dc_stop_thread,this);
		if(_pth != NULL)
		{
			ret = true;
		}
		else
		{
			pigpio_stop(piID);
		}
	}
	return ret;
}

void RPI_Driverd_Mot::set_motor(bool A1, bool A2, bool B1, bool B2)
{
	gpio_write(piID,MotA1_OUT,A1);
	gpio_write(piID,MotA2_OUT,A2);
	gpio_write(piID,MotB1_OUT,B1);
	gpio_write(piID,MotB2_OUT,B2);
}

void RPI_Driverd_Mot::forward(void)
{
	set_motor(true, false, true, false);
	setDCState(DC_FORWARD);
	startTimer=true;
}

void RPI_Driverd_Mot::stop(void)
{
	set_motor(false, false, false, false);
	setDCState(DC_STOP);
}


void RPI_Driverd_Mot::reverse(void)
{
	set_motor(false, true, false, true);
	setDCState(DC_BACKWARD);
	startTimer=true;
}

void RPI_Driverd_Mot::left(void)
{
	set_motor(true, false, false, false);
	setDCState(DC_LEFT);
	startTimer=true;
}

void RPI_Driverd_Mot::right(void)
{
	set_motor(false, false, true, false);
	setDCState(DC_RIGHT);
	startTimer=true;
}


void* RPI_Driverd_Mot::dc_stop_thread(void *user)
{
	RPI_Driverd_Mot *mySelf = (RPI_Driverd_Mot *) user;
	bool timer_flag = false;
	uint32_t startTimestamp = 0;
	uint32_t stopTimestamp = 0;

	while(mySelf->pthKeepAlive)
	{

		if(mySelf->startTimer) // new timer configuration
		{
			
			timer_flag = true;
			mySelf->startTimer = false;
			startTimestamp = get_current_tick(mySelf->piID);
			stopTimestamp = startTimestamp + (mySelf->dc_duration*1000); // TODO: handle integer overflow / wrap
		}
		
		if(timer_flag)	// run timer 
		{
			if(get_current_tick(mySelf->piID) >= stopTimestamp) // TODO: handle integer overflow / wrap
			{
				mySelf->stop();
				mySelf->setDCState(DC_STOP);
				timer_flag = false;
			}

		}
		usleep(10000); // sleep for 10 ms - fine enough
	}
	return NULL;
}

uint32_t RPI_Driverd_Mot::getDCDuration() const {
	return dc_duration;
}

void RPI_Driverd_Mot::setDCDuration(uint32_t dc_duration) {
	this->dc_duration = dc_duration;
}

uint8_t RPI_Driverd_Mot::getDCSpeed() const {
	return dc_speed;
}

void RPI_Driverd_Mot::setDCSpeed(uint8_t dutycycle) {
	
	if(dutycycle > 100)
	{
		dutycycle = (uint8_t) ((dutycycle*100)/255);
	}
	this->dc_speed = dutycycle;
	set_PWM_dutycycle(piID,PWMA_OUT,dutycycle);
	set_PWM_dutycycle(piID,PWMB_OUT,dutycycle);
}

uint8_t RPI_Driverd_Mot::getDCState() const {
	return dc_state;
}

void RPI_Driverd_Mot::setDCState(uint8_t dc_state) {
	this->dc_state = dc_state;
}

bool RPI_Driverd_Mot::getInitFlag() const {
	return initFlag;
}

void RPI_Driverd_Mot::quit(void)
{
	pigpio_stop(piID); // reset GPIO configuration

	pthKeepAlive = false;	// quit endless loop of pthread
	stop_thread(_pth);		// run stop pthread routine
}



