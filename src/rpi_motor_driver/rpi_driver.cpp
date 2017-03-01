/*
 * Driver for 2 DC motors and camera pan & tilt (2 servomotors)
 * It uses the PIGPIO Daemon as GPIO-Interface (default socket)
 * 
 * 
 * Copyright © 2017, Thomas Jerabek, UAS Technikum-Wien.
 * This work is free. You can redistribute it and/or modify it under the
 * terms of the Do What The Fuck You Want To Public License, Version 2,
 * as published by Sam Hocevar. See the COPYING file or http://www.wtfpl.net/ 
 * for more details.
 */

#include "rpi_driver.h"


/**
 * constructor
 */
RPI_Driver::RPI_Driver()
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
RPI_Driver::~RPI_Driver()
{
	quit();
}

bool RPI_Driver::init(void)
{
	bool ret = false;
	if (gpioInitialise() >= 0)	// wait for successed PiGPIO init
	{
		if(IRRC_ENABLE) // enable IR input pin with pull-up
		{
			gpioSetMode(IRRC_IN,PI_INPUT);
			gpioSetPullUpDown(IRRC_IN, PI_PUD_UP);
		}

		// output pins for motor control (for forward, reverse, left, right and stop);
		gpioSetMode(MotA1_OUT,PI_OUTPUT);
		gpioSetMode(MotA2_OUT,PI_OUTPUT);
		gpioSetMode(MotB1_OUT,PI_OUTPUT);
		gpioSetMode(MotB2_OUT,PI_OUTPUT);

		// set pwm outpus
		gpioSetPWMfrequency(PWMA_OUT, PWM_FREQ);
		gpioSetPWMfrequency(PWMB_OUT, PWM_FREQ);
		gpioSetPWMrange(PWMA_OUT, PWM_RANGE);
		gpioSetPWMrange(PWMB_OUT, PWM_RANGE);
		gpioPWM(PWMA_OUT,PWM_INIT);
		gpioPWM(PWMB_OUT,PWM_INIT);

		stop(); // initially set motors to stop state!
		dc_state = DC_STOP;
		dc_speed = PWM_INIT;
		dc_duration = STOP_TIMER_INIT;

		// camera pan & tilt
		gpioSetPWMfrequency(CAM_SERVO1_PWM_OUT, CAM_SERVO_PWM_FREQ);
		gpioSetPWMfrequency(CAM_SERVO2_PWM_OUT, CAM_SERVO_PWM_FREQ);
		gpioSetPWMrange(CAM_SERVO1_PWM_OUT, CAM_SERVO1_PWM_RANGE);
		gpioSetPWMrange(CAM_SERVO2_PWM_OUT, CAM_SERVO1_PWM_RANGE);
		gpioPWM(CAM_SERVO1_PWM_OUT, CAM_SERVO1_PWM_INIT);
		gpioPWM(CAM_SERVO2_PWM_OUT, CAM_SERVO2_PWM_INIT);
		posServo1 = CAM_SERVO1_PWM_INIT;
		posServo2 = CAM_SERVO2_PWM_INIT;
		ret = true;
	}

	return ret;
}

void RPI_Driver::set_motor(bool A1, bool A2, bool B1, bool B2)
{
	gpioWrite(MotA1_OUT,A1);
	gpioWrite(MotA2_OUT,A2);
	gpioWrite(MotB1_OUT,B1);
	gpioWrite(MotB2_OUT,B2);
}

void RPI_Driver::forward(void)
{
	set_motor(true, false, true, false);
	setDCState(DC_FORWARD);
	gpioSetTimerFunc(STOP_TIMER,this->dc_duration,NULL);
	gpioSetTimerFuncEx(STOP_TIMER,this->dc_duration,dc_stop_callback, this);
}

void RPI_Driver::stop(void)
{
	set_motor(false, false, false, false);
	setDCState(DC_STOP);
}


void RPI_Driver::reverse(void)
{
	set_motor(false, true, false, true);
	setDCState(DC_BACKWARD);
	gpioSetTimerFunc(STOP_TIMER,this->dc_duration,NULL);
	gpioSetTimerFuncEx(STOP_TIMER,this->dc_duration,dc_stop_callback, this);
}

void RPI_Driver::left(void)
{
	set_motor(true, false, false, false);
	setDCState(DC_LEFT);
	gpioSetTimerFunc(STOP_TIMER,this->dc_duration,NULL);
	gpioSetTimerFuncEx(STOP_TIMER,this->dc_duration,dc_stop_callback, this);
}

void RPI_Driver::right(void)
{
	set_motor(false, false, true, false);
	setDCState(DC_RIGHT);
	gpioSetTimerFunc(STOP_TIMER,this->dc_duration,NULL);
	gpioSetTimerFuncEx(STOP_TIMER,this->dc_duration,dc_stop_callback, this);
}

void RPI_Driver::dc_stop_callback(void *user)
{
	RPI_Driver *mySelf = (RPI_Driver *) user;
	mySelf->stop();
	mySelf->setDCState(DC_STOP);
}

uint32_t RPI_Driver::getDCDuration() const {
	return dc_duration;
}

void RPI_Driver::setDCDuration(uint32_t dc_duration) {
	this->dc_duration = dc_duration;
}

uint8_t RPI_Driver::getDCSpeed() const {
	return dc_speed;
}

void RPI_Driver::setDCSpeed(uint8_t dutycycle) {
	
	if(dutycycle > 100)
	{
		dutycycle = (uint8_t) ((dutycycle*100)/255);
	}
	this->dc_speed = dutycycle;
	gpioPWM(PWMA_OUT,dutycycle);
	gpioPWM(PWMB_OUT,dutycycle);
}

uint8_t RPI_Driver::getDCState() const {
	return dc_state;
}

void RPI_Driver::setDCState(uint8_t dc_state) {
	this->dc_state = dc_state;
}

bool RPI_Driver::getInitFlag() const {
	return initFlag;
}

void RPI_Driver::quit(void)
{
	gpioTerminate(); // reset GPIO configuration
}


void RPI_Driver::setCamPos(uint16_t pan, uint16_t tilt)
{
	//set limits
	if(pan>2000) { pan = 2000;}
	if(pan<1000) { pan = 1000;}
	if(tilt>2000) { tilt = 2000;}
	if(tilt<1000) { tilt = 1000;}

	gpioPWM(CAM_SERVO1_PWM_OUT, pan);
	gpioPWM(CAM_SERVO2_PWM_OUT, tilt);
}


