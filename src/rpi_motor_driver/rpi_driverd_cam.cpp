/*
 * Camera Driver for Camera pan & tilt (2 servomotors)
 * It uses the PIGPIO Daemon as GPIO-Interface (default socket)
 * 
 * 
 * Copyright © 2017, Thomas Jerabek, UAS Technikum-Wien.
 * This work is free. You can redistribute it and/or modify it under the
 * terms of the Do What The Fuck You Want To Public License, Version 2,
 * as published by Sam Hocevar. See the COPYING file or http://www.wtfpl.net/ 
 * for more details.
 */

#include "rpi_driverd_cam.h"


/**
 * constructor
 */
RPI_Driverd_Cam::RPI_Driverd_Cam()
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
RPI_Driverd_Cam::~RPI_Driverd_Cam()
{
	quit();
}

bool RPI_Driverd_Cam::init(void)
{
	bool ret = false;
	piID = pigpio_start(NULL,NULL);
	if (piID>= 0)	// wait for successed PiGPIO init
	{
		// camera pan & tilt
		set_PWM_frequency(piID,CAM_SERVO1_PWM_OUT, CAM_SERVO_PWM_FREQ);
		set_PWM_frequency(piID,CAM_SERVO2_PWM_OUT, CAM_SERVO_PWM_FREQ);
		set_PWM_range(piID,CAM_SERVO1_PWM_OUT, CAM_SERVO1_PWM_RANGE);
		set_PWM_range(piID,CAM_SERVO2_PWM_OUT, CAM_SERVO1_PWM_RANGE);
		set_PWM_dutycycle(piID,CAM_SERVO1_PWM_OUT, CAM_SERVO1_PWM_INIT);
		set_PWM_dutycycle(piID,CAM_SERVO2_PWM_OUT, CAM_SERVO2_PWM_INIT);
		posServo1 = CAM_SERVO1_PWM_INIT;
		posServo2 = CAM_SERVO2_PWM_INIT;
		ret = true;
	}

	return ret;
}


bool RPI_Driverd_Cam::getInitFlag() const {
	return initFlag;
}

void RPI_Driverd_Cam::quit(void)
{
	pigpio_stop(piID); // reset GPIO configuration
}


void RPI_Driverd_Cam::setCamPos(uint16_t pan, uint16_t tilt)
{
	//set limits
	if(pan>2000) { pan = 2000;}
	if(pan<1000) { pan = 1000;}
	if(tilt>2000) { tilt = 2000;}
	if(tilt<1000) { tilt = 1000;}

	set_PWM_dutycycle(piID,CAM_SERVO1_PWM_OUT, pan);
	set_PWM_dutycycle(piID,CAM_SERVO2_PWM_OUT, tilt);
}


