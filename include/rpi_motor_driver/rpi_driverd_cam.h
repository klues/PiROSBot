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

#ifndef RPI_DRIVERD_CAM_H
#define RPI_DRIVERD_CAM_H

#include <stdint.h>
#include <pigpiod_if2.h>
#include <motor_defs.h>


class RPI_Driverd_Cam
{
public:
	RPI_Driverd_Cam();
	virtual ~RPI_Driverd_Cam();

	bool init(void);

	// Camera Servos
	void setCamPos(uint16_t pan, uint16_t tilt); //pan = x; tilt = rotate 

	bool getInitFlag()const;

	void quit(void);



private:

	// CAMERA CONTROL SERVOS
	static const uint8_t CAM_SERVO1_PWM_OUT = 23; // Servo 1 for camera on pin 23
	static const uint8_t CAM_SERVO2_PWM_OUT = 24; // Servo 1 for camera on pin 24

	static const uint16_t CAM_SERVO_PWM_FREQ = 50; // 50Hz
	static const uint16_t CAM_SERVO1_PWM_RANGE = 20000; // range
	static const uint16_t CAM_SERVO1_PWM_INIT = 1500; // middle position (1.5ms pulse)
	static const uint16_t CAM_SERVO2_PWM_INIT = 1500; // middle position (1.5ms pulse)

	// PIGPIO deamon ID
	int piID; 

	//CAM Servo variables
	uint16_t posServo1;
	uint16_t posServo2;

	bool initFlag;							// true after correct GPIO init	

};




#endif
