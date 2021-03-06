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

#ifndef RPI_DRIVERD_MOT_H
#define RPI_DRIVERD_MOT_H

#include <stdint.h>
#include <pigpiod_if2.h>
#include <motor_defs.h>


class RPI_Driverd_Mot
{
public:
	RPI_Driverd_Mot();
	virtual ~RPI_Driverd_Mot();

	bool init(void);
	// DC Engines
	void set_motor(bool A1, bool A2, bool B1, bool B2);
	void forward(void);
	void stop(void);
	static void* dc_stop_thread(void *user);
	void reverse(void);
	void left(void);
	void right(void);
	void setDCSpeed(uint8_t dutycycle);
	void setDCDuration(uint32_t dc_duration);
	void setDCState(uint8_t dc_state);
	uint8_t getDCSpeed() const;
	uint8_t getDCState() const;
	uint32_t getDCDuration() const;

	bool getInitFlag()const;
	// TODO: duration timer with CALLBACK !
	void quit(void);



private:
	
	
	static const bool IRRC_ENABLE = false;
	// GPIO configuration for both MC33886 chips
	static const uint8_t IRRC_IN = 18;		// IR remote control input

	static const uint8_t MotA1_OUT = 6;		//  Motor A IN1
	static const uint8_t MotA2_OUT = 13; 	//  Motor A IN2
	static const uint8_t PWMA_OUT = 12;		//  Motor A PWM

//	static const uint8_t MotB1_OUT = 20;	//  Motor B IN1
//	static const uint8_t MotB2_OUT = 21;	//  Motor B IN2
	static const uint8_t MotB1_OUT = 21;	//  Motor B IN1
	static const uint8_t MotB2_OUT = 20;	//  Motor B IN2
	static const uint8_t PWMB_OUT = 26;		//  Motor B PWM

	static const uint16_t PWM_FREQ = 1000;	// PWM Frequency: 1kHz (max 10kHz!)
	static const uint8_t PWM_RANGE = 100;	// PWM range (from 0 to 100) -> fine enough?
	static const uint8_t PWM_INIT = 50;		// initial duty cycle

	static const uint8_t STOP_TIMER = 0;	// Timer ID for stopping the current movement

	static const uint16_t STOP_TIMER_INIT = 1000;
	

	// PIGPIO deamon ID
	int piID; 
	bool startTimer;
	bool pthKeepAlive;		// DO NOT USE - set in init() and reset in quit()
	pthread_t *_pth;

	//DC motor variables
	uint8_t dc_state;
	uint8_t dc_speed;
	uint32_t dc_duration;

	bool initFlag;							// true after correct GPIO init	

};




#endif
