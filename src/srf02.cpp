/*
 * Class for reading distance of SRF02 sensor over I2C
 * 
 * 
 * Copyright © 2018, Benjamin Klaus, UAS Technikum-Wien.
 * This work is free. You can redistribute it and/or modify it under the
 * terms of the Do What The Fuck You Want To Public License, Version 2,
 * as published by Sam Hocevar. See the COPYING file or http://www.wtfpl.net/ 
 * for more details.
 */

#include "srf02.h"
#include <unistd.h>


/**
 * constructor
 */
SRF02::SRF02()
{
	piID = pigpio_start(NULL,NULL);
}

/**
 * destructor
 */
SRF02::~SRF02()
{
}


uint16_t SRF02::getDistance(void)
{
	uint16_t distance = -1;
	int handle;
	
	handle = i2c_open(piID,1,0x70,0); // "/dev/i2c-1", default address is 0xE0 -> must be shifted right

	if(handle>=0)
	{
		//TODO: do distance measurement for SRF02 ultrasonic module
		//use methods
		//int i2c_write_byte_data(int pi, unsigned handle, unsigned i2c_reg, unsigned bVal)
		//int i2c_read_byte_data(int pi, unsigned handle, unsigned i2c_reg)
		//int i2c_close(int pi, unsigned handle)
		//int usleep(useconds_t usec);
		
		//see documentation:
		// http://abyz.me.uk/rpi/pigpio/pdif2.html#i2c_write_byte
		// http://man7.org/linux/man-pages/man3/usleep.3.html
		// https://www.robot-electronics.co.uk/htm/srf02techI2C.htm
	}

	return distance;	
}


