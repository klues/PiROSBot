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

#ifndef SRF02_H
#define SRF02_H

#include <pigpiod_if2.h>

class SRF02
{
public:
	SRF02();
	virtual ~SRF02();

	uint16_t getDistance(void);

private:

	int piID;
};

#endif
