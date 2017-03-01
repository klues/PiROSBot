/*
 * Defines for DC motor driver.
 * 
 * 
 * Copyright © 2017, Thomas Jerabek, UAS Technikum-Wien.
 * This work is free. You can redistribute it and/or modify it under the
 * terms of the Do What The Fuck You Want To Public License, Version 2,
 * as published by Sam Hocevar. See the COPYING file or http://www.wtfpl.net/ 
 * for more details.
 */

#ifndef MOTOR_DEFS_H
#define MOTOR_DEFS_H

#define DC_STOP	 	0
#define DC_FORWARD	1
#define DC_BACKWARD 2
#define DC_LEFT	 	3
#define	DC_RIGHT	4
#define	DC_NC		5	//not changed
#define	DC_SU		6	//speed update
#define	DC_DU		7	//duration update


#define SPEED_MIN		0
#define SPEED_MAX		100
#define SPEED_STEP		5

#define	DURATION_MIN	100
#define	DURATION_MAX	10000
#define	DURATION_STEP	100

#endif
