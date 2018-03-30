/*
 * ROS Node: Keyboard controller for 2 DC motors and camera pan & tilt (2 servomotors).
 * 
 * 
 * Copyright © 2017, Thomas Jerabek, UAS Technikum-Wien.
 * This work is free. You can redistribute it and/or modify it under the
 * terms of the Do What The Fuck You Want To Public License, Version 2,
 * as published by Sam Hocevar. See the COPYING file or http://www.wtfpl.net/ 
 * for more details.
 */

#include "srf02.h"
#include <stdio.h>

int main(int argc, char **argv)
{
	int i;
	SRF02 srf02;
	for(i=0; i<100; i++) {
       int distance = srf02.getDistance();
       printf("%d\n", distance);
    }
	return 0;
}

