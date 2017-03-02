/*
 * ROS Camera Driver Node for Camera pan & tilt (2 servomotors)
 * It uses the PIGPIO Daemon as GPIO-Interface (default socket)
 * 
 * 
 * Copyright © 2017, Thomas Jerabek, UAS Technikum-Wien.
 * This work is free. You can redistribute it and/or modify it under the
 * terms of the Do What The Fuck You Want To Public License, Version 2,
 * as published by Sam Hocevar. See the COPYING file or http://www.wtfpl.net/ 
 * for more details.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pirosbot/CAM_Control.h"
#include <motor_defs.h>
#include <rpi_driverd_cam.h>
#include <stdio.h>

#define PIN		18
#define PWMA1	 6 
#define PWMA2	13
#define PWMB1	20
#define PWMB2	21
#define D1  	12
#define D2 		26
#define PWM		50

RPI_Driverd_Cam myDriver;


void cam_ControlCallback(const pirosbot::CAM_Control& msg)
{
		myDriver.setCamPos(msg.pan, msg.tilt);
		ROS_INFO("CAM control!");
}

/*
void cleanQuit(void)
{
	ROS_INFO("reset GPIO configuration & quit node");

	myDriver.quit(); // reset GPIO configuration
	ros::shutdown();
	exit(0);
}*/

int main(int argc, char **argv)
{
	if(!myDriver.getInitFlag())
	{
		ROS_INFO("GPIO init failed!");
		return -1;
	}


	ros::init(argc, argv, "RPiCameraDriver");

	ros::NodeHandle n;

	ros::Subscriber subCAMControl = n.subscribe("cam_control", 1000, cam_ControlCallback);

	ros::spin();

	printf("reset GPIO configuration & quit node\n");

	myDriver.quit(); // reset GPIO configuration
	//ros::shutdown(); // ros::spin shuts down

	return 0;
}
