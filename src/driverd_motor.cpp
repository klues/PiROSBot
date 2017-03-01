/*
 * ROS Motor Driver Node for 2 DC motors
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
#include "rpi_motor_control/DC_Control.h"
#include <motor_defs.h>
#include <rpi_driverd_mot.h>
//#include <stdio.h>

#define PIN		18
#define PWMA1	 6 
#define PWMA2	13
#define PWMB1	20
#define PWMB2	21
#define D1  	12
#define D2 		26
#define PWM		50

RPI_Driverd_Mot myDriver;


void dc_ControlCallback(const rpi_motor_control::DC_Control& msg)
{

  	switch(msg.state)
  	{
		case DC_STOP:
			myDriver.stop();
			ROS_INFO("STOP!");
			break;

		case DC_FORWARD:
			myDriver.setDCSpeed(msg.speed);
			myDriver.setDCDuration(msg.duration);
			myDriver.forward();
			ROS_INFO("Move forward for %d ms with %d speed", msg.duration, msg.speed);
			break;

		case DC_BACKWARD:
			myDriver.setDCSpeed(msg.speed);
			myDriver.setDCDuration(msg.duration);
			myDriver.reverse();
			ROS_INFO("Move backward for %d ms with %d speed", msg.duration, msg.speed);
			break;

		case DC_LEFT:
			myDriver.setDCSpeed(msg.speed);
			myDriver.setDCDuration(msg.duration);
			myDriver.left();
			ROS_INFO("Move left for %d ms with %d speed", msg.duration, msg.speed);
			break;

		case DC_RIGHT:
			myDriver.setDCSpeed(msg.speed);
			myDriver.setDCDuration(msg.duration);
			myDriver.right();
			ROS_INFO("Move right for %d ms with %d speed", msg.duration, msg.speed);
			break;

		case DC_NC:
			ROS_INFO("not changed");
			break;

		case DC_SU:
			myDriver.setDCSpeed(msg.speed);
			ROS_INFO("Speed update: %d %%", msg.speed);
			break;

		case DC_DU:
			myDriver.setDCDuration(msg.duration);
			ROS_INFO("Duration update: %d ms", msg.duration);
			break;

		default:
			ROS_INFO("default: [%d][%d][%d]", msg.state, msg.speed, msg.duration);
			break;
	}

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


	ros::init(argc, argv, "RPiMotorDriver");

	ros::NodeHandle n;

	ros::Subscriber subDCControl = n.subscribe("dc_control", 1000, dc_ControlCallback);
	
	ros::spin();

	//printf("reset GPIO configuration & quit node\n");

	myDriver.quit(); // reset GPIO configuration
	//ros::shutdown(); // ros::spin() shuts already down

	return 0;
}
