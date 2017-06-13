/*
 * ROS Node: Joy controller for 2 DC motors and camera pan & tilt (2 servomotors).
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
#include <sensor_msgs/Joy.h>
#include "pirosbot/DC_Control.h"
#include "pirosbot/CAM_Control.h"
#include <motor_defs.h>
#include <math.h>       /* fabs */



#define CAM_PWM_INIT		1500
#define CAM_PWM_MIN		1000
#define CAM_PWM_MAX		2000


class JoyController
{
public:
	JoyController();
	void info(void);	


private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle joy_node;
	ros::Publisher dc_control_pub;	
	ros::Publisher cam_control_pub;
  	ros::Subscriber joy_sub;

	sensor_msgs::Joy myJoy;
	double axes[6];
	bool buttons[12];

	uint8_t dc_state;
	uint8_t dc_speed;
	uint32_t dc_duration;	

	uint16_t cam_tilt;
	uint16_t cam_pan;
};


/**
*
*	JoyController constructor; set initial values and setup publisher 
*
*/
JoyController::JoyController()
{
	dc_state = 0;
	dc_speed = 50;
	dc_duration = 35;
	dc_control_pub = joy_node.advertise<pirosbot::DC_Control>("dc_control", 1000);

	cam_tilt = CAM_PWM_INIT;
	cam_pan = CAM_PWM_INIT;
	cam_control_pub = joy_node.advertise<pirosbot::CAM_Control>("cam_control", 1000);

	myJoy.buttons.resize(12);
	myJoy.axes.resize(6);
  	joy_sub = joy_node.subscribe<sensor_msgs::Joy>("joy", 10, &JoyController::joyCallback, this);
}

/**
*
*	JoyController callback; gets & converts joy input to dc and cam control messages 
*
*/
void JoyController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

	
	if(joy->axes[3] != myJoy.axes[3])
	{
		//ROS_INFO("Tilt: %f",myJoy.axes[3]);
		pirosbot::CAM_Control cam_ctrl_msg;
		cam_ctrl_msg.tilt = ((joy->axes[3]+3)*500); // [-1...1] -> [1000...2000] > ((x+3)/2) * 1000
		cam_ctrl_msg.pan = 0;
		cam_control_pub.publish(cam_ctrl_msg);
	}
	
	if((joy->axes[0] != myJoy.axes[0]) || (joy->axes[1] != myJoy.axes[1]))
	{
		uint8_t myState = DC_STOP;
		pirosbot::DC_Control dc_ctrl_msg;
		if((joy->axes[0] > 0) && (fabs(joy->axes[1]) <= joy->axes[0]))
		{
			myState = DC_LEFT;
		}
		else if((joy->axes[0] < 0) && (fabs(joy->axes[1]) >= joy->axes[0]))
		{
			myState = DC_RIGHT;
		}
		else if((joy->axes[1] > 0) && (fabs(joy->axes[0]) <= joy->axes[1]))
		{
			myState = DC_FORWARD;
		}
		else if((joy->axes[1] < 0) && (fabs(joy->axes[0]) >= joy->axes[1]))
		{
			myState = DC_BACKWARD;
		}
		/*else 
		{
			myState = DC_STOP;
		}*/

		dc_ctrl_msg.state = myState;
		dc_ctrl_msg.speed = this->dc_speed;//TODO!
		dc_ctrl_msg.duration = this->dc_duration;
		dc_control_pub.publish(dc_ctrl_msg);
	}

	for(int i=0;i<joy->buttons.size();i++) // use std::copy(...)
	{
		myJoy.buttons[i] = joy->buttons[i];
	}
	for(int i=0;i<joy->axes.size();i++)
	{
		myJoy.axes[i] = joy->axes[i];
	}
	//ROS_INFO("joy callback");
}

/**
*	
*	show control info
*	
*/
void JoyController::info(void)
{
	puts("-------------------------------------------");
	puts("Joy Control for DC motors, camera movement and uArm:");
	puts("... todo ... ");
	/*puts("- Arrow keys:			DC: move robot");
	puts("- Space:		DC: stop");
	puts("- m:			DC: increase speed");
	puts("- n:			DC: decrease speed");
	puts("- b:			DC: increase duration (move time)");
	puts("- v:			DC: decrease duration (move time)");
	puts("- w:			CAM: up");
	puts("- s:			CAM: down");
	puts("- a:			CAM: left");
	puts("- d:			CAM: right");
	puts("- c:			close/quit");
	puts("- q:			quit");*/
	puts("-------------------------------------------");

  	return;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "PiROSBot_JoyController");

	JoyController myJoyController;

	myJoyController.info();

	ros::spin();
	return 0;
}

