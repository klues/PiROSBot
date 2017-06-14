/*
 * ROS Node: Joy controller for 
 * 	PiROSBot: 2 DC motors and camera tilt
 * 	uArm:
 * 
 * Copyright © 2017, Thomas Jerabek, UAS Technikum-Wien.
 * This work is free. You can redistribute it and/or modify it under the
 * terms of the Do What The Fuck You Want To Public License, Version 2,
 * as published by Sam Hocevar. See the COPYING file or http://www.wtfpl.net/ 
 * for more details.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sensor_msgs/Joy.h>
#include <uarm_metal/Position.h>
#include "pirosbot/DC_Control.h"
#include "pirosbot/CAM_Control.h"
#include <motor_defs.h>
#include <math.h>       /* fabs */


#define CAM_PWM_INIT		1500
#define CAM_PWM_MIN		1000
#define CAM_PWM_MAX		2000
#define DC_DURATION_INIT	 100

class JoyController
{
public:
	JoyController();
	void info(void);	
	void loop (void);
	

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void uarmCoordsCallback(const uarm_metal::Position::ConstPtr& pos);
	void pirosbotTimerCallback(const ros::TimerEvent&);
	void uarmTimerCallback(const ros::TimerEvent&);

	// joy vars
	ros::NodeHandle joy_node;
  	ros::Subscriber joy_sub;
	sensor_msgs::Joy myJoy;

	double axes[6];
	bool buttons[12];

	// PiROSBot vars
	ros::Publisher dc_control_pub;	
	ros::Publisher cam_control_pub;
	ros::Timer pirosbotTimer;
	ros::Time pirosbotStateTime;
	ros::Duration stateDuration;
	uint8_t pirosbotState;

	uint8_t dc_state;
	uint8_t dc_speed;
	uint32_t dc_duration;	

	uint16_t cam_tilt;
	uint16_t cam_pan;

	// uArm vars
	ros::Publisher uarm_pump_pub;	
	ros::Publisher uarm_cord_pub;

  	ros::Subscriber uarm_coords_sub;
	uint8_t uarmState;
	uarm_metal::Position uarm_pos;
	bool uarmCoordsSet;
	ros::Timer uarmTimer;
	ros::Time uarmStateTime;
};


/**
*
*	JoyController constructor; set initial values and setup publisher 
*
*/
JoyController::JoyController()
{
	pirosbotState = 0;

	dc_state = DC_STOP;
	dc_speed = 50;
	dc_duration = DC_DURATION_INIT;
	stateDuration = (ros::Duration) ( ((double)DC_DURATION_INIT) / 1000);
	//ROS_INFO("Duration: %f", stateDuration.toSec());
	dc_control_pub = joy_node.advertise<pirosbot::DC_Control>("dc_control", 1000);

	cam_tilt = CAM_PWM_INIT;
	cam_pan = CAM_PWM_INIT;
	cam_control_pub = joy_node.advertise<pirosbot::CAM_Control>("cam_control", 1000);

	myJoy.buttons.resize(12);
	myJoy.axes.resize(6);
  	joy_sub = joy_node.subscribe<sensor_msgs::Joy>("joy", 10, &JoyController::joyCallback, this);

	// uArm
	uarmCoordsSet = false;
  	uarm_coords_sub = joy_node.subscribe<uarm_metal::Position>("uarm_metal/position_read", 10, &JoyController::uarmCoordsCallback, this);

	uarm_pump_pub = joy_node.advertise<std_msgs::Bool>("uarm_metal/pump", 1000);

	uarm_cord_pub = joy_node.advertise<uarm_metal::Position>("uarm_metal/position_write", 1000);
}

/**
*
*	JoyController callback; gets & converts joy input to dc and cam control messages 
*
*/
void JoyController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

	// PIROSBOT
	if(joy->axes[3] != myJoy.axes[3])
	{
		//ROS_INFO("Tilt: %f",myJoy.axes[3]);
		pirosbot::CAM_Control cam_ctrl_msg;
		cam_ctrl_msg.tilt = ((joy->axes[3]-3)*(-500)); // [-1...1] -> [2000...1000] > ((x-3)/2) * 1000 *(-1)
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

		if((myState != dc_state) ) 
		{	
			this->pirosbotStateTime = ros::Time::now();
			dc_ctrl_msg.state = myState;
			dc_ctrl_msg.speed = this->dc_speed;//TODO!
			dc_ctrl_msg.duration = this->dc_duration;
			dc_control_pub.publish(dc_ctrl_msg);
			pirosbotTimer.stop();
			this->pirosbotState = 0;
		}
		this->dc_state = myState; // update internal state
	}

	// uArm
	if(joy->buttons[0] != myJoy.buttons[0])
	{
		std_msgs::Bool uarm_pump_ctrl_msg;
		if (joy->buttons[0] == 1)
		{
			uarm_pump_ctrl_msg.data = true;
		}
		else
		{
			uarm_pump_ctrl_msg.data = false;
		}
		uarm_pump_pub.publish(uarm_pump_ctrl_msg);
	}
	

	if((uarmCoordsSet == true) && 
	  ((joy->axes[4] != myJoy.axes[4]) || 
	   (joy->axes[5] != myJoy.axes[5]) || 
	   (joy->buttons[3] != myJoy.buttons[3]) || 
	   (joy->buttons[5] != myJoy.buttons[5]))) // x = [4], y = [5], z = b4 & b6 (=b[3] &b[5]
	{
		uarm_metal::Position uarm_coords_msg;
		uarm_coords_msg.x = this->uarm_pos.x;
		uarm_coords_msg.y = this->uarm_pos.y;
		uarm_coords_msg.z = this->uarm_pos.z;
	
		if(joy->axes[4] > 0) 
		{uarm_coords_msg.x  +=1;}
		else if(joy->axes[4] < 0)
		{uarm_coords_msg.x  -=1;}
		

		if(joy->axes[5] > 0) 
		{uarm_coords_msg.y +=1;}
		else if(joy->axes[5] < 0)
		{uarm_coords_msg.y -=1;}
		
		if((joy->buttons[5] > 0) && (joy->buttons[3] < 1))
		{uarm_coords_msg.z +=1;}
		else if((joy->buttons[3] > 0) && (joy->buttons[5] < 1))
		{uarm_coords_msg.z -=1;}
		
		uarm_coords_msg.x = roundf(uarm_coords_msg.x);//*100.0)/100.0F;
		uarm_coords_msg.y = roundf(uarm_coords_msg.y);//*100.0)/100.0F;
		uarm_coords_msg.z = roundf(uarm_coords_msg.z);//*100.0)/100.0F;

		//ROS_INFO("x %f, y %f, z %f",uarm_coords_msg.x,uarm_coords_msg.y,uarm_coords_msg.z);
		uarm_cord_pub.publish(uarm_coords_msg);
		this->uarmStateTime = ros::Time::now();
		uarmTimer.stop();
		this->uarmState = 0;

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

void JoyController::uarmCoordsCallback(const uarm_metal::Position::ConstPtr& pos)
{
	this->uarm_pos.x = pos->x;
	this->uarm_pos.y = pos->y;
	this->uarm_pos.z = pos->z;
	uarmCoordsSet = true;

}

void JoyController::pirosbotTimerCallback(const ros::TimerEvent& event)
{
	//ROS_INFO("t_cb1");
	this->pirosbotState = 1;
}

void JoyController::uarmTimerCallback(const ros::TimerEvent& event)
{
	//ROS_INFO("t_cb2");
	this->uarmState = 1;
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

void JoyController::loop(void)
{
	pirosbot::DC_Control dc_ctrl_msg;
	pirosbot::CAM_Control cam_ctrl_msg;
	uarm_metal::Position uarm_coords_msg;
	pirosbotTimer = joy_node.createTimer(ros::Duration(0.025), &JoyController::pirosbotTimerCallback,this, true);
	uarmTimer = joy_node.createTimer(ros::Duration(0.025), &JoyController::uarmTimerCallback,this, true);
	
	pirosbotTimer.stop();
	uarmTimer.stop();
	this->pirosbotState = 0;
	this->uarmState = 0;

	while (ros::ok())
	{
		if( (dc_state != DC_STOP) && 
		    (this->pirosbotState != 2) &&
		    ((ros::Time::now().toSec() + 0.05) >= (pirosbotStateTime.toSec() + stateDuration.toSec())) )
		{	
			this->pirosbotStateTime = ros::Time::now();
			dc_ctrl_msg.state = dc_state;
			dc_ctrl_msg.speed = this->dc_speed;//TODO!
			dc_ctrl_msg.duration = this->dc_duration;
			dc_control_pub.publish(dc_ctrl_msg);
			this->pirosbotState = 0;
		
		}
		else if(this->pirosbotState == 0)
		{
			pirosbotTimer.setPeriod(ros::Duration(0.025));
			pirosbotTimer.start();
			this->pirosbotState = 2;
		}
	
		if( (uarmCoordsSet == true) && 
		    (this->uarmState == 1) && 
		    ((fabs(myJoy.axes[4]) > 0) || (fabs(myJoy.axes[5]) > 0) || 
		    (myJoy.buttons[3] > 0) || (myJoy.buttons[5] > 0)) )
		{	
			this->uarmStateTime = ros::Time::now();
			
			uarm_coords_msg.x = this->uarm_pos.x;
			uarm_coords_msg.y = this->uarm_pos.y;
			uarm_coords_msg.z = this->uarm_pos.z;
	
			if(myJoy.axes[4] > 0) 
			{uarm_coords_msg.x  +=1;}
			else if(myJoy.axes[4] < 0)
			{uarm_coords_msg.x  -=1;}
		

			if(myJoy.axes[5] > 0) 
			{uarm_coords_msg.y +=1;}
			else if(myJoy.axes[5] < 0)
			{uarm_coords_msg.y -=1;}
		
			if((myJoy.buttons[5] > 0) && (myJoy.buttons[3] < 1))
			{uarm_coords_msg.z +=1;}
			else if((myJoy.buttons[3] > 0) && (myJoy.buttons[5] < 1))
			{uarm_coords_msg.z -=1;}
		
			uarm_coords_msg.x = roundf(uarm_coords_msg.x);//*100.0)/100.0F;
			uarm_coords_msg.y = roundf(uarm_coords_msg.y);//*100.0)/100.0F;
			uarm_coords_msg.z = roundf(uarm_coords_msg.z);//*100.0)/100.0F;

			uarm_cord_pub.publish(uarm_coords_msg);

			this->uarmState = 0;
		}
		else if(this->uarmState == 0)
		{
			uarmTimer.setPeriod(ros::Duration(0.25));
			uarmTimer.start();
			this->uarmState = 2;
		}
		ros::spinOnce();	
	}

  	return;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PiROSBot_JoyController");

	JoyController myJoyController;

	myJoyController.info();

	myJoyController.loop();
	
	//ros::spin();
	ros::shutdown();	// shutdown ros
	return 0;
}

