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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rpi_motor_control/DC_Control.h"
#include "rpi_motor_control/CAM_Control.h"
#include <motor_defs.h>
#include <signal.h>
#include <termios.h>
#include <sstream>



#define KEYCODE_LEFT 		0x44
#define KEYCODE_UP 			0x41
#define KEYCODE_RIGHT 		0x43 
#define KEYCODE_DOWN 		0x42
#define KEYCODE_STOP 		' '
#define KEYCODE_CLOSE 		'c'
#define KEYCODE_QUIT 		'q'
#define KEYCODE_INCSPEED 	'm'
#define KEYCODE_DECSPEED 	'n'
#define KEYCODE_INCDURATION 'b'
#define KEYCODE_DECDURATION 'v'
#define KEYCODE_CAM_UP 		'w'
#define KEYCODE_CAM_DOWN 	's'
#define KEYCODE_CAM_LEFT 	'a'
#define KEYCODE_CAM_RIGHT 	'd'

#define TOPIC_NONE		0
#define TOPIC_DC_MOTOR	1
#define TOPIC_CAM	2

#define CAM_PWM_INIT	1500
#define CAM_PWM_MIN		1000
#define CAM_PWM_MAX		2000


class KeyController
{
public:
	KeyController();
	void keyLoop(void);	

private:
	ros::NodeHandle control_node;
	ros::Publisher dc_control_pub;	
	ros::Publisher cam_control_pub;
	uint8_t dc_state;
	uint8_t dc_speed;
	uint32_t dc_duration;	

	uint16_t cam_tilt;
	uint16_t cam_pan;

	//terminal stuff
	int kfd;
	struct termios cooked, raw;
};


/**
*
*	KeyController constructor; set initial values and setup publisher 
*
*/
KeyController::KeyController()
{
	kfd = 0;

	dc_state = 0;
	dc_speed = 50;
	dc_duration = 1000;
	dc_control_pub = control_node.advertise<rpi_motor_control::DC_Control>("dc_control", 1000);

	cam_tilt = CAM_PWM_INIT;
	cam_pan = CAM_PWM_INIT;
	cam_control_pub = control_node.advertise<rpi_motor_control::CAM_Control>("cam_control", 1000);
}

/**
*	
*	capture keyboard input and transmit to RPi motor driver node
*	
*/
void KeyController::keyLoop(void)
{
	char c;
 	uint8_t updateFlag = TOPIC_NONE;
 	bool quitFlag = false;

	// get the console in raw mode                                                              
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO ); //| ISIG);
	// Setting a new line, then end of file                         
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("-------------------------------------------");
	puts("Keyboard Control for DC motors and camera movement:");
	puts("- Arrow keys:			DC: move robot");
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
	puts("- q:			quit");
	puts("-------------------------------------------");

	while (ros::ok())
	{
	// get the next event from the keyboard  
	if(read(kfd, &c, 1) < 0)
	{
	  perror("read():");
	  exit(-1);
	}

	updateFlag = TOPIC_NONE; // reset topic

	switch(c)
	{
	  case KEYCODE_LEFT:
		ROS_INFO("LEFT");
		dc_state = DC_LEFT;
		updateFlag = TOPIC_DC_MOTOR;
		break;

	  case KEYCODE_RIGHT:
		ROS_INFO("RIGHT");
		dc_state = DC_RIGHT;
		updateFlag = TOPIC_DC_MOTOR;
		break;

	  case KEYCODE_UP:
		ROS_INFO("FORWARD");
		dc_state = DC_FORWARD;
		updateFlag = TOPIC_DC_MOTOR;
		break;

	  case KEYCODE_DOWN:
		ROS_INFO("BACKWARD");
		dc_state = DC_BACKWARD;
		updateFlag = TOPIC_DC_MOTOR;
		break;

	 case KEYCODE_STOP: // space 0x20
		ROS_INFO("STOP");
		dc_state = DC_STOP;
		updateFlag = TOPIC_DC_MOTOR;
		break;

	 case KEYCODE_DECSPEED: 
		if(dc_speed >= SPEED_MIN+SPEED_STEP)
		{
			dc_speed -= SPEED_STEP;
			ROS_INFO("Speed(-): %d %%", dc_speed);
		}
		dc_state = DC_SU;
		updateFlag = TOPIC_DC_MOTOR;
		break;

	 case KEYCODE_INCSPEED:
		if(dc_speed <= SPEED_MAX-SPEED_STEP)
		{
			dc_speed += SPEED_STEP;
			ROS_INFO("Speed(+): %d %%", dc_speed);
		}
		dc_state = DC_SU;
		updateFlag = TOPIC_DC_MOTOR;
		break;

	 case KEYCODE_DECDURATION:
		if(dc_duration >= DURATION_MIN+DURATION_STEP)
		{
			dc_duration -= DURATION_STEP;
			ROS_INFO("Duration(-): %d ms", dc_duration);
		}
		dc_state = DC_DU;
		updateFlag = TOPIC_DC_MOTOR;
		break;

	 case KEYCODE_INCDURATION:
		if(dc_duration <= DURATION_MAX-DURATION_STEP)
		{
			dc_duration += DURATION_STEP;
			ROS_INFO("Duration(+): %d ms", dc_duration);
		}
		dc_state = DC_DU;
		updateFlag = TOPIC_DC_MOTOR;
		break;


	 case KEYCODE_CAM_UP: // w
		if(cam_tilt > CAM_PWM_MIN)
		{
			cam_tilt -=20;
			ROS_INFO("camera up (%d)", cam_tilt );
		}
		updateFlag = TOPIC_CAM;
		break;

	 case KEYCODE_CAM_DOWN: // s
		if(cam_tilt < CAM_PWM_MAX)
		{
			cam_tilt +=20;
			ROS_INFO("camera down (%d)", cam_tilt );
		}
		updateFlag = TOPIC_CAM;
		break;

	 case KEYCODE_CAM_LEFT: // a
		if(cam_pan < CAM_PWM_MAX)
		{
			cam_pan +=20;
			ROS_INFO("camera left (%d)", cam_pan );
		}
		updateFlag = TOPIC_CAM;
		break;

	 case KEYCODE_CAM_RIGHT: // d
		if(cam_pan > CAM_PWM_MIN)
		{
			cam_pan -=20;
			ROS_INFO("camera right (%d)", cam_pan);
		}
		updateFlag = TOPIC_CAM;
		break;
	//case KEYCODE_INTERRUPT:
	case KEYCODE_CLOSE:
	case KEYCODE_QUIT:
		quitFlag = true;
		break;
 
	default:
		//ROS_INFO("input: 0x%02X\n", c);
		break;
	}

	if(updateFlag == TOPIC_DC_MOTOR) // create & send message for DC engine control
	{
		rpi_motor_control::DC_Control dc_ctrl_msg;

		// TODO: check limits!
		dc_ctrl_msg.state = dc_state;
		dc_ctrl_msg.speed = dc_speed;
		dc_ctrl_msg.duration = dc_duration;
		dc_control_pub.publish(dc_ctrl_msg);
		updateFlag = TOPIC_NONE;
	}
	else if(updateFlag == TOPIC_CAM) // create & send message for DC engine control
	{
		rpi_motor_control::CAM_Control cam_ctrl_msg;

		// TODO: check limits!
		cam_ctrl_msg.tilt = cam_tilt;
		cam_ctrl_msg.pan = cam_pan;
		cam_control_pub.publish(cam_ctrl_msg);
		updateFlag = TOPIC_NONE;
	}
	if(quitFlag)
	{
		tcsetattr(kfd, TCSANOW, &cooked); // restore terminal
		ros::shutdown();	// shutdown ros
		exit(0);	// exit application
	}
}


  return;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "RPiMotorKeyController");

	KeyController myKeyController;

	myKeyController.keyLoop();

	return 0;
}

