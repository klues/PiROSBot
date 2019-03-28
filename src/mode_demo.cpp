/*
 * Demo mode, node that moves motor/cam left/right in a loop,
 * subscribes to topic "distance_fake"
 * 
 * Copyright © 2019, Benjamin Klaus, UAS Technikum-Wien.
 * This work is free. You can redistribute it and/or modify it under the
 * terms of the Do What The Fuck You Want To Public License, Version 2,
 * as published by Sam Hocevar. See the COPYING file or http://www.wtfpl.net/ 
 * for more details.
 */

#include "ros/ros.h"
#include "pirosbot/DC_Control.h"
#include "pirosbot/CAM_Control.h"
#include <motor_defs.h>
#include "std_msgs/Int64.h"

int currentDistance = -1;
ros::Publisher dc_control_pub;
ros::Publisher cam_control_pub;

void distance_Callback(const std_msgs::Int64& msg)
{
	currentDistance = msg.data;
	ROS_INFO("got distance value: %dcm", currentDistance);
}

void driveMotor(int state, int speed, int duration) {
    pirosbot::DC_Control dc_ctrl_msg;
    dc_ctrl_msg.state = state;
    dc_ctrl_msg.speed = speed;
    dc_ctrl_msg.duration = duration;
    dc_control_pub.publish(dc_ctrl_msg);
}

void moveCam(int cam_tilt, int cam_pan) {
    pirosbot::CAM_Control cam_ctrl_msg;
    cam_ctrl_msg.tilt = cam_tilt;
    cam_ctrl_msg.pan = cam_pan;
    cam_control_pub.publish(cam_ctrl_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "DemoMode");
	ros::NodeHandle node_handle;
	dc_control_pub = node_handle.advertise<pirosbot::DC_Control>("dc_control", 1000);
    cam_control_pub = node_handle.advertise<pirosbot::CAM_Control>("cam_control", 1000);
	ros::Subscriber subDistance = node_handle.subscribe("distance_fake", 1000, distance_Callback);


    int minCamValue = 1300;
    int maxCamValue = 1700;
    int middle = 1500;
	while(ros::ok()) {
	    driveMotor(DC_LEFT, 50, 500);
	    ros::Duration(1.0).sleep();
	    driveMotor(DC_RIGHT, 50, 500);
	    ros::Duration(1.0).sleep();
        moveCam(middle, minCamValue);
        ros::Duration(1.0).sleep();
        moveCam(middle, maxCamValue);
        ros::Duration(1.0).sleep();
        moveCam(middle, middle);
        ros::Duration(3.0).sleep();
    }

	ros::shutdown();
    return 0;
}
