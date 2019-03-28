/*
 * ROS Node: Fake Distance node - publishes simple "fake" distance data, can be used for testing
 * instead of real distance node.
 *
 * Copyright © 2019, Benjamin Klaus, UAS Technikum-Wien.
 * This work is free. You can redistribute it and/or modify it under the
 * terms of the Do What The Fuck You Want To Public License, Version 2,
 * as published by Sam Hocevar. See the COPYING file or http://www.wtfpl.net/ 
 * for more details.
 */

#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "pirosbot/SetDistance.h"

uint16_t currentDistance = 20;

bool setDistanceCallback(pirosbot::SetDistance::Request &req, pirosbot::SetDistance::Response &res) {
	int value = req.value;
	std::stringstream ss;
	if(value <= 0) {
		ss << "invalid value (<=0):" << value;
		res.result = ss.str();
		return true;
	}
	currentDistance = value;
	ss << "fake distance set to:" << value;
	res.result = ss.str();
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "DistanceNodeFake");
    ros::NodeHandle node_handle;
    ros::Publisher distance_pub;
    distance_pub = node_handle.advertise<std_msgs::Int64>("distance_fake", 1000);
    ros::ServiceServer service = node_handle.advertiseService("setDistanceFake", setDistanceCallback);
    ros::Rate r(3); // 10 hz

	while(ros::ok()) {
		std_msgs::Int64 msg;
		msg.data = currentDistance;
		distance_pub.publish(msg);
		ROS_INFO("sent fake distance: %dcm", currentDistance);
		r.sleep();
        ros::spinOnce();
    }
    ros::shutdown();	// shutdown ros
	return 0;
}

