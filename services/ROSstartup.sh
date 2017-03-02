#! /bin/bash

ROS_PORT=11311

ROSLAUNCH_BIN="roslaunch"
ROSMAINSOURCE="/opt/ros/kinetic/setup.bash"
ROSSOURCE="/home/ros/catkin_ws/devel/setup.bash"
ROSLAUNCH_FILE="pirosbot rpidriver.launch"

echo "source ... "
source $ROSMAINSOURCE
source $ROSSOURCE 

# wait for network connection (wifi) before setting ROS IP
echo "waiting for wifi connection ... " 
#while ! ping -c 1 -W 1 192.168.0.50 ; do
#	count=$((count+1))
#	echo $count
#	sleep 1
#done

# set current IP for ROS network
export ROS_IP=$(hostname -I)
export ROS_MASTER_URI=http://localhost:$ROS_PORT


echo "starting PIGPIO Daemon ..."
pigpiod

sleep 2

echo $ROS_IP
echo $ROS_MASTER_URI
echo "starting ros ... "
$ROSLAUNCH_BIN $ROSLAUNCH_FILE --port=$ROS_PORT

