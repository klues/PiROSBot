#! /bin/bash
ROSLAUNCH_BIN="roslaunch"
ROSLAUNCH_FILE="pirosbot rpidriver.launch"
ROSMAINSOURCE="/opt/ros/kinetic/setup.bash"
ROSSOURCE="/home/ros/catkin_ws/devel/setup.bash"

echo "source ... "
source $ROSMAINSOURCE
source $ROSSOURCE 

export ROS_PORT=11311
export ROS_IP=$(hostname -I)
export ROS_MASTER_URI=http://localhost:$ROS_PORT

echo "starting PIGPIO Daemon ..."
pigpiod

sleep 2

echo $ROS_PORT
echo $ROS_IP
echo $ROS_MASTER_URI
echo "starting ros PiROSBot motor drivers... "
$ROSLAUNCH_BIN $ROSLAUNCH_FILE --port=$ROS_PORT

