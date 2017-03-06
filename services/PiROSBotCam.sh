#! /bin/bash

ROS_PORT=11311

ROSLAUNCH_BIN="roslaunch"
ROSMAINSOURCE="/opt/ros/kinetic/setup.bash"
ROSSOURCE="/home/ros/catkin_ws/devel/setup.bash"
ROSLAUNCH_FILE="raspicam_node camerav2_640x480.launch"


echo "source ... "
source $ROSMAINSOURCE
source $ROSSOURCE 

# set current IP for ROS network
export ROS_IP=$(hostname -I)
export ROS_MASTER_URI=http://localhost:$ROS_PORT

# wait for startup of PiROSBot Motor nodes
sleep 10

echo $ROS_IP
echo $ROS_MASTER_URI
echo "starting ros camera ... "
$ROSLAUNCH_BIN $ROSLAUNCH_FILE --port=$ROS_PORT

