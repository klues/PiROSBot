#! /bin/bash
ROSLAUNCH_BIN="roslaunch"
ROSLAUNCH_FILE="raspicam_node camerav2_640x480.launch"
ROSMAINSOURCE="/opt/ros/kinetic/setup.bash"
ROSSOURCE="/home/ros/catkin_ws/devel/setup.bash"

echo "source ... "
source $ROSMAINSOURCE
source $ROSSOURCE 

export ROS_PORT=11311
export ROS_HOSTNAME=$(hostname)
export ROS_MASTER_URI=http://localhost:$ROS_PORT

echo $ROS_PORT
echo $ROS_IP
echo $ROS_MASTER_URI

echo "testing camera ..."
python /home/ros/services/cameratest.py

echo "starting ros camera ... "
$ROSLAUNCH_BIN $ROSLAUNCH_FILE --port=$ROS_PORT
