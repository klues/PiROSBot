# ROS Node for PiROSBot

To avoid running out of memory during the build process on Raspberry Pi 3, execute catkin_make with the option -j1 or --jobs=1.
e.g., $catkin_make -j1

uses https://github.com/joan2937/pigpio to control GPIOs (included in this repo - no submodule)



# Joy Controller

The joy controller is an experimental interface for PiROSBot and uArm!

## PiROSBot

Setup the uarm_metal ROS package https://github.com/jerabek/uarm_metal with all depenencies (see uarm_metal README).

$ rosrun uarm_metal uarm.py / or $ roslaunch uarm_metal uarm.launch

$ rosparam set /uarm_metal/read_position 1 (also executable from remote pc)

## Remote (PC)
Following steps are necessary to use the joy controller:

$ sudo apt-get install ros-kinetic-joy

$ sudo chmod a+rw /dev/input/jsX (X equals the device number e.g., 0)



Once the ROS_MASTER_URI & ROS_IP are set and roscore is running:

Terminal 1:

$ rosparam set joy_node/dev "/dev/input/jsX" (X equals the device number e.g., 0)

$ rosrun joy joy_node



Terminal 2:

$ rosrun pirosbot joy_controller 


