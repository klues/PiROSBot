# ROS NOde for PiROSBot

To avoid running out of memory during the build process on Raspberry Pi 3, execute catkin_make with the option -j1 or --jobs=1.
e.g., $catkin_make -j1

uses https://github.com/joan2937/pigpio to control GPIOs (included in this repo - no submodule)