# README #

## ROSstartup ##
Script waits for network connection to router with IP adr. 192.168.0.50 (disabled) and starts the ROS hardware driver nodes.

##### SETUP #####
Applications -> Settings -> Settings Manager -> Session and Startup -> Application Autostart
-> "+Add"
Name: ROS_RPi_Driver
Command: xfce4-terminal -x /home/torades/ROSstartup.sh 


## off_button ##
Script runs in background and waits for a falling edge on GPIO pin 17 (BCM numbering). If Button is pressed, the Raspberry Pi shuts down.
Important: use external pull-up resistor because internal is only active after script start!

##### SETUP #####

sudo nano /etc/rc.local

Enter following line before 'exit 0':
python /home/torades/services/off_button.py &
It is important to add the "&" at the end of the line!


Maybe it is necessary to update or install python as well as RPi.GPIO:
sudo apt-get update
sudo apt-get install python-dev
sudo apt-get install python-pip
sudo pip install -U RPi.GPIO


ref: http://www.instructables.com/id/Simple-Raspberry-Pi-Shutdown-Button/?ALLSTEPS