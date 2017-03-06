#! /bin/bash


# wait for network connection (wifi) before setting ROS IP
echo "waiting for wifi connection ... " 
#while ! ping -c 1 -W 1 192.168.0.50 ; do
#	count=$((count+1))
#	echo $count
#	sleep 1
#done


xfce4-terminal \
	-T PiROSBot-Motors -e ./PiROSBotMot.sh \
	--tab -T PiROSBot-Camera -e ./PiROSBotCam.sh 
