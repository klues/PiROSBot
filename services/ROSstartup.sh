#! /bin/bash
#prevent duplicated start of script
if ! mkdir /tmp/rosstart.lock 2>/dev/null; then
    echo "startup script already running" >&2
    exit 0
fi

echo "waiting for system startup..."
sleep 10

# wait for network connection (wifi) before setting ROS IP
#echo "waiting for wifi connection ... " 
#while ! ping -c 1 -W 1 192.168.178.32 ; do
#	count=$((count+1))
#	echo $count
#	sleep 1
#done

xfce4-terminal \
	-H -T roscore -e "bash /home/ros/services/roscore.sh" &

echo "waiting for startup of roscore..."
sleep 10

xfce4-terminal \
	-H -T PiROSBot-Motors -e "bash /home/ros/services/PiROSBotMot.sh" &

echo "waiting for startup of PiROSBot Motor nodes..."
sleep 10

xfce4-terminal \
	-H -T PiROSBot-Cam -e "bash /home/ros/services/PiROSBotCam.sh"

rm -rf /tmp/rosstart.lock
read -p "Press any key to continue... " -n1 -s



