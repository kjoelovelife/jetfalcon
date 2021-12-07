#!/usr/bin/env bash
echo "Setting ROS_MASTER_URI..."
current_ip=$(hostname -I | awk '{print $1}')
if [ $# -gt 0 ]; then
	# provided a hostname, use it as ROS_MASTER_URI
	export ROS_MASTER_URI=http://$1.local:11311/
else
	echo "No hostname provided. Using $HOSTNAME.local."
	export ROS_MASTER_URI=http://$current_ip:11311/
fi

echo "ROS_MASTER_URI set to $ROS_MASTER_URI"
