#!/bin/bash

echo "Activating ROS..."
source /opt/ros/melodic/setup.bash
echo "...done."


export workspace=$HOME/jetfalcon
echo "Setting up PYTHONPATH."
export PYTHONPATH=$workspace/catkin_ws/src:$PYTHONPATH

echo "Setup ROS_HOSTNAME."
current_ip=$(hostname -I | awk '{print $1}')
export ROS_HOSTNAME=$current_ip


source $workspace/catkin_ws/devel/setup.bash
source $workspace/setup/set_ros_master.sh
source $workspace/setup/set_vehicle_name.sh $HOSTNAME


# TODO: check that the time is >= 2015

# TODO: run a python script that checks all libraries are installed

exec "$@" #Passes arguments. Need this for ROS remote launching to work.
