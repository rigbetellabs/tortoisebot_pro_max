#!/bin/bash

source /opt/ros/galactic/setup.bash
source /home/ttb-promax/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=169

#if [ $? -ne 0 ]; then
#    echo "Failed to source ROS 2 setup.bash"
#    exit 1
#fi

sleep 5
ros2 launch tortoisebotpromax_bringup autobringup.launch.py

#if [ $? -ne 0 ]; then
#    echo "Failed to launch autobringup.launch.py"
#    exit 1
#fi


