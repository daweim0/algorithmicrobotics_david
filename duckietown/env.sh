#!/usr/bin/env bash

source /opt/ros/kinetic/setup.bash
source ~/ws/devel/setup.bash

export ROS_IP=`ifconfig wlan0 | grep 'inet addr' | cut -d: -f2 | awk '{print $1}'`
export ROS_MASTER_URI=duckie.local:11311

exec "$@"
