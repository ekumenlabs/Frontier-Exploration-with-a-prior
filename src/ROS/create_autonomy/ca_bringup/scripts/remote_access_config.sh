#!/bin/bash

# usage: source remote_access_config.sh ROBOT_IP

if [[ -z $1 ]]; then
	echo "usage: source remote_access_config.sh ROBOT_IP"
	return
fi

COMPUTER_IP=`hostname -I | cut --delimiter " " --fields 1`
ROBOT_IP=$1

echo "Configuring ROS Networking on a remote machine [IP: $COMPUTER_IP]"
# IP of the robot
export ROS_MASTER_URI=http://$ROBOT_IP:11311
# Remote machine's IP
export ROS_IP=$COMPUTER_IP
export ROS_HOSTNAME=$COMPUTER_IP

echo "ROS remote machine configured!"
