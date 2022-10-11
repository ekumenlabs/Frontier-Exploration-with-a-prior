#!/bin/bash
if [ "$#" != "4" ]; then
    echo "You must provide three args; START_X, START_Y, WORLD and USE_VISIBILITY"
    echo "EXPLORATION_TYPE must be 'utn' or maryland."
    exit -1
else
    echo "Welcome!"
    echo "Starting to map with start_x: $1, start_y: $2, world: $3 , use_visibility: $4"
fi

cd /create_ws && catkin_make
source devel/setup.bash
export LOCALIZATION=slam
export RVIZ=false
export GUI=false
export LASER=rplidar
export EXPLORATION_TYPE=utn
export START_X=$1
export START_Y=$2
export WORLD_NAME=/home/create/.gazebo/worlds/$3.world
export VISIBILITY=$4
export PLOT=false

roslaunch blueprint_explore run_everything.launch
