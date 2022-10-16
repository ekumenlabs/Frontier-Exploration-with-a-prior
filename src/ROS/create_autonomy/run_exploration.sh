#!/bin/bash
if [ "$#" != "4" ]; then
    echo "You must provide three args; START_X, START_Y, WORLD and USE_VISIBILITY"
    echo "EXPLORATION_TYPE must be 'utn' or maryland."
    exit -1
else
    echo "Welcome!"
    echo "Starting to map with start_x: $1, start_y: $2, world: $3 , use_visibility: $4"
fi

cd /create_ws/Frontier-Exploration-with-a-prior && python3.8 -m pip install -e .
cd /create_ws && catkin_make
source devel/setup.bash
export LOCALIZATION=slam
export RVIZ=true
export GUI=true
export LASER=rplidar
export START_X=$1
export START_Y=$2
export WORLD_NAME=/home/create/.gazebo/worlds/$3.world
export POLYGON_PATH=/home/create/.gazebo/polygons/$3_polygon.pkl
export VISIBILITY=$4
export PLOT=true
OUTPUTS_BASE_DIR=/create_ws/src/outputs
if [ $VISIBILITY == 'true' ]; then
    OUTPUT_DIR="$OUTPUTS_BASE_DIR/$3/UTN"
else
    OUTPUT_DIR="$OUTPUTS_BASE_DIR/$3/MARYLAND"
fi

echo "Running $WORLD_NAME, loading polygon from $POLYGON_PATH"
mkdir -p $OUTPUT_DIR && cd $OUTPUT_DIR && rosbag record /create1/explored /create1/travelled_distance&
roslaunch blueprint_explore run_everything.launch
rosbag reindex $OUTPUT_DIR/*
