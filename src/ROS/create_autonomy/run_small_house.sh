catkin_make
source devel/setup.bash
export LOCALIZATION=hector_mapping
export RVIZ=true
export LASER=rplidar
roslaunch -v ca_gazebo create_small_house.launch world_name:=/home/create/.gazebo/worlds/small_house_clean.world

