#!/bin/bash

cd ~/2019Offseason/zebROS_ws/

if [ -z $ROS_ROOT ]; then
	source /opt/ros/melodic/setup.bash
	if [ ! -z devel/setup.bash ]; then
		source devel/setup.bash
	fi
elif [[ ! $ROS_ROOT = "/opt/ros/melodic/share/ros" ]]; then
	echo "ROS is not configured for a native build (maybe set up for a cross build instead?)"
	echo "Run ./native_build.sh in a new terminal window"
	exit 1
fi

catkin config --blacklist \
	zed_ar_track_alvar_example \
	zed_display_rviz \
	zed_depth_sub_tutorial \
	zed_nodelet_example \
	zed_ros \
	zed_rtab_map_example \
	zed_tracking_sub_tutorial \
	zed_video_sub_tutorial

catkin build -DCATKIN_ENABLE_TESTING=OFF -DBUILD_WITH_OPENMP=ON "$@"
