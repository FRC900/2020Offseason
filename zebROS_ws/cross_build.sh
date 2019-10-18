#!/bin/bash

cd ~/2019Offseason/zebROS_ws/

if [ -z $ROS_ROOT ]; then
	PATH=$PATH:$HOME/wpilib/2020/roborio/bin
	source ~/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/opt/ros/melodic/setup.bash
elif [[ ! $ROS_ROOT = "$HOME/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/opt/ros/melodic/share/ros" ]]; then
	echo "ROS is not configured for a cross build (maybe set up for a native build instead?)"
	echo "Run ./cross_build.sh in a new terminal window"
	exit 1
fi

catkin_make_isolated --install --use-ninja -DCMAKE_TOOLCHAIN_FILE=`pwd`/rostoolchain.cmake -DCATKIN_ENABLE_TESTING=OFF "$@"
