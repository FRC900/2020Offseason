#!/usr/bin/env bash

# Setup ROS for Local Development
source /opt/ros/melodic/setup.bash
source ~/2020RobotCode/zebROS_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:5802
export ROS_IP=`ip route get 127.0.0.1 | sed 's/ via [[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+//' | sed 's/lo //' | head -1 | cut -d ' ' -f 5`
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

exec "$@"
