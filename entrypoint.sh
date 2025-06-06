#!/bin/bash
set -e

# Source ROS and workspace
source /opt/ros/humble/setup.bash
source /src/install/setup.bash

# Launch basic ROS2 node with write thread disabled
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ah_ros_py ah_node_launch.py write_thread:=False

exec "$@"
