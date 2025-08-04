#!/bin/bash
set -e

# Source ROS and workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
source /src/install/setup.bash

# Launch basic ROS2 node with write thread disabled
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Basic node
# ros2 launch ah_ros_py ah_node_launch.py write_thread:=False

# # Hand wave with URDF / RViz
# ros2 launch urdf_launch display.launch.py &
# ros2 launch ah_ros_py hand_wave.launch.py js_publisher:=True

# Simulated hand wave with URDF / RViz
ros2 launch urdf_launch display.launch.py &
ros2 launch ah_ros_py hand_wave.launch.py js_publisher:=True simulated_hand:=True

exec "$@"