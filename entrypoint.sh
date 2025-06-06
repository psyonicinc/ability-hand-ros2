#!/bin/bash
set -e

# Source ROS and workspace
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

exec "$@"