#!/bin/bash

# Exit on error
set -e

echo "📦 Installing Cyclone DDS RMW implementation for ROS 2 Humble..."
sudo apt update
sudo apt install -y ros-humble-rmw-cyclonedds-cpp

# Get the directory where the script is run from
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
CYCLONE_PATH="file://$SCRIPT_DIR/cyclonedds.xml"

# Append environment variables to .bashrc
BASHRC="$HOME/.bashrc"
{
    echo ""
    echo "# Added by ability-hand-ros2/cyclone_dds_install.sh"
    echo "export ROS_DOMAIN_ID=0"
    echo "export ROS_LOCALHOST_ONLY=0"
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    echo "export CYCLONEDDS_URI=$CYCLONE_PATH"
} >> "$BASHRC"

echo "✅ Environment variables added to $BASHRC"
echo "🔁 Run 'source ~/.bashrc' or open a new terminal to apply changes."
