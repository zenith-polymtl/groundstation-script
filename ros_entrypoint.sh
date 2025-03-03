#!/bin/bash
set -e

# Source ROS2 setup files
source /opt/ros/humble/setup.bash

# Clone the repository if it doesn't exist or is empty
if [ ! -d "/ros2_ws/.git" ]; then
  echo "Cloning repository..."
  rm -rf /ros2_ws/* /ros2_ws/.[!.]*
  git clone https://github.com/zenith-polymtl/ros2-mission-2 /ros2_ws
  cd /ros2_ws
  pip3 install -r requirements.txt
fi

# Execute the command passed to this script
exec "$@"
