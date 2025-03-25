#!/bin/bash
# Start Xvfb
Xvfb :99 -screen 0 1920x1080x24 &
sleep 2

# Set display
export DISPLAY=:99

# Start a minimal window manager
fluxbox -no-slit -no-toolbar &
sleep 1

# Source ROS environment 
source /opt/ros/humble/setup.bash

# Clone and build repository
if [ ! -d "/ros2_ws/.git" ]; then
  echo "Cloning repository..."
  rm -rf /ros2_ws/* /ros2_ws/.[!.]*
  git clone https://github.com/zenith-polymtl/ros2-mission-2 /ros2_ws
  cd /ros2_ws
  pip3 install -r requirements.txt
else
  cd /ros2_ws
  git pull
fi

# Build and run
cd /ros2_ws
colcon build --packages-up-to mission
source /ros2_ws/install/setup.bash 2>/dev/null || true

source ROS_DOMAIN_ID=1
source ROS_LOCALHOST_ONLY=0
source RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source CYCLONEDDS_URI=file:///connections/cycloneDDS_profile.xml
source VNC_PASSWD=none
source QT_QPA_PLATFORM=vnc

# Set Qt to use VNC platform but not bind to specific port
# Let it choose a different port than 5900
export QT_QPA_PLATFORM=vnc:size=1920x1080:port=5901

# Launch application
echo "Starting mission control application..."
exec ros2 run mission control
