#!/bin/bash
set -e

# Setup ROS2 environment
source /opt/ros/humble/setup.bash

exec "$@"
