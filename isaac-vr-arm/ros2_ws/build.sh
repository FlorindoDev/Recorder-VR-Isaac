#!/bin/bash
colcon build --packages-select isaac_arm_ml_control
source /opt/ros/humble/setup.sh
source ./install/setup.sh
