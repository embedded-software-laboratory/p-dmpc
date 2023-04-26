#!/bin/bash
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# source ros
source /opt/ros/eloquent/setup.bash

# launch ros2node
ros2 launch ${SCRIPT_DIR}/launch_j0.xml