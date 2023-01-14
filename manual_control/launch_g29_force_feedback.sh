#!/bin/bash
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
force_feedback_dir="${SCRIPT_DIR}/ros2_ws"

source /opt/ros/eloquent/setup.bash
source $force_feedback_dir/install/setup.bash
ros2 launch ${SCRIPT_DIR}/launch_g29_force_feedback.xml