#!/bin/bash

force_feedback_dir=~/Documents/ros2_ws;

cd $force_feedback_dir
source /opt/ros/eloquent/setup.bash
source $force_feedback_dir/install/setup.bash
ros2 run ros_g29_force_feedback  g29_force_feedback --ros-args --params-file $force_feedback_dir/install/ros_g29_force_feedback/share/ros_g29_force_feedback/config/g29.yaml && fg