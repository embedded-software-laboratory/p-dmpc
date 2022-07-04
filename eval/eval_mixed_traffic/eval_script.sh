#!/bin/bash  

seed=$1

# source ros
source /opt/ros/eloquent/setup.bash

# create publisher for both manual devices
# ros2 topic pub /j0 sensor_msgs/Joy "axes: [0.5,0.0,0.0,0.0,0.0,0.0], buttons: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]"
# TODO: use seed for rosrate
ros2 topic pub /j0 sensor_msgs/Joy "axes: [0.5,0.0,0.0,0.0,0.0,0.0]"