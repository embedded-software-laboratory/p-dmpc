#!/bin/bash  

#seed=$1
#value=$(bash -c 'RANDOM=$1; echo $[ $RANDOM % 10]')
#echo $value

# source ros
source /opt/ros/eloquent/setup.bash

cd /usr/local/MATLAB/R2022a/bin
./matlab -sd /home/david/dev/software/high_level_controller/graph_based_planning -r eval_script

# create publisher for both manual devices
# steering wheel 
#ros2 topic pub -r 5 /j0 sensor_msgs/Joy "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: }, axes: [0.0,0.0,0.0,0.0,0.0,0.0], buttons: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}" &
#ros2 topic pub -r 0.5 /j0 sensor_msgs/Joy "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: }, axes: [-0.5,0.0,0.0,0.0,0.0,0.0], buttons: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}" &

# gamepad
#ros2 topic pub -r 5 /j1 sensor_msgs/Joy "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: }, axes: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], buttons: [0,0,0,0,0,0,0,0,0,0,0]}" &
#ros2 topic pub -r 1 /j1 sensor_msgs/Joy "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: }, axes: [-0.5,0.0,0.0,0.0,0.0,0.0,0.0,0.0], buttons: [0,0,0,0,0,0,0,0,0,0,0]}" && fg