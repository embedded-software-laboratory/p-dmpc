#!/bin/bash  

controller_amount=$1
force_feedback_enabled=$2
force_feedback_dir=~/Documents/ros2_ws;

# source ros
cd launch
source /opt/ros/eloquent/setup.bash 

if [ $controller_amount -eq 1 ]; then
    # only launch steering wheel
    ros2 launch single_controller_launch.xml &
else
    # launch steering wheel and gamepad
    ros2 launch dual_controller_launch.xml &
fi

if [ $force_feedback_enabled -eq 1 ]; then
    echo $force_feedback_enabled
    cd $force_feedback_dir
    source /opt/ros/eloquent/setup.bash
    source $force_feedback_dir/install/setup.bash
    ros2 run ros_g29_force_feedback  g29_force_feedback --ros-args --params-file $force_feedback_dir/install/ros_g29_force_feedback/share/ros_g29_force_feedback/config/g29.yaml && fg
    # if the line above causes an error, run cat /proc/bus/input/devices, and check if the event handler for the steering wheel
    # is equal to the value in $force_feedback_dir/install/ros_g29_force_feedback/share/ros_g29_force_feedback/config/g29.yaml
fi

