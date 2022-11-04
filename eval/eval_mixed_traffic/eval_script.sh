#!/bin/bash  

# source ros
source /opt/ros/eloquent/setup.bash

cd /usr/local/MATLAB/R2022a/bin
./matlab -sd /home/david/dev/software/high_level_controller/graph_based_planning -r eval_script