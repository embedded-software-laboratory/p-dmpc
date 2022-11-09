#!/bin/bash  

vehicle_id=$1
force_feedback=$2

cd /usr/local/MATLAB/R2022a/bin
./matlab -sd /home/david/dev/software/high_level_controller/graph_based_planning -r "expert_mode($vehicle_id, $force_feedback)"
