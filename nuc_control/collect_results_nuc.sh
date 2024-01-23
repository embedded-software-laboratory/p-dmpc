#!/bin/bash
# guest password (DO NOT PUSH TO GIT)
export SSHPASS=
dirname="eval_files_$(date +%Y%m%d%H%M%S)"
mkdir /tmp/$dirname

#Get command line arguments
VEHICLE_IDS=("$@")

# list of working nucs to be used for execution
WORKING_NUCS=(01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20)

# get numbers of vehicles
N_VEH=${#VEHICLE_IDS[@]}

# copy result from every hlc and delete directly to avoid two results
for (( i=0; i<$N_VEH; i++ )); do
    sshpass -e scp -r guest@192.168.1.2${WORKING_NUCS[$i]}:/home/guest/dev/software/high_level_controller/graph_based_planning/results/*/* /tmp/$dirname/result_$(printf "%02d" $i).mat
    sshpass -e ssh guest@192.168.1.2${WORKING_NUCS[$i]} 'rm -fr /home/guest/dev/software/high_level_controller/graph_based_planning/results/*'
done
