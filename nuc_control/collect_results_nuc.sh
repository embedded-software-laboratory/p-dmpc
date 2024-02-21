#!/bin/bash
source nuc_control/nuc-password.sh
source nuc_control/working-nucs.sh

dirname="eval_files_$(date +%Y%m%d%H%M%S)"
mkdir /tmp/$dirname

#Get command line arguments
VEHICLE_IDS=("$@")


# get numbers of vehicles
N_VEH=${#VEHICLE_IDS[@]}

# copy result from every hlc and delete directly to avoid two results
for (( i=0; i<$N_VEH; i++ )); do
    sshpass -e scp -r guest@192.168.1.2${WORKING_NUCS[$i]}:/home/guest/dev/software/high_level_controller/graph_based_planning/results/*/* /tmp/$dirname/result_$(printf "%02d" $i).mat
    sshpass -e ssh guest@192.168.1.2${WORKING_NUCS[$i]} 'rm -fr /home/guest/dev/software/high_level_controller/graph_based_planning/results/*'
done
