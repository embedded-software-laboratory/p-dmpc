#!/bin/bash
source nuc_control/nuc-password.sh

#Get command line arguments
VEHICLE_IDS=("$@")

# list of working nucs to be used for execution
WORKING_NUCS=(01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20)

# get numbers of vehicles
N_VEH=${#VEHICLE_IDS[@]}
export LCC_BASH_DIR="/home/cpm/dev/software/lab_control_center/bash/"
# start hlc and middleware non blocking
for (( i=0; i<$N_VEH; i++ )); do
    VEH_ID=${VEHICLE_IDS[$i]}
    middleware_id=$(printf "middleware_${WORKING_NUCS[$i]}")

    IP=192.168.1.2${WORKING_NUCS[$i]}

    sshpass -e ssh -t guest@${IP} 'bash /tmp/scripts/remote_start.bash' "--script_path=/home/guest/dev/software/high_level_controller/graph_based_planning/main_distributed.m --script_arguments='$((i + 1))' --middleware_arguments='--node_id=${middleware_id} --vehicle_ids=${VEH_ID} --dds_domain=21 --simulated_time=0 --dds_initial_peer=rtps@udpv4://192.168.1.249:25598'"


done
