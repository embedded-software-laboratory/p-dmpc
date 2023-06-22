#!/bin/bash
# guest password (DO NOT PUSH TO GIT)
export SSHPASS=cpm

#Get command line arguments
VEHICLE_IDS=("$@")

# list of working nucs to be used for execution
WORKING_NUCS=(01 02 03 05 07 08 10 11 12 13 14 16 19)

# get numbers of vehicles
N_VEH=${#VEHICLE_IDS[@]}

# start script non blocking
for (( i=0; i<$N_VEH; i++ )); do
    VEH_ID=${VEHICLE_IDS[$i]}
    sshpass -e ssh -t guest@192.168.1.2${WORKING_NUCS[$i]} 'tmux kill-session -t "matlab-hlc"'
done
