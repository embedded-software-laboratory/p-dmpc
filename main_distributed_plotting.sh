#!/bin/bash
# guest password (DO NOT PUSH TO GIT)
export SSHPASS=

# Get directory of the script (use before first use of cd)
SCRIPT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/"
echo ${SCRIPT_PATH}
cd ..

#Get command line arguments
VEHICLE_IDS=("$@")

echo ${VEHICLE_IDS[@]}

# list of working nucs to be used for execution
WORKING_NUCS=(01 02 03 05 07 08 10 11 12 13 14 16 19)

# get numbers of vehicles
N_VEH=${#VEHICLE_IDS[@]}

# copy scripts to nucs
for (( i=0; i<$N_VEH; i++ )); do
    echo ${WORKING_NUCS[$i]}
    #sshpass -e scp -r ./graph_based_planning guest@192.168.1.2${WORKING_NUCS[$i]}:/home/guest/dev/software/high_level_controller/
    sshpass -e rsync -r ./graph_based_planning guest@192.168.1.2${WORKING_NUCS[$i]}:/home/guest/dev/software/high_level_controller/
done

cd ${SCRIPT_PATH}

# start script non blocking
for (( i=0; i<$N_VEH; i++ )); do
    VEH_ID=${VEHICLE_IDS[$i]}
    sshpass -e ssh -t guest@192.168.1.2${WORKING_NUCS[$i]} 'nohup /usr/local/MATLAB/R2023a/bin/matlab' "-logfile matlab.log -sd '/home/guest/dev/software/high_level_controller/graph_based_planning' -batch 'main_distributed($VEH_ID); quit();'" "> /dev/null 2>&1 < /dev/null &"
done
