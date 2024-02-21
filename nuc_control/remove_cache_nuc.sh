#!/bin/bash
source nuc_control/nuc-password.sh
source nuc_control/working-nucs.sh

# get numbers of nucs
N_NUCS=${#WORKING_NUCS[@]}

for (( i=0; i<$N_NUCS; i++ )); do
    sshpass -e ssh guest@192.168.1.2${WORKING_NUCS[$i]} 'rm -r /home/guest/dev/software/high_level_controller/graph_based_planning/hlc/model/motion_primitive_automaton/library; rm -r /home/guest/dev/software/high_level_controller/graph_based_planning/hlc/communication/cust1/matlab_msg_gen'
done
