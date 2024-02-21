#!/bin/bash
source nuc_control/nuc-password.sh
source nuc_control/working-nucs.sh

# get numbers of nucs
N_NUCS=${#WORKING_NUCS[@]}

for (( i=0; i<$N_NUCS; i++ )); do
    sshpass -e ssh guest@192.168.1.2${WORKING_NUCS[$i]} 'sudo poweroff'
done
