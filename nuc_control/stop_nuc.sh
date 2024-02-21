#!/bin/bash
source nuc_control/nuc-password.sh
source nuc_control/working-nucs.sh

N_NUCS=${#WORKING_NUCS[@]}

# stop tmux sessions on all used hlcs
for (( i=0; i<$N_NUCS; i++ )); do
    sshpass -e ssh -t guest@192.168.1.2${WORKING_NUCS[$i]} 'tmux kill-session -t "hlc"'
done
