#!/bin/bash
export SSHPASS=

# list of working nucs to be used for execution
WORKING_NUCS=(01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20)

# get numbers of nucs
N_NUCS=${#WORKING_NUCS[@]}

for (( i=0; i<$N_NUCS; i++ )); do
    sshpass -e ssh guest@192.168.1.2${WORKING_NUCS[$i]} 'sudo poweroff'
done
