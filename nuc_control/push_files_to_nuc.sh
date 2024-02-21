#!/bin/bash
source nuc_control/nuc-password.sh
source nuc_control/working-nucs.sh

# Get directory of the script (use before first use of cd)
SCRIPT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/"
echo ${SCRIPT_PATH}
cd ..

# LCC directory to reuse lcc bash scripts
LCC_BASH_DIR="/home/cpm/dev/software/lab_control_center/bash/"

# get numbers of nucs
N_NUCS=${#WORKING_NUCS[@]}

for (( i=0; i<$N_NUCS; i++ )); do
    IP=192.168.1.2${WORKING_NUCS[$i]}
    # sync files
    sshpass -e rsync --progress -r -u \
        --exclude 'hlc/communication/cust1/matlab_msg_gen' \
        --exclude 'results' \
        --exclude '.git' \
        ./graph_based_planning \
        guest@${IP}:/home/guest/dev/software/high_level_controller/
    # remove old logs and scripts
    sshpass -e ssh -t guest@${IP} 'rm -rf ~/dev/lcc_script_logs; mkdir -p ~/dev/lcc_script_logs; rm -rf /tmp/scripts; mkdir -p /tmp/scripts'

    # kill resource hungry lab autostart used by LCC
    sshpass -e ssh -t guest@${IP} 'pkill autostart'

    # Copy scripts to the NUC
    sshpass -e rsync -u ${LCC_BASH_DIR}remote_start.bash guest@${IP}:/tmp/scripts
    sshpass -e rsync -u ${LCC_BASH_DIR}environment_variables.bash guest@${IP}:/tmp/scripts
    sshpass -e rsync -u ${LCC_BASH_DIR}tmux_middleware.bash guest@${IP}:/tmp/scripts
    sshpass -e rsync -u ${LCC_BASH_DIR}tmux_script.bash guest@${IP}:/tmp/scripts
done