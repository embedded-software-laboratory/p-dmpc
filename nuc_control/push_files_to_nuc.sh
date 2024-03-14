#!/bin/bash
source nuc_control/nuc-password.sh

# LCC directory to reuse lcc bash scripts
LCC_BASH_DIR="/home/cpm/dev/software/lab_control_center/bash/"

# remove old logs and scripts
# kill resource hungry lab autostart used by LCC
# remove previous results
cat nuc_control/working-nucs | xargs --max-args=1 --max-procs=20 -i \
    sshpass -e ssh -t guest@{} 'rm -rf ~/dev/lcc_script_logs; mkdir -p ~/dev/lcc_script_logs; rm -rf /tmp/scripts; mkdir -p /tmp/scripts; pkill autostart; rm -rf /home/guest/dev/software/high_level_controller/graph_based_planning/results/*'
# sync files
cat nuc_control/working-nucs | xargs --max-args=1 --max-procs=20 -i \
    sshpass -e rsync -r -u \
    --exclude 'hlc/communication/cust1/matlab_msg_gen' \
    --exclude 'hlc/optimizer/graph_search_cpp' \
    --exclude 'results*' \
    --exclude '.git' \
    --exclude 'derived' \
    ../graph_based_planning \
    guest@{}:/home/guest/dev/software/high_level_controller/
# Copy scripts to the NUC
cat nuc_control/working-nucs | xargs --max-args=1 --max-procs=20 -i \
    sshpass -e rsync -u \
    ${LCC_BASH_DIR}remote_start.bash \
    ${LCC_BASH_DIR}environment_variables.bash \
    ${LCC_BASH_DIR}tmux_middleware.bash \
    ${LCC_BASH_DIR}tmux_script.bash \
    guest@{}:/tmp/scripts