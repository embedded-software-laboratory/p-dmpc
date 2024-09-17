#!/bin/bash
source nuc_control/nuc-password.sh

# stop tmux sessions on all used hlcs
cat nuc_control/working-nucs | xargs --max-args=1 --max-procs=20 -i \
    sshpass -e ssh -t guest@{} 'tmux new-session -d -s "lab_update" "bash /home/guest/autostart/lab_update.bash"'
