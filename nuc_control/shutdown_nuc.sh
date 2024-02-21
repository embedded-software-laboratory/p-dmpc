#!/bin/bash
source nuc_control/nuc-password.sh

cat nuc_control/working-nucs | xargs --max-args=1 --max-procs=20 -i \
    sshpass -e ssh guest@{} 'sudo poweroff'
