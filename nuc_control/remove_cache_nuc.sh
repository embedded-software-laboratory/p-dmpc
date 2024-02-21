#!/bin/bash
source nuc_control/nuc-password.sh


cat nuc_control/working-nucs | xargs --max-args=1 --max-procs=20 -i \
    sshpass -e ssh guest@{} 'rm -r /home/guest/dev/software/high_level_controller/graph_based_planning/hlc/model/motion_primitive_automaton/library; rm -r /home/guest/dev/software/high_level_controller/graph_based_planning/hlc/communication/cust1/matlab_msg_gen'