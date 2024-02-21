#!/bin/bash
source nuc_control/nuc-password.sh

dirname="eval_files_$(date +%Y%m%d%H%M%S)"
mkdir /tmp/$dirname


# copy result from every hlc
cat nuc_control/working-nucs | xargs --max-args=1 --max-procs=20 -i \
    sshpass -e scp -r guest@{}:/home/guest/dev/software/high_level_controller/graph_based_planning/results/*/* /tmp/$dirname/
# delete directly to avoid two results
cat nuc_control/working-nucs | xargs --max-args=1 --max-procs=20 -i \
    sshpass -e ssh guest@{} 'rm -fr /home/guest/dev/software/high_level_controller/graph_based_planning/results/*'
