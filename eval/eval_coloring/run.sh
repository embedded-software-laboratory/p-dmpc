#!/usr/bin/env bash
# This is the master script to reproduce the results of the paper
# execute from the root directory of the repository
matlab -nodisplay -r "openProject('graph_based_planning.prj'); eval_coloring_paper(); quit()"