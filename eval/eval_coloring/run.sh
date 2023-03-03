#!/usr/bin/env bash
# This is the master script to reproduce the results of the paper
# execute from the root directory of the repository
matlab -nodisplay -r "startup(); eval_coloring_paper(); quit()"