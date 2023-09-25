#!/bin/bash

compile () {
	echo -e "#pragma once\n#define DO_EVAL 1\nconstexpr unsigned int Threads = $1;\nconstexpr unsigned int Experiments = $2;" > ../../hlc/optimizer/graph_search_cpp/config.h
  LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6 matlab -nodesktop -r "compile_eval";
}

do_three() {
	LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6 matlab -nodesktop -r "main_eval('testing_sceanrio3', 'monte_carlo_trajectory', ${1})";
	LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6 matlab -nodesktop -r "main_eval('testing_sceanrio3', 'monte_carlo_trajectory', ${1})";
	LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6 matlab -nodesktop -r "main_eval('testing_sceanrio3', 'monte_carlo_trajectory', ${1})";
}

for experiments in 200000 500000 1000000 1500000 2000000 3000000 4000000
do
  compile 12 $experiments
  do_three $experiments
done