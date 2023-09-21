#!/bin/bash

compile () {
	echo -e "#pragma once\n#define DO_EVAL 1\nconstexpr unsigned int Threads = $1;\nconstexpr unsigned int Experiments = $2;" > ../../hlc/optimizer/graph_search_cpp/config.h
  LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6 matlab -nodesktop -r "compile_eval";
}

do_three() {
	LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6 matlab -nodesktop -r "main_eval(${1}, ${2}, ${3})";
	LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6 matlab -nodesktop -r "main_eval(${1}, ${2}, ${3})";
	LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6 matlab -nodesktop -r "main_eval(${1}, ${2}, ${3})";
}

for scenario in \'testing_sceanrio1\' \'testing_sceanrio2\' \'testing_sceanrio3\'
do
	for experiments in 200000 2000000
	do
		compile 12 $experiments
		do_three $scenario \'monte_carlo_trajectory\' $experiments
	done
	do_three $scenario \'cbs_trajectory\' 1
	do_three $scenario \'normal_trajectory\' 1
done