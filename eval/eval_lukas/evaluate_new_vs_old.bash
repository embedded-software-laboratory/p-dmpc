#!/bin/bash

compile () {
	echo -e "#pragma once\n#define DO_EVAL 1\nconstexpr unsigned int Threads = $1;\nconstexpr unsigned int Experiments = $2;" > ../../hlc/optimizer/graph_search_cpp/config.h
  LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6 matlab -nodesktop -r "compile_eval";
}

do_three() {
	LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6 matlab -nodesktop -r "main_eval(${1}, ${2})";
	LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6 matlab -nodesktop -r "main_eval(${1}, ${2})";
	LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6 matlab -nodesktop -r "main_eval(${1}, ${2})";
}

compile 2 2000000

for scenario in \'C15\' \'C25\' \'C35\'
do
	do_three $scenario \'normal\'
done
