#!/bin/bash
compile () {
	echo -e "#pragma once\n#define DO_EVAL 1\nconstexpr unsigned int Threads = $1;\nconstexpr unsigned int Experiments = $2;" > ../../hlc/optimizer/graph_search_cpp/config.h
  LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6 matlab -nodesktop -r "compile_eval";
}

do_three() {
	LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6 matlab -nodesktop -r "main_eval('C35', ${3}, ${1}, ${2})";
	LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6 matlab -nodesktop -r "main_eval('C35', ${3}, ${1}, ${2})";
	LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6 matlab -nodesktop -r "main_eval('C35', ${3}, ${1}, ${2})";
}

for threads in 1 2 3 4 5 6 7 8
do
	compile $threads 2000000
	do_three $threads 2000000 \'leaf_parallelization\'
	do_three $threads 2000000 \'parallelization\'
done
