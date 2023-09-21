function Get-run {
	param (
		$scenario_version,
		$eval_option,
		$threads
	)

	Start-Process matlab -ArgumentList "-nodesktop -r \""main_eval('$scenario_version', '$eval_option', $threads)\""" -Wait
}

function Get-do_three {
	param (
		$scenario_version,
		$eval_option,
		$threads
	)

	for($i = 0; $i -lt 3; $i++){ Get-run $scenario_version $eval_option $threads }
}

function Get-compile {
	param (
		$threads,
		$experiments,
		$DO_EVAL
	)

	$file = "../../hlc/optimizer/graph_search_cpp/config.h"
	Set-Content $file ("#pragma once" + [Environment]::NewLine + "#define DO_EVAL $DO_EVAL" + [Environment]::NewLine + "constexpr unsigned int Threads = $threads;" + [Environment]::NewLine + "constexpr unsigned int Experiments = $experiments;")

	Start-Process matlab -ArgumentList "-nodesktop -r \""compile_eval\""" -Wait
}

# Start
Set-Location -Path $PSScriptRoot

$threads_list = @(2, 3, 4, 5, 6, 7, 8) # $threads_list = @(1, 2, 3, 4, 5, 6, 7, 8) MVSC lässt leider keine arrays mit Größe 0 zu, aber gcc.
foreach ($threads in $threads_list) {
	Get-compile $threads 200000 1
	Get-do_three C35 leaf_parallelization $threads
	Get-do_three C35 parallelization $threads
}