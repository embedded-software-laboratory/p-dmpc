function Get-run {
	param (
		$scenario_version,
		$eval_option,
		$threads,
		$experiments
	)

	Start-Process matlab -ArgumentList "-nodesktop -r \""main_eval('$scenario_version', '$eval_option', $threads, $experiments)\""" -Wait
}

function Get-do_three {
	param (
		$scenario_version,
		$eval_option,
		$threads,
		$experiments
	)

	for($i = 0; $i -lt 3; $i++){ Get-run $scenario_version $eval_option $threads $experiments }
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

$experiments_list = @(20000, 200000, 2000000)
foreach ($experiments in $experiments_list) {
	$threads_list = @(2, 3, 4, 5, 6, 7, 8) # $threads_list = @(1, 2, 3, 4, 5, 6, 7, 8) MVSC lässt leider keine arrays mit Größe 0 zu, aber gcc.
	foreach ($threads in $threads_list) {
		Get-compile $threads $experiments 1
		Get-do_three C35 monte_carlo_settings $threads $experiments
	}
}