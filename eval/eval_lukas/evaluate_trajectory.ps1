function Get-run {
	param (
		$scenario_version,
		$eval_option,
		$experiments
	)

	Start-Process matlab -ArgumentList "-nodesktop -r \""main_eval('$scenario_version', '$eval_option', $experiments)\""" -Wait
}

function Get-do_three {
	param (
		$scenario_version,
		$eval_option,
		$experiments
	)

	for($i = 0; $i -lt 3; $i++){ Get-run $scenario_version $eval_option $experiments }
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

$scenario_list = @("testing_sceanrio1", "testing_sceanrio2", "testing_sceanrio3")
foreach ($scenario in $scenario_list) {
	$experiments_list = @(200000, 2000000)
	foreach ($experiments in $experiments_list) {
		Get-compile 12 $experiments 1
		Get-do_three $scenario monte_carlo_trajectory $experiments
	}
	Get-do_three $scenario cbs_trajectory 1
	Get-do_three $scenario normal_trajectory 1
}