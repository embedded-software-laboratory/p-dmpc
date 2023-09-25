function Get-run {
	param (
		$scenario_version,
		$eval_option
	)

	Start-Process matlab -ArgumentList "-nodesktop -r \""main_eval('$scenario_version', '$eval_option')\""" -Wait
}

function Get-do_three {
	param (
		$scenario_version,
		$eval_option
	)

	for($i = 0; $i -lt 3; $i++){ Get-run $scenario_version $eval_option }
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
Get-compile 2 200000 1

$scenario_list = @("C15", "C25", "C35")
foreach ($scenario in $scenario_list) { Get-do_three $scenario normal }
