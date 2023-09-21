function main_eval(scenario_version, eval_option, varargin)
filepath = pwd;
cd ../..;
startup();

if scenario_version == "C35"
    load(fullfile(filepath, 'C35.mat'), 'scenario');
elseif scenario_version == "C25"
	load(fullfile(filepath, 'C25.mat'), 'scenario');
elseif scenario_version == "C15"
	load(fullfile(filepath, 'C15.mat'), 'scenario');
elseif scenario_version == "C45"
	load(fullfile(filepath, 'C45.mat'), 'scenario');
elseif scenario_version == "C55"
	load(fullfile(filepath, 'C55.mat'), 'scenario');
elseif scenario_version == "Com55"
	load(fullfile(filepath, 'Com55.mat'), 'scenario');
elseif scenario_version == "testing_sceanrio1"
	load(fullfile(filepath, 'testing_sceanrio1.mat'), 'scenario');
elseif scenario_version == "testing_sceanrio2"
	load(fullfile(filepath, 'testing_sceanrio2.mat'), 'scenario');
elseif scenario_version == "testing_sceanrio3"
	load(fullfile(filepath, 'testing_sceanrio3.mat'), 'scenario');
end

if eval_option == "parallelization"
    scenario.options.cpp_implementation = Function.CentralizedOptimalAStarParallelization;
elseif eval_option == "leaf_parallelization"
	scenario.options.cpp_implementation = Function.CentralizedOptimalNodeParallelization;
elseif eval_option == "normal"
	scenario.options.cpp_implementation = Function.CentralizedOptimalPolymorphic;
elseif eval_option == "monte_carlo_settings"
	scenario.options.cpp_implementation = Function.CentralizedNaiveMonteCarloPolymorphicParallel;
elseif eval_option == "cbs"
	scenario.options.cpp_implementation = Function.CentralizedConflictBased;
elseif eval_option == "normal_trajectory"
	scenario.options.cpp_implementation = Function.CentralizedGrouping;
elseif eval_option == "monte_carlo_trajectory"
	scenario.options.cpp_implementation = Function.CentralizedNaiveMonteCarloPolymorphicParallel;
elseif eval_option == "cbs_trajectory"
	scenario.options.cpp_implementation = Function.CentralizedConflictBased;
end

plant = PlantFactory.get_experiment_interface(scenario.options.environment);

hlc_factory = HLCFactory();
hlc_factory.set_scenario(scenario);
dry_run = (scenario.options.environment == Environment.CpmLab);
hlc = hlc_factory.get_hlc(scenario.options.veh_ids, dry_run, plant);
pause(1);
[result, scenario] = hlc.run();

addpath('eval/eval_lukas');
pause(7);

filesList = dir;
names = regexpi({filesList.name}, 'Evaluation[0-9]+', 'match');
names = names(~cellfun('isempty', names));
names = [names{:}];
names = convertCharsToStrings(names);
names = sort(names);
name = names(end);

text = fileread(name);
data = jsondecode(text);
data = data(2:end);

cd 'eval/eval_lukas';
if eval_option == "parallelization"
    eval_parallelization;
elseif eval_option == "leaf_parallelization"
	eval_leaf_parallelization;
elseif eval_option == "normal"
	eval_normal;
elseif eval_option == "monte_carlo_settings"
	eval_monte_carlo_settings;
elseif eval_option == "cbs"
	eval_cbs;
elseif eval_option == "normal_trajectory"
	eval_trajectory;
elseif eval_option == "monte_carlo_trajectory"
	eval_trajectory;
elseif eval_option == "cbs_trajectory"
	eval_trajectory;
end

quit;