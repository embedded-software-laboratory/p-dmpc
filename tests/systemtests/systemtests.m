classdef systemtests < matlab.unittest.TestCase

    properties (TestParameter)
        scenario = {'circle', 'commonroad'};
        mpa = {'single_speed', 'triple_speed', 'realistic'};
        optimizer_centralized = {'MatlabOptimal', 'CppOptimal'}; % 'CppSampled' crashes matlab in tests
        optimizer_prioritized = {'MatlabOptimal', 'MatlabSampled', 'CppOptimal'};
        parallel = {'sequential', 'parallel'};
        coupling = {'reachable_set', 'full', 'no', 'distance'};
        priority = {'coloring', 'constant', 'random', 'FCA'};
        weight = {'constant', 'random', 'distance'};
    end

    methods (Test, ParameterCombination = 'pairwise')

        function test_centralized(testCase, scenario, optimizer_centralized)
            lastwarn('');
            fprintf('\ncentralized systemtest for %s\n', scenario);
            %load Config from json
            options = Config.load_from_file('tests/systemtests/Config_systemtests_centralized.json');
            options.scenario_type = ScenarioType(scenario);
            options.is_prioritized = false;
            options.optimizer_type = OptimizerType(optimizer_centralized);
            options.validate();

            testCase.verifyEmpty(lastwarn);

            main(options);
            testCase.verifyTrue(true);
        end

        function test_prioritized(testCase, scenario, mpa, parallel, optimizer_prioritized, coupling, priority, weight)
            lastwarn('');
            %load Config from json
            options = Config.load_from_file('tests/systemtests/Config_systemtests_prioritized.json');

            options.scenario_type = ScenarioType(scenario);
            options.mpa_type = MpaType(mpa);
            options.is_prioritized = true;

            if strcmp(parallel, 'parallel')
                options.compute_in_parallel = true;
            elseif strcmp(parallel, 'sequential')
                options.compute_in_parallel = false;
            end

            options.optimizer_type = OptimizerType(optimizer_prioritized);

            options.coupling = CouplingStrategies([coupling, '_coupling']);
            options.priority = PriorityStrategies([priority, '_priority']);
            options.weight = WeightStrategies([weight, '_weight']);
            options.validate();

            testCase.verifyEmpty(lastwarn);

            main(options);
            testCase.verifyTrue(true);
        end

    end

    methods (Test)

        function test_prioritized_STAC(testCase)
            lastwarn('');
            %load Config from json
            options = Config.load_from_file('tests/systemtests/Config_systemtests_prioritized_stac.json');
            options.validate();
            testCase.verifyEmpty(lastwarn);
            main(options);
            testCase.verifyTrue(true);
        end

        function test_visualization(testCase, scenario)
            lastwarn('');
            fprintf('\nvisualization systemtest for %s\n', scenario)
            %load Config from json
            options = Config.load_from_file(['tests/systemtests/Config_visualization_2', char(scenario), '.json']);
            testCase.verifyEmpty(lastwarn);

            options = options.validate();

            main(options);
            testCase.verifyTrue(true);
        end

        function test_plot_default(testCase, scenario)
            lastwarn('');
            fprintf('\ndefault evaluation plotting systemtest for %s\n', scenario)
            %load Config from json
            options = Config.load_from_file('tests/systemtests/Config_plot_default_results.json');
            options.scenario_type = ScenarioType(scenario);

            testCase.verifyEmpty(lastwarn);

            %let main run and read result file
            experiment_result = main(options);

            testCase.verifyTrue(true);

            load(experiment_result.output_path, 'experiment_result');

            %verify and check export too
            plot_default(experiment_result, do_export = true);
            testCase.verifyTrue(true);

            % verify export_video
            export_video( ...
                experiment_result, ...
                step_start = 5, ...
                step_end = 7 ...
            );

            % verify other result plotting functions
            step_indices = [1, 5, 9];
            plot_experiment_snapshots( ...
                experiment_result, ...
                step_indices, ...
                do_export = false ...
            );
            testCase.verifyTrue(true);
            plot_experiment_snapshots( ...
                experiment_result, ...
                step_indices, ...
                do_export = true ...
            );
            testCase.verifyTrue(true);

            plot_partitioned_graph(experiment_result);
            testCase.verifyTrue(true);

            plot_partitioned_graph( ...
                experiment_result, ...
                show_weights = true, ...
                show_cut_edges = false, ...
                i_step = 9, ...
                do_export = false, ...
                fig = figure(Visible = 'off') ...
            );
            testCase.verifyTrue(true);

            close all;
        end

        function test_plot_timing_result(testCase)
            lastwarn('');
            fprintf('\nTest plotting of timing results.\n')
            %load Config from json as done in priority_based and load specific result structs of already performed experiments
            options = Config.load_from_file('tests/systemtests/Config_plot_timing_result.json');

            testCase.verifyEmpty(lastwarn);

            %let main run
            main(options);

            testCase.verifyTrue(true);

            % Load results
            result_veh1 = load(FileNameConstructor.get_results_full_path(options, 1)).experiment_result;
            result_veh2 = load(FileNameConstructor.get_results_full_path(options, 2)).experiment_result;

            % Test plotting of runtime over multiple experiments
            results_multiple_experiments = cell(2);
            results_multiple_experiments{1, 1} = result_veh2; % use result of vehicle 2 also as if it was the result in an experiment with only one vehicle
            results_multiple_experiments{2, 1} = result_veh1;
            results_multiple_experiments{2, 2} = result_veh2;

            plot_runtime_multiple_experiments(results_multiple_experiments, do_export = true);
            testCase.verifyTrue(true);
            plot_runtime_multiple_experiments(results_multiple_experiments, do_export = false);
            testCase.verifyTrue(true);

            % Test plotting of runtime of one timestep within one experiment
            result_one_experiment = cell(1, 2);
            result_one_experiment{1, 1} = result_veh1;
            result_one_experiment{1, 2} = result_veh2;
            result_one_experiment_normalized = normalize_timing_results(result_one_experiment);

            plot_runtime_for_step(result_one_experiment_normalized, 5, do_export = true);
            testCase.verifyTrue(true);
            plot_runtime_for_step(result_one_experiment_normalized, 5, do_export = false);
            testCase.verifyTrue(true);

            close all;

        end

    end

end
