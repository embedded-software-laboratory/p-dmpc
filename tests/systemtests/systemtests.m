classdef systemtests < matlab.unittest.TestCase

    properties (TestParameter)
        scenario = {'circle', 'commonroad', 'lanelet2'};
        mpa = {'single_speed', 'triple_speed', 'realistic'};
        optimizer_centralized = {'MatlabOptimal', 'CppOptimal'}; % 'CppSampled' crashes matlab in tests
        optimizer_prioritized = {'MatlabOptimal', 'MatlabSampled', 'CppOptimal'};
        computation_mode = {'sequential', 'parallel_threads'};
        coupling = {'reachable_set', 'full', 'no', 'distance'};
        priority = {'coloring', 'constant', 'random', 'FCA', 'optimal', 'explorative'};
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

            experiment_result_actual = main(options);
            testCase.verifyTrue(true);

            experiment_result_expected = load_expected(experiment_result_actual);
            testCase.verifyTrue(experiment_result_expected.isequal(experiment_result_actual), "did not match expected result");

        end

        function test_prioritized(testCase, scenario, mpa, computation_mode, optimizer_prioritized, coupling, priority, weight)
            lastwarn('');
            %load Config from json
            options = Config.load_from_file('tests/systemtests/Config_systemtests_prioritized.json');

            options.scenario_type = ScenarioType(scenario);
            options.mpa_type = MpaType(mpa);
            options.is_prioritized = true;

            options.computation_mode = ComputationMode(computation_mode);

            options.optimizer_type = OptimizerType(optimizer_prioritized);

            options.coupling = CouplingStrategies([coupling, '_coupling']);
            options.priority = PriorityStrategies([priority, '_priority']);
            options.weight = WeightStrategies([weight, '_weight']);
            options.validate();

            testCase.verifyEmpty(lastwarn);

            experiment_result_actual = main(options);
            testCase.verifyTrue(true);

            experiment_result_expected = load_expected(experiment_result_actual);
            testCase.verifyTrue(experiment_result_expected.isequal(experiment_result_expected), "did not match expected result");
        end

    end

    methods (Test)

        function test_visualization(testCase, scenario)
            lastwarn('');
            fprintf('\nvisualization systemtest for %s\n', scenario)
            %load Config from json
            options = Config.load_from_file(['tests/systemtests/Config_visualization_2', char(scenario), '.json']);
            testCase.verifyEmpty(lastwarn);

            options = options.validate();

            experiment_result_actual = main(options);
            testCase.verifyTrue(true);

            experiment_result_expected = load_expected(experiment_result_actual);
            testCase.verifyTrue(experiment_result_expected.isequal(experiment_result_actual), "did not match expected result");
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

            %verify and check export too
            plot_default(experiment_result, do_export = true);

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

            plot_experiment_snapshots( ...
                experiment_result, ...
                step_indices, ...
                do_export = true ...
            );

            plot_partitioned_graph(experiment_result);

            plot_partitioned_graph( ...
                experiment_result, ...
                show_weights = true, ...
                show_cut_edges = false, ...
                i_step = 9, ...
                do_export = false, ...
                fig = figure(Visible = 'off') ...
            );

            plot_mpa_over_time(experiment_result);
            plot_mpa_local_reachable_sets(experiment_result);

            close all;
        end

        function test_plot_timing_result(testCase)
            lastwarn('');
            fprintf('\nTest plotting of timing results.\n')
            %load Config from json as done in priority_based and load specific result structs of already performed experiments
            options = Config.load_from_file('tests/systemtests/Config_plot_timing_result.json');

            testCase.verifyEmpty(lastwarn);

            %let main run
            experiment_result_3 = main(options);

            n_vehicles = 2;
            options.amount = n_vehicles;
            options.path_ids = options.path_ids(1:n_vehicles);
            experiment_result_2 = main(options);

            n_vehicles = 1;
            options.amount = n_vehicles;
            options.path_ids = options.path_ids(1:n_vehicles);
            experiment_result_1 = main(options);

            testCase.verifyTrue(true);

            results_multiple_experiments = [
                                            experiment_result_1
                                            experiment_result_2
                                            experiment_result_3
                                            ];
            plot_computation_time_over_vehicle_number(results_multiple_experiments, do_export = true);
            testCase.verifyTrue(true);
            plot_computation_time_over_vehicle_number(results_multiple_experiments, do_export = false);
            testCase.verifyTrue(true);

            % Test plotting of runtime of one timestep within one experiment
            k = 5;
            folder_path = FileNameConstructor.experiment_result_folder_path( ...
                experiment_result_3.options ...
            );
            filename = strcat('computation_time_step_', string(k), '.pdf');
            file_path = fullfile(folder_path, filename);
            export_plot( ...
                @plot_computation_time_for_step, ...
                experiment_result_3, ...
                k, ...
                file_path = file_path, ...
                export_fig_cfg = ExportFigConfig.document() ...
            );

            close all;

        end

    end

end
