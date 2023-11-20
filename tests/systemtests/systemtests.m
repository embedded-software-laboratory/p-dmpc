classdef systemtests < matlab.unittest.TestCase

    properties (TestParameter)
        priority = {'coloring', 'constant', 'random', 'FCA', 'STAC'};
        scenario_type = {'circle', 'commonroad'};
        parallel = {'sequential', 'parallel'};
        optimizer_type_centralized = {'MatlabOptimal', 'CppOptimal'}; % 'CppSampled' crashes matlab in tests
        optimizer_type_priority_based = {'MatlabOptimal', 'MatlabSampled', 'CppOptimal'};
        weight_strategy = {'constant_weight'
                           'random_weight'
                           'STAC_weight'
                           'distance_weight'};
        coupling = {'reachable_set_coupling', 'full_coupling', 'no_coupling', 'distance_coupling'};
    end

    methods (Test)

        function centralized(testCase, scenario_type, optimizer_type_centralized)
            lastwarn('');
            fprintf('\ncentralized systemtest for %s\n', scenario_type);
            %load Config from json
            options = Config.load_from_file('tests/systemtests/Config_systemtests.json');
            options.scenario_type = ScenarioType(scenario_type);
            options.is_prioritized = false;
            options.optimizer_type = OptimizerType(optimizer_type_centralized);

            testCase.verifyEmpty(lastwarn);

            main(options);
            testCase.verifyTrue(true);
        end

        function priority_based(testCase, scenario_type, parallel, optimizer_type_priority_based)
            lastwarn('');
            fprintf('\nprioritized %s systemtest for %s\n', parallel, scenario_type);
            %load Config from json
            options = Config.load_from_file('tests/systemtests/Config_systemtests.json');
            options.scenario_type = ScenarioType(scenario_type);
            options.is_prioritized = true;
            options.optimizer_type = OptimizerType(optimizer_type_priority_based);

            if strcmp(parallel, 'parallel')
                options.compute_in_parallel = true;
            elseif strcmp(parallel, 'sequential')
                options.compute_in_parallel = false;
            end

            testCase.verifyEmpty(lastwarn);

            main(options);
            testCase.verifyTrue(true);
        end

        function priority_strategy(testCase, scenario_type, priority)
            lastwarn('');
            fprintf('\ntest priority %s for %s\n', priority, scenario_type);
            %load Config from json
            options = Config.load_from_file('tests/systemtests/Config_systemtests.json');
            options.scenario_type = ScenarioType(scenario_type);
            options.is_prioritized = true;
            options.priority = PriorityStrategies.([priority, '_priority']);

            testCase.verifyEmpty(lastwarn);

            main(options);
            testCase.verifyTrue(true);
        end

        function grouping(testCase)
            lastwarn('');
            fprintf('\nTesting grouping algorithm\n');
            %load Config from json
            options = Config.load_from_file('tests/systemtests/Config_grouping.json');

            main(options);
            testCase.verifyTrue(true);
        end

        function visualization(testCase, scenario_type)
            lastwarn('');
            fprintf('\nvisualization systemtest for %s\n', scenario_type)
            %load Config from json
            options = Config.load_from_file(['tests/systemtests/Config_visualization_2', char(scenario_type), '.json']);
            testCase.verifyEmpty(lastwarn);

            main(options);
            testCase.verifyTrue(true);
        end

        function test_weigher(testCase, scenario_type, weight_strategy)
            lastwarn('');
            fprintf('\nweigher systemtest for %s\n', scenario_type)
            %load Config from json
            options = Config.load_from_file('tests/systemtests/Config_systemtests.json');
            options.scenario_type = ScenarioType(scenario_type);
            options.is_prioritized = true;
            options.max_num_CLs = 1;
            options.weight = WeightStrategies(weight_strategy);

            testCase.verifyEmpty(lastwarn);

            main(options);
            testCase.verifyTrue(true);
        end

        function test_coupler(testCase, scenario_type, coupling)
            lastwarn('');
            fprintf('\ncoupler systemtest for %s\n', coupling)
            %load Config from json
            options = Config.load_from_file('tests/systemtests/Config_coupler.json');
            options.coupling = CouplingStrategies(coupling);
            options.scenario_type = ScenarioType(scenario_type);

            testCase.verifyEmpty(lastwarn);

            main(options);
            testCase.verifyTrue(true);
        end

        function test_plot_default(testCase, scenario_type)
            lastwarn('');
            fprintf('\ndefault evaluation plotting systemtest for %s\n', scenario_type)
            %load Config from json
            options = Config.load_from_file('tests/systemtests/Config_plot_default_results.json');
            options.scenario_type = ScenarioType(scenario_type);

            testCase.verifyEmpty(lastwarn);

            %let main run and read result file
            experiment_result = main(options);

            testCase.verifyTrue(true);

            output_path = FileNameConstructor.get_results_full_path( ...
                experiment_result.options, ...
                experiment_result.options.amount ... %gen_scenario_name uses options.amount as default
            );
            full_result = load(output_path);

            %verify without exporting
            plot_default(full_result.experiment_result, do_export = false);
            testCase.verifyTrue(true);

            %verify and check export too
            plot_default(full_result.experiment_result, do_export = true);
            testCase.verifyTrue(true);

            % verify export_video
            export_video(full_result.experiment_result);

            % verify other result plotting functions
            step_indices = [1, 5, 9];
            plot_experiment_snapshots( ...
                full_result.experiment_result, ...
                step_indices, ...
                do_export = false ...
            );
            testCase.verifyTrue(true);
            plot_experiment_snapshots( ...
                full_result.experiment_result, ...
                step_indices, ...
                do_export = true ...
            );
            testCase.verifyTrue(true);
            close all;
        end

        function test_plot_timing_result(testCase)
            lastwarn('');
            fprintf('\nTest plotting of timing results. Note: No experiment is run here but instead only old results are reused.\n')
            %load Config from json as done in priority_based and load specific result structs of already performed experiments
            options = Config.load_from_file('tests/systemtests/Config_systemtests.json');
            options.scenario_type = ScenarioType.circle;
            options.is_prioritized = true;
            options.compute_in_parallel = true;

            % Load old results created in priority_based
            result_veh1 = load(FileNameConstructor.get_results_full_path(options, 1));
            result_veh2 = load(FileNameConstructor.get_results_full_path(options, 2));

            % Test plotting of runtime over multiple experiments
            results_multiple_experiments = cell(2);
            results_multiple_experiments{1, 1} = result_veh2.experiment_result; % use result of vehicle 2 also as if it was the result in an experiment with only one vehicle
            results_multiple_experiments{2, 1} = result_veh1.experiment_result;
            results_multiple_experiments{2, 2} = result_veh2.experiment_result;

            plot_runtime_multiple_experiments(results_multiple_experiments, do_export = true);
            testCase.verifyTrue(true);
            plot_runtime_multiple_experiments(results_multiple_experiments, do_export = false);
            testCase.verifyTrue(true);

            % Test plotting of runtime of one timestep within one experiment
            result_one_experiment = cell(1, 2);
            result_one_experiment{1, 1} = result_veh1.experiment_result;
            result_one_experiment{1, 2} = result_veh2.experiment_result;
            result_one_experiment_normalized = normalize_timing_results(result_one_experiment);

            plot_runtime_for_step(result_one_experiment_normalized, 5, do_export = true);
            testCase.verifyTrue(true);
            plot_runtime_for_step(result_one_experiment_normalized, 5, do_export = false);
            testCase.verifyTrue(true);

            close all;

        end

    end

end
