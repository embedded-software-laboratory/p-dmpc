classdef systemtests < matlab.unittest.TestCase

    properties (TestParameter)
        priority = {'coloring', 'constant', 'random', 'FCA', 'STAC'};
        scenario_type = {'circle', 'commonroad'};
        parallel = {'sequential', 'parallel'};
        use_cpp = {true, false}
        weight_strategy = {'constant_weight'
                           'random_weight'
                           'STAC_weight'
                           'distance_weight'};
        coupler_type = {'ReachableSet', 'FullyConnected'};
    end

    methods (Test)

        function centralized(testCase, scenario_type, use_cpp)
            lastwarn('');
            fprintf('\ncentralized systemtest for %s\n', scenario_type);
            %load Config from json
            options = Config.load_from_file('tests/systemtests/Config_systemtests.json');
            options.scenario_type = ScenarioType(scenario_type);
            options.is_prioritized = false;

            if use_cpp
                options.cpp_optimizer = CppOptimizer.CentralizedOptimalPolymorphic;
            else
                options.cpp_optimizer = CppOptimizer.None;
            end

            testCase.verifyEmpty(lastwarn);

            main(options);
            testCase.verifyTrue(true);
        end

        function priority_based(testCase, scenario_type, parallel, priority, use_cpp)
            lastwarn('');
            fprintf('\nprioritized %s systemtest for %s with %s priority\n', parallel, scenario_type, priority);
            %load Config from json
            options = Config.load_from_file('tests/systemtests/Config_systemtests.json');
            options.scenario_type = ScenarioType(scenario_type);
            options.is_prioritized = true;
            options.priority = PriorityStrategies.([priority, '_priority']);

            if use_cpp
                options.cpp_optimizer = CppOptimizer.GraphSearchPBOptimal;
            else
                options.cpp_optimizer = CppOptimizer.None;
            end

            if strcmp(parallel, 'parallel')
                options.compute_in_parallel = true;
            elseif strcmp(parallel, 'sequential')
                options.compute_in_parallel = false;
            end

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

        function test_coupler(testCase, coupler_type)
            lastwarn('');
            fprintf('\ncoupler systemtest for %s\n', coupler_type)
            %load Config from json
            rawJson = fileread('tests/systemtests/Config_systemtests.json');
            options = Config();
            options = options.importFromJson(rawJson);
            options.scenario_type = ScenarioType.circle;
            options.is_prioritized = true;
            options.max_num_CLs = 1;
            options.coupler_type = CouplerType(coupler_type);

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
            result = main(options);

            testCase.verifyTrue(true);

            output_path = FileNameConstructor.get_results_full_path( ...
                result.options, ...
                result.options.amount ... %gen_scenario_name uses options.amount as default
            );
            full_result = load(output_path);

            %verify without exporting
            plot_default(full_result.result, do_export = false);
            testCase.verifyTrue(true);

            %verify and check export too
            plot_default(full_result.result, do_export = true);
            testCase.verifyTrue(true);

            % verify export_video
            export_video(full_result.result);

            % verify other result plotting functions
            step_indices = [1, 5, 9];
            plot_experiment_snapshots( ...
                full_result.result, ...
                step_indices, ...
                do_export = false ...
            );
            testCase.verifyTrue(true);
            plot_experiment_snapshots( ...
                full_result.result, ...
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
            results_multiple_experiments{1, 1} = result_veh2.result; % use result of vehicle 2 also as if it was the result in an experiment with only one vehicle
            results_multiple_experiments{2, 1} = result_veh1.result;
            results_multiple_experiments{2, 2} = result_veh2.result;

            plot_runtime_multiple_experiments(results_multiple_experiments, do_export = true);
            testCase.verifyTrue(true);
            plot_runtime_multiple_experiments(results_multiple_experiments, do_export = false);
            testCase.verifyTrue(true);

            % Test plotting of runtime of one timestep within one experiment
            result_one_experiment = cell(1, 2);
            result_one_experiment{1, 1} = result_veh1.result;
            result_one_experiment{1, 2} = result_veh2.result;
            result_one_experiment_normalized = normalize_timing_results(result_one_experiment);

            plot_runtime_for_step(result_one_experiment_normalized, 5, do_export = true);
            testCase.verifyTrue(true);
            plot_runtime_for_step(result_one_experiment_normalized, 5, do_export = false);
            testCase.verifyTrue(true);

            close all;

        end

    end

end
