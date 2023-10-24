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
    end

    methods (Test)

        function centralized(testCase, scenario_type, use_cpp)
            lastwarn('');
            fprintf('\ncentralized systemtest for %s\n', scenario_type);
            %load Config from json
            rawJson = fileread('tests/systemtests/Config_systemtests.json');
            options = Config();
            options = options.importFromJson(rawJson);
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
            rawJson = fileread('tests/systemtests/Config_systemtests.json');
            options = Config();
            options = options.importFromJson(rawJson);
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

        function visualization(testCase, scenario_type)
            lastwarn('');
            fprintf('\nvisualization systemtest for %s\n', scenario_type)
            %load Config from json
            rawJson = fileread(['tests/systemtests/Config_visualization_2', char(scenario_type), '.json']);
            options = Config();
            options = options.importFromJson(rawJson);
            testCase.verifyEmpty(lastwarn);

            main(options);
            testCase.verifyTrue(true);
        end

        function test_weigher(testCase, scenario_type, weight_strategy)
            lastwarn('');
            fprintf('\nweigher systemtest for %s\n', scenario_type)
            %load Config from json
            rawJson = fileread('tests/systemtests/Config_systemtests.json');
            options = Config();
            options = options.importFromJson(rawJson);
            options.scenario_type = ScenarioType(scenario_type);
            options.is_prioritized = true;
            options.max_num_CLs = 1;
            options.weight = WeightStrategies(weight_strategy);

            testCase.verifyEmpty(lastwarn);

            main(options);
            testCase.verifyTrue(true);
        end

        function test_plot_default(testCase, scenario_type)
            lastwarn('');
            fprintf('\ndefault evaluation plotting systemtest for %s\n', scenario_type)
            %load Config from json
            rawJson = fileread('tests/systemtests/Config_plot_default_results.json');
            config = Config();
            config = config.importFromJson(rawJson);

            testCase.verifyEmpty(lastwarn);

            %let main run and read result file
            [result, ~] = main(config);

            testCase.verifyTrue(true);

            output_path = FileNameConstructor.get_results_full_path( ...
                result.scenario.options, ...
                config.amount ... %gen_scenario_name uses config.amount as default
            );
            full_result = load(output_path);

            %verify without exporting
            plot_default(full_result.result, do_export = false);
            testCase.verifyTrue(true);

            %verify and check export too
            plot_default(full_result.result, do_export = true);
            testCase.verifyTrue(true);

        end

    end

end
