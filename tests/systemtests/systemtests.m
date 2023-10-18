classdef systemtests < matlab.unittest.TestCase

    properties (TestParameter)
        priority = {'coloring', 'constant', 'random', 'FCA', 'STAC'};
        scenario_type = {'circle', 'commonroad'};
        parallel = {'sequential', 'parallel'};
        use_cpp = {true, false}
        weight_strategy = {'constant_weight'
                           'STAC_weight'
                           'random_weight'
                           'distance_weight'};
        %    'optimal_weight'}; % currently not working, see issues
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

            %OPTION 1: manually create
            %create hlc manually because result returned by main does not contain mpa
            plant = PlantFactory.get_experiment_interface(config.environment);
            % create scenario
            random_seed = RandStream('mt19937ar');
            scenario = create_scenario(config, random_seed, plant);
            plant.setup(scenario)

            hlc_factory = HLCFactory();
            hlc_factory.set_scenario(scenario);
            dry_run = (scenario.options.environment == Environment.CpmLab); % TODO: dry run also for unified lab api?
            hlc = hlc_factory.get_hlc(plant.controlled_vehicle_ids, dry_run, plant);
            [result, ~] = hlc.run();

            %OPTION 2: let main run and read result file

            plot_default(mpa = hlc.mpa, scenario = result.scenario);
            testCase.verifyTrue(true);

            plot_default(mpa = hlc.mpa, scenario = result.scenario, do_export = true);
            testCase.verifyTrue(true);

        end

    end

end
