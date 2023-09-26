classdef systemtests < matlab.unittest.TestCase

    properties (TestParameter)
        priority = {'coloring', 'constant', 'random', 'FCA', 'STAC'};
        scenario_type = {ScenarioType.circle, ScenarioType.commonroad};
        parallel = {'sequential', 'parallel'};
        use_cpp = {true, false}
    end

    methods (Test)

        function centralized(testCase, scenario_type, use_cpp)
            lastwarn('');
            fprintf('\ncentralized systemtest for %s\n', scenario_type);
            %load Config from json
            rawJson = fileread('tests/systemtests/Config_systemtests.json');
            options = Config();
            options = options.importFromJson(rawJson);
            options.scenario_type = scenario_type;
            options.is_prioritized = false;
            if use_cpp
                options.cpp_implementation = Function.CentralizedOptimal;
            else
                options.cpp_implementation = Function.None;
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
            options.scenario_type = scenario_type;
            options.is_prioritized = true;
            options.priority = PriorityStrategies.([priority, '_priority']);
            if use_cpp
                options.cpp_implementation = Function.GraphSearchPBOptimal;
            else
                options.cpp_implementation = Function.None;
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

    end

end
