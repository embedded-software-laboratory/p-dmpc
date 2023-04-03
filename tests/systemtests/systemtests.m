classdef systemtests < matlab.unittest.TestCase

    properties (TestParameter)
        priority = {'coloring', 'constant', 'random', 'FCA', 'STAC'};
        %priority = {'coloring', 'right_of_way', 'constant', 'random', 'FCA', 'STAC'};
        scenario_name = {'Circle_scenario', 'Commonroad'};
        parallel = {'sequential', 'parallel'};
        use_cpp = {true, false}
    end

    methods (Test)
        function centralized(testCase, scenario_name)
            lastwarn('');
            fprintf('\ncentralized systemtest for %s\n', scenario_name);
            %load Config from json
            rawJson = fileread('tests/systemtests/Config_systemtests.json');
            options = Config();
            options = options.importFromJson(rawJson);
            options.scenario_name = scenario_name;
            options.isPB = false;
            testCase.verifyEmpty(lastwarn);

            main(options);
            testCase.verifyTrue(true);
        end

        function priority_based(testCase, scenario_name, parallel, priority, use_cpp)
            lastwarn('');
            fprintf('\nprioritized %s systemtest for %s with %s priority\n', parallel, scenario_name, priority);
            %load Config from json
            rawJson = fileread('tests/systemtests/Config_systemtests.json');
            options = Config();
            options = options.importFromJson(rawJson);
            options.scenario_name = scenario_name;
            options.isPB = true;
            options.priority = Priority_strategies.([priority, '_priority']);
            options.use_cpp = use_cpp;
            if strcmp(parallel, 'parallel')
                options.isParl = true;
            elseif strcmp(parallel, 'sequential')
                options.isParl = false;
            end
            testCase.verifyEmpty(lastwarn);

            main(options);
            testCase.verifyTrue(true);
        end

        function visualization(testCase, scenario_name)
            lastwarn('');
            fprintf('\nvisualization systemtest for %s\n', scenario_name)
            %load Config from json
            rawJson = fileread(['tests/systemtests/Config_visualization_2', scenario_name, '.json']);
            options = Config();
            options = options.importFromJson(rawJson);
            testCase.verifyEmpty(lastwarn);

            main(options);
            testCase.verifyTrue(true);
        end
    end

end
