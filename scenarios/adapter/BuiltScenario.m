classdef BuiltScenario < ScenarioAdapter
    % BuiltScenario This scenario adapter loads a scenario
    % that was built from the disk

    properties (Access = public)
        scenario Scenario % (1, 1)
    end

    methods (Access = public)

        function obj = BuiltScenario()
            obj = obj@ScenarioAdapter();
        end

        function init(obj, amount, ~, filepath_matlab_scenario)
            % initialize the member scenario of the scenario adapter
            % with a scenario saved in filepath_matlab_scenario

            arguments
                obj (1, 1) BuiltScenario
                amount (1, 1) double
                ~% path_ids are not used in this scenario adapter
                filepath_matlab_scenario (1, :) char = 'scenario.mat';
            end

            scenario_loaded = load(filepath_matlab_scenario, 'scenario').scenario;

            assert( ...
                length(scenario_loaded.vehicles) == amount, ...
                'Amount of vehicles in scenario (%d) does not match amount (%d)', ...
                length(scenario_loaded.vehicles), ...
                amount ...
            )

            obj.scenario = scenario_loaded;
        end

    end

end
