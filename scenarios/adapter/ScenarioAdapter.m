classdef (Abstract) ScenarioAdapter < handle
    % ScenarioAdapter This class provides an adapter for an HighLevelController
    % to use the required scenario

    properties (Abstract)
        scenario Scenario % (1, 1)
    end

    methods (Abstract)
        % filepath either for a full scenario
        % in a .mat file or a lanelet2 map in a .osm file
        init(obj, amount, path_ids, filepath)
    end

    methods (Static)

        function scenario_adapter = get_scenario_adapter(scenario_type)

            arguments
                scenario_type (1, 1) ScenarioType
            end

            if scenario_type == ScenarioType.lab_default
                scenario_adapter = ReceivedScenario();
                return
            end

            scenario_adapter = BuiltScenario();
        end

    end

    methods (Access = public)

        function obj = ScenarioAdapter()
        end

    end

end
