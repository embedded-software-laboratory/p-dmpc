classdef (Abstract) ScenarioAdapter < handle
    % ScenarioAdapter This class provides an adapter for an HighLevelController
    % to use the required scenario

    properties (Access = public)
        manual_vehicles ManualVehicle % (1, n_hdv)
    end

    properties (Abstract)
        scenario Scenario % (1, 1)
    end

    methods (Abstract)
        init(obj, options, plant)
    end

    methods (Static)

        function scenario_adapter = get_scenario_adapter(scenario_type)

            arguments
                scenario_type (1, 1) ScenarioType
            end

            if scenario_type == ScenarioType.testbed_default
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

    methods (Access = protected)

        function init_hdv(obj, options)

            for i_hdv = 1:options.manual_control_config.amount
                hdv_id = options.manual_control_config.hdv_ids(i_hdv);
                obj.manual_vehicles(i_hdv) = ManualVehicle(hdv_id, options, obj.scenario.road_raw_data);
            end

        end

    end

end
