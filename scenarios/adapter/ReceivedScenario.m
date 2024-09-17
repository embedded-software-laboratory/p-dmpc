classdef ReceivedScenario < ScenarioAdapter
    % ReceivedScenario This class gets a scenario with map and vehicle
    % information according to the member scenario via its interface

    properties (Access = public)
        scenario Scenario % (1, 1)
    end

    methods (Access = public)

        function obj = ReceivedScenario()
            obj = obj@ScenarioAdapter();
        end

        function init(obj, options, plant)
            % initialize the member scenario of the scenario adapter
            % currently it receives only a filepath to a lanelet2 map,
            % transforms the map to Matlab data types and
            % generates a scenario with random paths for each vehicle

            arguments
                obj (1, 1) ReceivedScenario
                options (1, 1) Config
                plant (1, 1) Plant
            end

            if isa(plant, "UnifiedTestbedInterface")
                filepath_lanelet2_map = plant.receive_map();
            else
                filepath_lanelet2_map = 'lab_reduced.osm';
            end

            obj.scenario = Lanelet2(options, filepath_lanelet2_map);

            % init manual vehicles
            obj.init_hdv(options);
        end

    end

    methods (Access = private)

        function map_as_string = receive_map(~)
            % receiving the map for the scenario
            % the functionality is currently part of UnifiedTestbedInterface
            map_as_string = [];
        end

    end

end
