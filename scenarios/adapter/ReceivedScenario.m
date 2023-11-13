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

        function init(obj, options, filepath_lanelet2_map)
            % initialize the member scenario of the scenario adapter
            % currently it receives only a filepath to a lanelet2 map,
            % transforms the map to Matlab data types and
            % generates a scenario with random paths for each vehicle

            arguments
                obj (1, 1) ReceivedScenario
                options (1, 1) Config
                filepath_lanelet2_map (1, :) char = 'lab_reduced.osm';
            end

            % filepath_lanelet2_map = obj.receive_map();
            obj.scenario = lanelet2_scenario(options.amount, options.path_ids, filepath_lanelet2_map);

            % init manual vehicles
            obj.init_hdv(options);
        end

    end

    methods (Access = private)

        function map_as_string = receive_map(~)
            % receiving the map for the scenario
            % the functionality is currently part of UnifiedLabApi
            map_as_string = [];
        end

    end

end
