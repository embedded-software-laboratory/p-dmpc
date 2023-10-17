classdef Scenario
    % SCENARIO  Scenario class

    properties (Access = public)
        vehicles = []; % array of Vehicle objects
        obstacles = {}; % static obstacles = {[xs;ys],...}
        lanelet_crossing_areas = {}; % crossing area of one vehicle's lanelet with another vehicle's lanelet
        options Config;

        model = [];
        dynamic_obstacle_area = {};

        lanelets; % coordinates of all lanelets
        intersection_lanelets; % IDs of intersection lanelets
        road_raw_data; % raw road data
        road_data_file_path; % path to file of road data
        lanelet_boundary; % boundaries of all lanelets
        lanelet_relationships; % relationship between two adjacent lanelets
        adjacency_lanelets; % (nLanelets x nLanelets) matrix, entry is 1 if two lanelets are adjacent
        intersection_center = [2.25, 2]; % (numOfIntersection x 2) matrix, positions of intersection center
        random_stream = RandStream('mt19937ar'); % for reproducibility
    end

    properties (Dependent)
    end

    methods

        function obj = Scenario()
        end

        function iter = get_next_dynamic_obstacles(obj, iter, iStep)
            % GET_NEXT_DYNAMIC_OBSTACLES   Filter dynamic obstacles in scenario by current time step and prediction horizon length.

            if ~isempty(obj.dynamic_obstacle_area)
                iter.dynamic_obstacle_area = obj.dynamic_obstacle_area(:, iStep:iStep + obj.options.Hp - 1);
            end

        end

    end

end
