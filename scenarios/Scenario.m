classdef Scenario
    % SCENARIO  Scenario class

    properties (Access = public)
        vehicles = []; % array of Vehicle objects
        obstacles = {}; % static obstacles = {[xs;ys],...}
        lanelet_crossing_areas = {}; % crossing area of one vehicle's lanelet with another vehicle's lanelet
        mpa;
        options;
        speed_profile_mpas = [];

        model = [];
        r_goal = 0.1; % goal circle
        dynamic_obstacle_area = {};
        dynamic_obstacle_shape = {};
        dynamic_obstacle_fullres = {};
        dynamic_obstacle_reachableSets = {}; % reachable sets of the coupled vehicles with higher priorities in other groups

        assignPrios = false;
        lanelets; % coordinates of all lanelets
        intersection_lanelets; % IDs of intersection lanelets
        boundary;
        road_raw_data; % raw road data
        road_data_file_path; % path to file of road data
        lanelet_boundary; % boundaries of all lanelets
        lanelet_relationships; % relationship between two adjacent lanelets
        adjacency_lanelets; % (nLanelets x nLanelets) matrix, entry is 1 if two lanelets are adjacent
        priority_list = 1; % priority list of vehicles; a smaller value for a higher priority
        intersection_center = [2.25, 2]; % (numOfIntersection x 2) matrix, positions of intersection center
        random_stream = RandStream('mt19937ar'); % for reproducibility
    end

    properties (Dependent)
    end

    methods

        function obj = Scenario()
        end

    end

end
