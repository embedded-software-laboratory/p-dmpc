classdef Scenario
% SCENARIO  Scenario class    

    properties
        vehicles = [];                   % array of Vehicle objects
        obstacles = {};                  % static obstacles = {[xs;ys],...}
        lanelet_crossing_areas = {}; % crossing area of one vehicle's lanelet with another vehicle's lanelet
        controller_name = 'RHGS';
        controller = @centralized_controller;
        sub_controller = @graph_search;
        mpa;
        manual_mpa_initialized = false;
        updated_manual_vehicle_path = false;
        second_manual_mpa_initialized = false;
        updated_second_manual_vehicle_path = false;
        g29_force_feedback                      % to send position and torque to Logitech G29 steering wheel
        options;
        speed_profile_mpas = [];        
        
        model = [];
        r_goal = 0.1;                   % goal circle
        dynamic_obstacle_area = {};
        dynamic_obstacle_shape = {};
        dynamic_obstacle_fullres = {};
        dynamic_obstacle_reachableSets = {};    % reachable sets of the coupled vehicles with higher priorities in other groups

        assignPrios = false;
        lanelets;                       % coordinates of all lanelets
        intersection_lanelets;          % IDs of intersection lanelets
        boundary;
        road_raw_data;                  % raw road data
        lanelet_boundary;               % boundaries of all lanelets
        lanelet_relationships;          % relationship between two adjacent lanelets
        adjacency_lanelets;             % (nLanelets x nLanelets) matrix, entry is 1 if two lanelets are adjacent 
        semi_adjacency_lanelets;        % (nLanelets x nLanelets) matrix, entry is 1 if two lanelets are adjacent but not intersecting
        ros_subscribers = {};           % ROS 2 subscribers (used to read messages from other vehicles)
        priority_list = 1;             % priority list of vehicles; a smaller value for a higher priority
        time_enter_intersection = []; % time step when vehicle enters the intersection
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
