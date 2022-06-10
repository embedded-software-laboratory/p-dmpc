classdef Scenario
% SCENARIO  Scenario class    

    properties
        vehicles = [];                  % array of Vehicle objects
        obstacles = {};                 % static obstacles = {[xs;ys],...}
        nVeh = 0;
        name = 'UnnamedScenario';
        controller_name = 'RHC';
        controller = @(s,i) centralized_controller(s,i);
        dt = 0.4;                       % RHC sample time [s]
        T_end = 18;                     % Duration of simulation. [s]
        Hp = 4;
        mpa;
        trim_set = 3;
        offset = 0.008;
        
        model = [];
        time_per_tick = 0.01;
        r_goal = 0.1;                   % goal circle
        dynamic_obstacle_area = {};
        dynamic_obstacle_shape = {};
        dynamic_obstacle_fullres = {};
        dynamic_obstacle_reachableSets = {};    % reachable sets of the coupled vehicles with higher priorities in other groups
        plot_limits = [-10,10;-10,10];          % default fallback if not defined
        adjacency;                              % (nVeh x nVeh) matrix, entry is 1 if two vehicles drive in two adjacent lanelets and their distance are smaller enough
        semi_adjacency;                         % (nVeh x nVeh) matrix, entry is 1 if two vehicles drive in two semi-adjacent lanelets and their distance are smaller enough
        directed_coupling;
        assignPrios = false;
        isParl = false                  % if use parallel computation (logical variable)
        vehicle_to_lanelet;
        lanelets;                       % coordinates of all lanelets
        intersection_lanelets;          % IDs of intersection lanelets
        boundary;
        road_raw_data;                  % raw road data
        lanelet_boundary;               % boundaries of all lanelets
        lanelet_relationships;          % relationship between two adjacent lanelets
        adjacency_lanelets;             % (nLanelets x nLanelets) matrix, entry is 1 if two lanelets are adjacent 
        semi_adjacency_lanelets;        % (nLanelets x nLanelets) matrix, entry is 1 if two lanelets are adjacent but not intersecting
        last_veh_at_intersection = [];  % store information about which vehicles were at the intersection in the last time step
        k;                              % simulation steps
        priority_option;
        coupling_weights = [];          % (nVeh x nVeh) matrix, coupling weights of all coupling vehicle pair; higher value indicates stronger coupling
        coupling_infos;                 % couling informations of each coupling pair
        ros_subscribers = {};           % ROS 2 subscribers (used to read messages from other vehicles)
        max_num_CLs = 4;                % max number of computation levels to limit the total planning time in each time step (used in the parallel computation)
        is_allow_non_convex = true     % whether to allow non-convex polygons; if true, the separating axis theorem cannot be used since it works only for convex polygons. `InterX.m` can be used instead.
        priority_list = [];             % priority list of vehicles; a smaller value for a higher priority
        strategy_consider_veh_with_lower_prio = '1';    % currently five stategies are supported to let vehicle with a higher priority consider vehicle with a lower priority
                                                        % '0': do not consider 
                                                        % '1': consider currently occupied area as static obstacle
                                                        % '2': consider one-step reachable sets as static obstacle
                                                        % '3': consider old trajectory as dynamic obstacle
                                                        % '4': consider the occupied area of emergency braking maneuver as static obstacle 
        is_allow_enter_crossing_area = false; % whether to allow lower priority vehicle enter the crossing area of its own lanelet and lanelet of its coupling vehicle at the intersection
        time_enter_intersection = []; % time step when vehicle enters the intersection
        intersection_center = [2.25, 2]; % (numOfIntersection x 2) matrix, positions of intersection center
        distance_threshold_intersection = [1.1]; % vector with length numOfIntersection, threshold of distance from a vehicle's position to the intersection center point exceed which a vehicle is considered as entering the intersection
        belonging_vector; % a column vector whose value indicate which group each vehicle belongs to 
        lanelet_crossing_areas = {}; % crossing area of one vehicle's lanelet with another vehicle's lanelet
    end
    
    properties (Dependent)
        tick_per_step;
        Hu;
        k_end;
    end
    
    methods
        function obj = Scenario()
        end
        
        function result = get.k_end(obj)
            result = floor(obj.T_end/obj.dt);
        end
        function result = get.tick_per_step(obj)
            result = obj.dt/obj.time_per_tick;
        end
        function result = get.Hu(obj)
            result = obj.Hp;
        end
        
        function plot(obj)
            for iVeh = 1:numel(obj.vehicles)
                % vehicle rectangle
                veh = obj.vehicles(iVeh);
                veh.plot(vehColor(iVeh));
                % reference trajectory
                line(   veh.referenceTrajectory(:,1), ...
                        veh.referenceTrajectory(:,2), ...
                        'Color',vehColor(iVeh),'LineStyle', '--', 'LineWidth',1 ...
                );
            end
        end
        
    end
    
    
end
