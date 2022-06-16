classdef Scenario
% SCENARIO  Scenario class    

    properties
        vehicles = [];                  % array of Vehicle objects
        obstacles = {};                 % static obstacles = {[xs;ys],...}
        lanelet_crossing_areas = {}; % crossing area of one vehicle's lanelet with another vehicle's lanelet
        nVeh = 0;
        name = 'UnnamedScenario';
        controller_name = 'RHC';
        controller = @(s,i) centralized_controller(s,i);
        dt = 0.4;                       % RHC sample time [s]
        T_end = 18;                     % Duration of simulation. [s]
        Hp = 4;
        mpa;
        manual_vehicle_id = 0;
        second_manual_vehicle_id = 0;
        manual_mpa_initialized = false;
        updated_manual_vehicle_path = false;
        second_manual_mpa_initialized = false;
        updated_second_manual_vehicle_path = false;
        vehicle_ids = [];
        trim_set = 3;
        offset = 0.01;
        options;
        speed_profile_mpas = [];
        
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
        coupling_info;                  % couling information of each coupling pair
        ros_subscribers = {};           % ROS 2 subscribers (used to read messages from other vehicles)
        max_num_CLs = 4;                % max number of computation levels to limit the total planning time in each time step (used in the parallel computation)
        pool;
        mixedTrafficCollisionAvoidanceMode = 0;     % mode for collision avoidance in CPM Lab Mode with manual vehicles
        priority_list = [];             % priority list of vehicles; a smaller value for a higher priority
        is_allow_non_convex = true      % whether to allow non-convex polygons; if true, the separating axis theorem cannot be used since it works only for convex polygons. `InterX.m` can be used instead.
        strategy_consider_veh_without_ROW = '1'; % Stategies to let vehicle with the right-of-way consider vehicle without the right-of-way
                                                 % '0': do not consider 
                                                 % '1': consider currently occupied area as static obstacle
                                                 % '2': consider one-step reachable sets as static obstacle
                                                 % '3': consider old trajectory as dynamic obstacle
                                                 % '4': consider the occupied area of emergency braking maneuver as static obstacle 
        strategy_enter_crossing_area = '3'; % Strategy to let vehicle without the right-of-way enter the crossing area of its lanelet with lanelet of its coupled vehicle
                                            % '0': no constraint on entering the crossing area 
                                            % '1': not allowed to enter the crossing area if they are coupled at intersecting lanelets of the intersection
                                            % '2': not allowed to enter the crossing area if they are coupled at intersecting or merging lanelets of the intersection
                                            % '3': not allowed to enter the crossing area if they are coupled at intersecting or merging lanelets regardless whether they are at the intersection or not
        
        time_enter_intersection = []; % time step when vehicle enters the intersection
        intersection_center = [2.25, 2]; % (numOfIntersection x 2) matrix, positions of intersection center
        distance_threshold_intersection = [1.1]; % vector with length numOfIntersection, threshold of distance from a vehicle's position to the intersection center point exceed which a vehicle is considered as entering the intersection
        belonging_vector; % a column vector whose value indicate which group each vehicle belongs to 
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
