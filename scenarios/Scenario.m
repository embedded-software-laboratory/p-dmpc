classdef Scenario
% SCENARIO  Scenario class    

    properties
        vehicles = [];                   % array of Vehicle objects
        obstacles = {};                  % static obstacles = {[xs;ys],...}
        lanelet_crossing_areas = {}; % crossing area of one vehicle's lanelet with another vehicle's lanelet
        name = 'UnnamedScenario';
        controller_name = 'RHC';
        controller = @(s,i) centralized_controller(s,i);
        mpa;
        manual_vehicle_id = 0;
        second_manual_vehicle_id = 0;
        manual_mpa_initialized = false;
        updated_manual_vehicle_path = false;
        second_manual_mpa_initialized = false;
        updated_second_manual_vehicle_path = false;
        g29_force_feedback                      % to send position and torque to Logitech G29 steering wheel
        vehicle_ids = [];
        options;
        speed_profile_mpas = [];
        
        model = [];
        r_goal = 0.1;                   % goal circle
        dynamic_obstacle_area = {};
        dynamic_obstacle_shape = {};
        dynamic_obstacle_fullres = {};
        dynamic_obstacle_reachableSets = {};    % reachable sets of the coupled vehicles with higher priorities in other groups

        adjacency;                              % (nVeh x nVeh) matrix, entry is 1 if two vehicles drive in two adjacent lanelets and their distance are smaller enough
        semi_adjacency;                         % (nVeh x nVeh) matrix, entry is 1 if two vehicles drive in two semi-adjacent lanelets and their distance are smaller enough
        directed_coupling;                      % nVeh-by-nVeh matrix, entry if 1 if the corresponding two vehicles are coupled
        directed_coupling_reduced;              % nVeh-by-nVeh matrix, reduced directed adjacency by forbidding vehicles entering their lanelet crossing area
        assignPrios = false;
        vehicle_to_lanelet;
        lanelets;                       % coordinates of all lanelets
        intersection_lanelets;          % IDs of intersection lanelets
        boundary;
        road_raw_data;                  % raw road data
        lanelet_boundary;               % boundaries of all lanelets
        lanelet_relationships;          % relationship between two adjacent lanelets
        adjacency_lanelets;             % (nLanelets x nLanelets) matrix, entry is 1 if two lanelets are adjacent 
        semi_adjacency_lanelets;        % (nLanelets x nLanelets) matrix, entry is 1 if two lanelets are adjacent but not intersecting
        last_vehs_at_intersection = [];  % store information about which vehicles were at the intersection in the last time step
        k;                              % simulation steps
        coupling_weights = [];          % (nVeh x nVeh) matrix, coupling weights of all coupling vehicle pair; higher value indicates stronger coupling
        coupling_weights_optimal = [];  % "optimal" coupling weights
        coupling_weights_reduced = [];  % reduced coupling weights by forbidding vehicles entering their lanelet crossing areas
        coupling_info;                  % couling information of each coupling pair
        ros_subscribers = {};           % ROS 2 subscribers (used to read messages from other vehicles)
        mixedTrafficCollisionAvoidanceMode = 0;     % mode for collision avoidance in CPM Lab Mode with manual vehicles
        priority_list = [];             % priority list of vehicles; a smaller value for a higher priority
        time_enter_intersection = []; % time step when vehicle enters the intersection
        intersection_center = [2.25, 2]; % (numOfIntersection x 2) matrix, positions of intersection center
        belonging_vector; % a column vector whose value indicate which group each vehicle belongs to 
        parl_groups_info; % struct, store information of parallel groups 
        timer;            % struct, used to store computation time of different parts
        num_couplings_between_grps; % number of couplings between parallel groups
        num_couplings_between_grps_ignored; % reduced number of couplings between groups by using lanelet crossing lanelets
        random_seed = RandStream('mt19937ar'); % for reproducibility
    end
    
    properties (Dependent)
    end
    
    methods
        function obj = Scenario()
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
