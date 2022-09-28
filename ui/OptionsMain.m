classdef OptionsMain
    % OPTIONSMAIN Options for main.m

    properties
        manualVehicle_id        % one of the following: {'1','2',...,'nVeh'}, defines which vehicle is the first manual vehicle
        firstManualVehicleMode  % one of the following: {'1','2'}, '1' for guided-mode while '2' for expert-mode
        manualVehicle_id2       % one of the following: {'1','2',...,'nVeh'}, defines which vehicle is the second manual vehicle
        secondManualVehicleMode % one of the following: {'1','2'}, '1' for guided-mode while '2' for expert-mode
        collisionAvoidanceMode  % one of the following: {1,2,3}, defines collision avoid mode, 1 for priority-based, 2 for reachability analysis guided-mode, while 3 for reachability analysis expert-mode
        is_sim_lab              % true/false, is simulation or lab experiment
        is_mixed_traffic        % true/false, is mixed traffic
        force_feedback_enabled  % true/false
        consider_RSS    % true/false, is consider Responsibility-Sensitive Safety
        isPB            % true/false, is prioritize vehicles
        angles          % 1-by-nVeh scalar vector
        amount          % integer, number of vehicles
        visu            % 1-by-2 vector, online plotting is enabled if the first entry if true; node visualization is enabled if the second entry is true
        isParl          % true/false, is use parallel computation
        scenario_name   % one of the follows: {'Circle_scenario','Commonroad'}
        priority        % one of the following: {'topo_priority','right_of_way_priority','constant_priority','random_priority','FCA_priority','STAC_priority'}, defines which priority assignmen strategy is used
        dt              % scalar, sample time
        Hp              % scalar, prediction horizon
        trim_set        % scalar, ID of trim primitive
        T_end           % scalar, simulation duration
        max_num_CLs     % integer, maximum allowerd number of computation levels
        strategy_consider_veh_without_ROW       % one of the following: {'1','2','3','4','5'}, strategy of letting higher-priority vehicles consider their coupled vehicles with lower priorities
                                                    % '1': do not consider 
                                                    % '2': consider currently occupied area as static obstacle
                                                    % '3': consider the occupied area of emergency braking maneuver as static obstacle 
                                                    % '4': consider one-step reachable sets as static obstacle
                                                    % '5': consider old trajectory as dynamic obstacle
        strategy_enter_lanelet_crossing_area    % one of the following: {'1','2','3','4'}, strategy of forbidding vehicles with lower priorities entering their lanelet crossing area
                                                    % '1': no constraint on entering the crossing area 
                                                    % '2': not allowed to enter the crossing area if they are coupled at intersecting lanelets of the intersection
                                                    % '3': not allowed to enter the crossing area if they are coupled at intersecting or merging lanelets of the intersection
                                                    % '4': not allowed to enter the crossing area if they are coupled at intersecting or merging lanelets regardless whether they are at the intersection or not
        isSaveResult                % true/false, is save result
        isSaveResultReduced = true; % true/false, if true, reduced result will be save to save disk space (useful for a long run of simulation)
        customResultName        % string or char, custom file name to save result
        isAllowInheritROW       % true/false, is allow vehicles to inherit the right-of-way from their front vehicles
        is_eval                 % true/false, 
        visualize_reachable_set % true/false, 
        is_free_flow = false;   % true/false, if true, vehicles do not need to consider other vehicles.
        fallback_type = 'localFallback'; % one of the following {'no','local','global'}, 
                                            % 'no' for disable fallback; 
                                            % 'global' means once a vehicle triggers fallback, all other vehicles must also take fallback.
                                            % 'local' means once a vehicle triggers fallback, only vehicles that have direct or undirected couplings with it will take fallabck. 
        
        veh_ids = [];                           % vehicle IDs
        random_idx = [];                        % integer, random choose different vehicles
        isDealPredictionInconsistency = true;   % true/false, if ture, reachability analysis will be used to deal with the problem of prediction inconsistency; otherwise, one-step delayed trajectories will be considered
        is_allow_non_convex = true;             % true/false, whether to allow non-convex polygons; if true, the separating axis theorem cannot be used since it works only for convex polygons. `InterX.m` can be used instead.
        recursive_feasibility = true;           % true/false, if true, the last trim must be an equilibrium trims
        time_per_tick = 0.01;
        offset = 0.01;
        plot_limits = [-10,10;-10,10];          % default fallback if not defined
        is_use_dynamic_programming = true;      % true/false, use dynamic programming or brute-force approach to calculate local reachable sets

        % MPA
        is_save_mpa = true;             % true/false, the offline computed MPA will be saved if true
        is_load_mpa = true;             % true/false, the offline computed MPA  will be load if exists
        coupling_weight_mode = 'STAC';  % one of the following {'STAC','random','constant','optimal'}

        optionsPlotOnline = OptionsPlotOnline;      % setup for online plotting
        bound_reachable_sets = true;                % true/false, if true, reachable sets are bounded by lanelet boundaries

        is_force_parallel_vehs_in_same_grp = true;  % true/false, if true, vehicles move in parallel will be forced in the same group
        reference_path = struct('lanelets_index',[],'start_point',[]);  % custom reference path
        visualizeReferenceTrajectory
    end

    properties(Dependent)
        tick_per_step   % number of data points per step
        k_end           % total number of steps
        Hu              % control horizon
    end

    methods
        function obj = OptionsMain()
        end

        function obj = assign_data(obj,options)
            % Assign data to options (see an example at the bottom)
            obj.manualVehicle_id = options.manualVehicle_id;
            obj.firstManualVehicleMode = options.firstManualVehicleMode;
            obj.manualVehicle_id2 = options.manualVehicle_id2;
            obj.secondManualVehicleMode = options.secondManualVehicleMode;
            obj.collisionAvoidanceMode = options.collisionAvoidanceMode;
            obj.is_sim_lab = options.is_sim_lab;
            obj.is_mixed_traffic = options.is_mixed_traffic;
            obj.force_feedback_enabled = options.force_feedback_enabled;
            obj.consider_RSS = options.consider_RSS;
            obj.isPB = options.isPB;
            obj.angles = options.angles;
            obj.amount = options.amount;
            obj.visu = options.visu;
            obj.isParl = options.isParl;
            obj.scenario = options.scenario_name;
            obj.priority = options.priority;
            obj.dt = options.dt;
            obj.Hp = options.Hp;
            obj.trim_set = options.trim_set;
            obj.T_end = options.T_end;
            obj.max_num_CLs = options.max_num_CLs;
            obj.strategy_consider_veh_without_ROW = options.strategy_consider_veh_without_ROW;
            obj.strategy_enter_lanelet_crossing_area = options.strategy_enter_lanelet_crossing_area;
            obj.isSaveResult = options.isSaveResult;
            obj.customResultName = options.customResultName;
            obj.isAllowInheritROW = options.isAllowInheritROW;
            obj.is_eval = options.is_eval;
            obj.visualize_reachable_set = options.visualize_reachable_set;
            obj.is_free_flow = options.is_free_flow;
        end

        function result = get.tick_per_step(obj)
            result = round(obj.dt/obj.time_per_tick);
        end

        function result = get.k_end(obj)
            result = floor(obj.T_end/obj.dt);
        end

        function result = get.Hu(obj)
            result = obj.Hp;
        end

    end
end

%% Example
% options = struct;
% options.manualVehicle_id = '1';
% options.firstManualVehicleMode = '1';
% options.manualVehicle_id2 = '2';
% options.secondManualVehicleMode = '1';
% options.collisionAvoidanceMode = 1;
% options.is_sim_lab = true;
% options.is_mixed_traffic = false;
% options.force_feedback_enabled = true;
% options.consider_RSS = false;
% options.isPB = true;
% options.angles = [];
% options.amount = 3;
% options.visu = [true,false];
% options.isParl = false;
% options.scenario_name = 'Commonroad';
% options.priority = 'STAC_priority';
% options.dt = 0.2;
% options.Hp = 5;
% options.trim_set = 9;
% options.T_end = 30;
% options.max_num_CLs = 4;
% options.strategy_consider_veh_without_ROW = '3';
% options.strategy_enter_lanelet_crossing_area = '4';
% options.isSaveResult = false;
% options.customResultName = '';
% options.isAllowInheritROW = true;
% options.is_eval = false;
% options.visualize_reachable_set = false;

% optionsMain = OptionsMain().assign_data(options)
% % call main.m
% main(optionsMain)