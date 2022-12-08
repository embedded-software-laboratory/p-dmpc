classdef Config

    properties
        is_sim_lab = true;              % true/false, is simulation or lab experiment
        is_mixed_traffic = false;       % true/false, is mixed trafficfc
        mixed_traffic_config Mixed_traffic_config; % mixed traffic config
        isPB = true;            % true/false, is prioritize vehicles
        amount = 20;            % integer, number of vehicles
        angles                  % 1-by-nVeh scalar vector
        visu = [true;false];    % 1-by-2 vector, online plotting is enabled if the first entry if true; node visualization is enabled if the second entry is true
        isParl = false;         % true/false, is use parallel computation
        scenario_name = 'Commonroad'    % one of the follows: {'Circle_scenario','Commonroad'}
        priority Priority_strategies = 'right_of_way_priority'; % one of the following: {'topo_priority','right_of_way_priority','constant_priority','random_priority','FCA_priority','STAC_priority'}, defines which priority assignmen strategy is used
        dt = 0.2;           % scalar, sample time
        Hp = 6;             % scalar, prediction horizon
        trim_set = 7;       % scalar, ID of trim primitives
        T_end = 20;         % scalar, simulation duration
        max_num_CLs = 99;   % integer, maximum allowerd number of computation levels
        strategy_consider_veh_without_ROW = '2';    % one of the following: {'1','2','3','4','5'}, strategy of letting higher-priority vehicles consider their coupled vehicles with lower priorities
                                                        % '1': do not consider 
                                                        % '2': consider currently occupied area as static obstacle
                                                        % '3': consider the occupied area of emergency braking maneuver as static obstacle 
                                                        % '4': consider one-step reachable sets as static obstacle
                                                        % '5': consider old trajectory as dynamic obstacle
        strategy_enter_lanelet_crossing_area = '1'; % one of the following: {'1','2','3','4'}, strategy of forbidding vehicles with lower priorities entering their lanelet crossing area
                                                        % '1': no constraint on entering the crossing area 
                                                        % '2': not allowed to enter the crossing area if they are coupled at intersecting lanelets of the intersection
                                                        % '3': not allowed to enter the crossing area if they are coupled at intersecting or merging lanelets of the intersection
                                                        % '4': not allowed to enter the crossing area if they are coupled at intersecting or merging lanelets regardless whether they are at the intersection or not
        isSaveResult = false;           % true/false, is save result
        isSaveResultReduced = true;     % true/false, if true, reduced result will be save to save disk space (useful for a long run of simulation)
        customResultName = '';          % string or char, custom file name to save result
        isAllowInheritROW = false;      % true/false, is allow vehicles to inherit the right-of-way from their front vehicles

        is_eval = false;                % true/false, 
        visualize_reachable_set = false;    % true/false, 
        is_free_flow = false;           % true/false, if true, vehicles do not need to consider other vehicles.
        fallback_type = 'localFallback';    % one of the following {'no','local','global'}, 
                                            % 'no' for disable fallback; 
                                            % 'global' means once a vehicle triggers fallback, all other vehicles must also take fallback.
                                            % 'local' means once a vehicle triggers fallback, only vehicles that have direct or undirected couplings with it will take fallabck. 
        
        veh_ids = [];                           % vehicle IDs
        random_idx = [];                        % integer, random choose different vehicles
        isDealPredictionInconsistency = true;   % true/false, if true, reachability analysis will be used to deal with the problem of prediction inconsistency; otherwise, one-step delayed trajectories will be considered
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
        visualizeReferenceTrajectory = false;

    end

    properties(Dependent)
        tick_per_step   % number of data points per step
        k_end           % total number of steps
        Hu              % control horizon
    end

    methods
        function obj = Config()
        end

        function obj = assign_data(obj,struct)
            % Assign data to struct (see an example at the bottom)
            obj.is_sim_lab = struct.is_sim_lab;
            obj.is_mixed_traffic = struct.is_mixed_traffic;
            obj.isPB = struct.isPB;
            obj.angles = struct.angles;
            obj.amount = struct.amount;
            obj.visu = struct.visu;
            obj.isParl = struct.isParl;
            obj.scenario_name = struct.scenario_name;
            obj.priority = struct.priority;
            obj.dt = struct.dt;
            obj.Hp = struct.Hp;
            obj.trim_set = struct.trim_set;
            obj.T_end = struct.T_end;
            obj.max_num_CLs = struct.max_num_CLs;
            obj.strategy_consider_veh_without_ROW = struct.strategy_consider_veh_without_ROW;
            obj.strategy_enter_lanelet_crossing_area = struct.strategy_enter_lanelet_crossing_area;
            obj.isSaveResult = struct.isSaveResult;
            obj.customResultName = struct.customResultName;
            obj.isAllowInheritROW = struct.isAllowInheritROW;
            obj.is_eval = struct.is_eval;
            obj.visualize_reachable_set = struct.visualize_reachable_set;
            obj.is_free_flow = struct.is_free_flow;
            %create and set mixed_traffic_config object
            mt_config = Mixed_traffic_config();
            mt_config = mt_config.assign_data(struct.mixed_traffic_config);
            obj.mixed_traffic_config = mt_config;
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

        function result = exportAsJson(obj)
            result = jsonencode(obj);
        end

        function result = importFromJson(obj, json)
            result = assign_data(obj,jsondecode(json));
        end

    end
end
