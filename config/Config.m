classdef Config < matlab.mixin.Copyable

    properties
        environment = Environment.Simulation; % NOTE: Replacement of "is_sim_lab". Does now have three optinos (see Environment enum).
        manual_control_config ManualControlConfig = ManualControlConfig(); % manual control config
        is_prioritized = true; % true/false, is prioritize vehicles
        amount = 20; % integer, number of vehicles, does not include manual vehicles
        compute_in_parallel = false; % true/false, is use parallel(distributed) computation
        scenario_type ScenarioType = ScenarioType.commonroad; % one of the follows: {'Circle_scenario', 'Commonroad'}
        priority PriorityStrategies = PriorityStrategies.constant_priority; % defines which priority assignmen strategy is used
        weight WeightStrategies = WeightStrategies.constant_weight; % defines which weighting method is used
        dt_seconds = 0.2; % scalar, default sample time
        Hp = 6; % scalar, prediction horizon
        trim_set = 7; % scalar, ID of trim primitives
        T_end = 20; % scalar, simulation duration
        max_num_CLs = 99; % integer, maximum allowerd number of computation levels
        strategy_consider_veh_without_ROW = '2'; % one of the following: {'1', '2', '3', '4', '5'}, strategy of letting higher - priority vehicles consider their coupled vehicles with lower priorities
        % '1': do not consider
        % '2': consider currently occupied area as static obstacle
        % '3': consider the occupied area of emergency braking maneuver as static obstacle
        % '4': consider one-step reachable sets as static obstacle
        % '5': consider old trajectory as dynamic obstacle
        strategy_enter_lanelet_crossing_area = '1'; % one of the following: {'1', '2', '3', '4'}, strategy of forbidding vehicles with lower priorities entering their lanelet crossing area
        % '1': no constraint on entering the crossing area
        % '2': not allowed to enter the crossing area if they are coupled at intersecting lanelets of the intersection
        % '3': not allowed to enter the crossing area if they are coupled at intersecting or merging lanelets of the intersection
        % '4': not allowed to enter the crossing area if they are coupled at intersecting or merging lanelets regardless whether they are at the intersection or not
        should_save_result = true; % true/false, is save result
        should_reduce_result = true; % true/false, if true, reduced result will be save to save disk space (useful for a long run of simulation)
        result_name = ''; % string or char, custom file name to save result
        allow_priority_inheritance = false; % true/false, is allow vehicles to inherit the right-of-way from their front vehicles

        is_eval = false; % true/false,
        is_free_flow = false; % true/false, if true, vehicles do not need to consider other vehicles.
        fallback_type FallbackType = FallbackType.local_fallback; % one of the following {'no', 'local', 'global'},
        % 'no' for disable fallback;
        % 'global' means once a vehicle triggers fallback, all other vehicles must also take fallback.
        % 'local' means once a vehicle triggers fallback, only vehicles that have direct or undirected couplings with it will take fallabck.

        path_ids = []; % reference path IDs for selection of paths for the vehicles
        random_idx = []; % integer, random choose different vehicles
        isDealPredictionInconsistency = true; % true/false, if true, reachability analysis will be used to deal with the problem of prediction inconsistency; otherwise, one-step delayed trajectories will be considered
        is_allow_non_convex = true; % true/false, whether to allow non-convex polygons; if true, the separating axis theorem cannot be used since it works only for convex polygons. `InterX.m` can be used instead.
        recursive_feasibility = true; % true/false, if true, the last trim must be an equilibrium trims
        time_per_tick = 0.01;
        offset = 0.01;
        plot_limits = [-10, 10; -10, 10]; % default fallback if not defined
        is_use_dynamic_programming = true; % true/false, use dynamic programming or brute-force approach to calculate local reachable sets

        % MPA
        is_save_mpa = true; % true/false, the offline computed MPA will be saved if true
        is_load_mpa = true; % true/false, the offline computed MPA  will be load if exists

        options_plot_online = OptionsPlotOnline(); % setup for online plotting
        bound_reachable_sets = true; % true/false, if true, reachable sets are bounded by lanelet boundaries

        is_force_parallel_vehs_in_same_grp = true; % true/false, if true, vehicles move in parallel will be forced in the same group
        reference_path = struct('lanelets_index', [], 'start_point', []); % custom reference path
        cpp_optimizer CppOptimizer = CppOptimizer.None;
        mex_out_of_process_execution = false; % execute mex graph search functions in own process

    end

    properties (Dependent)
        tick_per_step % number of data points per step
        k_end % total number of steps
        Hu % control horizon
    end

    methods

        function obj = Config()
        end

        function obj = assign_data(obj, struct)
            fn = fieldnames(struct);

            for i_field = 1:length(fn)
                field = fn{i_field};

                if ~isprop(obj, field)
                    warning('Cannot set property %s for class Config as it does not exist', field);
                    continue;
                end

                if findprop(obj, field).Dependent
                    warning('Cannot set property %s for class Config as it is a dependent property', field);
                    continue;
                end

                if strcmp(field, 'manual_control_config')
                    obj.manual_control_config = ManualControlConfig();
                    obj.manual_control_config = obj.manual_control_config.assign_data(struct.manual_control_config);
                elseif strcmp(field, 'options_plot_online')
                    obj.options_plot_online = OptionsPlotOnline();
                    obj.options_plot_online = obj.options_plot_online.assign_data(struct.options_plot_online);
                else
                    obj.(field) = struct.(field);
                end

            end

        end

        function result = get.tick_per_step(obj)
            result = round(obj.dt_seconds / obj.time_per_tick);
        end

        function result = get.k_end(obj)
            result = floor(obj.T_end / obj.dt_seconds);
        end

        function result = get.Hu(obj)
            result = obj.Hp;
        end

        function result = exportAsJson(obj)
            result = jsonencode(obj);
        end

        function result = importFromJson(obj, json)
            result = assign_data(obj, jsondecode(json));
        end

        function result = use_cpp(obj)
            result = obj.cpp_optimizer ~= CppOptimizer.None;
        end

    end

end
