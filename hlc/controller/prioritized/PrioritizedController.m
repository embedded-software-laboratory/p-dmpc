classdef (Abstract) PrioritizedController < HighLevelController

    properties (Access = public)
        % instances of communication classes stored in a cell array
        % contains one instance for each vehicle that is controlled by the hlc
        % parallel/distributed execution: same size but only on entry that is not empty
        traffic_communication (1, :) cell
        predictions_communication (1, :) cell
    end

    properties (Access = protected)
        CL_based_hierarchy;
        lanelet_crossing_areas;
        prioritizer
        weighter
        coupler

        consider_parallel_coupling (1, 1) function_handle = @()[];
    end

    methods

        function obj = PrioritizedController(scenario, plant)
            obj = obj@HighLevelController(scenario, plant);

            if obj.scenario.options.use_cpp()
                obj.optimizer = GraphSearchMexPB(obj.scenario, obj.mpa, obj.plant.indices_in_vehicle_list);
            else
                obj.optimizer = GraphSearch(obj.scenario, obj.mpa);
            end

            obj.coupler = Coupler();
            obj.prioritizer = Prioritizer.get_prioritizer(obj.scenario.options.priority);
            obj.weighter = Weighter.get_weighter(obj.scenario.options.weight);

            if obj.scenario.options.isDealPredictionInconsistency
                obj.consider_parallel_coupling = @obj.parallel_coupling_reachability;
            else
                obj.consider_parallel_coupling = @obj.parallel_coupling_previous_trajectory;
            end

        end

    end

    methods (Access = protected)

        function init(obj)
            % initialize superclass
            init@HighLevelController(obj);

            % in priority-based computation, vehicles communicate via ROS 2
            timer = tic;
            fprintf('Creating ROS2 objects... ');
            obj.create_ros2_objects();
            fprintf('in %f seconds.\n', toc(timer));

            % initialize the communication network of ROS 2
            obj.init_communication();
        end

        function create_ros2_objects(obj)

            for vehicle_index = obj.plant.indices_in_vehicle_list
                vehicle_id = obj.plant.all_vehicle_ids(vehicle_index);

                % create instance of the communication class
                obj.traffic_communication{vehicle_index} = TrafficCommunication();
                obj.predictions_communication{vehicle_index} = PredictionsCommunication();

                % create node and store topic name and message type
                obj.traffic_communication{vehicle_index}.initialize( ...
                    vehicle_id, ...
                    'traffic', ...
                    '/vehicle_traffic', ...
                    'veh_msgs/Traffic' ...
                );
                obj.predictions_communication{vehicle_index}.initialize( ...
                    vehicle_id, ...
                    'prediction', ...
                    '/vehicle_prediction', ...
                    'veh_msgs/Predictions' ...
                );

                % create publishers
                obj.traffic_communication{vehicle_index}.create_publisher();
                obj.predictions_communication{vehicle_index}.create_publisher();

                % create subscribers
                obj.traffic_communication{vehicle_index}.create_subscriber();
                obj.predictions_communication{vehicle_index}.create_subscriber();
            end

            if length(obj.plant.indices_in_vehicle_list) == 1
                % wait for all subscribers to be created in distributed case,
                % because otherwise early sent messages will be lost.
                pause(5.0);
            end

        end

        function init_communication(obj)
            % communicate initial traffic and predictions message
            % synchronize the prioritized controllers in parallel/distributed execution

            % create struct with state indices that it is not created on every usage
            state_indices = indices();
            % measure vehicles' initial poses and trims
            [states_measured, trims_measured] = obj.plant.measure(obj.mpa);

            for vehicle_index = obj.plant.indices_in_vehicle_list
                % store state and trim in iteration data
                obj.iter.x0(vehicle_index, :) = states_measured(vehicle_index, :);
                obj.iter.trim_indices(vehicle_index) = trims_measured(vehicle_index);

                % calculate predicted lanelets
                predicted_lanelets = get_predicted_lanelets( ...
                    obj.scenario, ...
                    obj.mpa, ...
                    obj.iter, ...
                    vehicle_index, ...
                    states_measured(vehicle_index, state_indices.x), ...
                    states_measured(vehicle_index, state_indices.y) ...
                );

                % get vehicle size
                length = obj.scenario.vehicles(vehicle_index).Length;
                width = obj.scenario.vehicles(vehicle_index).Width;

                % get vehicles currently occupied area in vehicle frame
                % repeat first entry at the end to enclose the shape
                x_local_with_offset = [-1, -1, 1, 1, -1] * (length / 2 + obj.scenario.options.offset);
                y_local_with_offset = [-1, 1, 1, -1, -1] * (width / 2 + obj.scenario.options.offset);
                x_local_without_offset = [-1, -1, 1, 1, -1] * (length / 2);
                y_local_without_offset = [-1, 1, 1, -1, -1] * (width / 2);

                % calculate areas in global frame
                [x_global_with_offset, y_global_with_offset] = translate_global( ...
                    states_measured(vehicle_index, state_indices.heading), ...
                    states_measured(vehicle_index, state_indices.x), ...
                    states_measured(vehicle_index, state_indices.y), ...
                    x_local_with_offset, ...
                    y_local_with_offset ...
                );
                [x_global_without_offset, y_global_without_offset] = translate_global( ...
                    states_measured(vehicle_index, state_indices.heading), ...
                    states_measured(vehicle_index, state_indices.x), ...
                    states_measured(vehicle_index, state_indices.y), ...
                    x_local_without_offset, ...
                    y_local_without_offset ...
                );

                occupied_area.normal_offset = [x_global_with_offset; y_global_with_offset];
                occupied_area.without_offset = [x_global_without_offset; y_global_without_offset];

                % for initial time step, reachable_sets and predicted areas do not exist yet
                reachable_sets = {};
                predicted_occupied_areas = {};

                % send messages
                obj.traffic_communication{vehicle_index}.send_message( ...
                    obj.k, ...
                    states_measured(vehicle_index, :), ...
                    trims_measured(vehicle_index), ...
                    predicted_lanelets, ...
                    occupied_area, ...
                    reachable_sets ...
                );
                obj.predictions_communication{vehicle_index}.send_message( ...
                    obj.k, ...
                    predicted_occupied_areas ...
                );
            end

            % for synchronization read from all other controllers
            % to ensure that they are ready
            for vehicle_index = obj.plant.indices_in_vehicle_list
                % loop over vehicles that read messages
                other_vehicles = setdiff(1:obj.scenario.options.amount, vehicle_index);

                for vehicle_index_subscribed = other_vehicles
                    % loop over controllers that are subscribed
                    obj.traffic_communication{vehicle_index}.read_message( ...
                        obj.plant.all_vehicle_ids(vehicle_index_subscribed), ...
                        obj.k, ...
                        true, ...
                        40.0 ...
                    );
                    obj.predictions_communication{vehicle_index}.read_message( ...
                        obj.plant.all_vehicle_ids(vehicle_index_subscribed), ...
                        obj.k, ...
                        true, ...
                        40.0 ...
                    );
                end

            end

        end

        function runtime_others = init_step(obj)
            runtime_others_tic = tic;

            determine_couplings_timer = tic;
            obj.couple();
            obj.iter.timer.determine_couplings = toc(determine_couplings_timer);

            assign_priority_timer = tic;
            obj.prioritize();
            obj.iter.timer.assign_priority = toc(assign_priority_timer);

            obj.weigh();

            obj.reduce();

            group_vehicles_timer = tic;
            obj.group();
            obj.iter.timer.group_vehs = toc(group_vehicles_timer);

            nVeh = obj.scenario.options.amount;
            Hp = obj.scenario.options.Hp;

            % initialize variable to store control results
            obj.info = ControlResultsInfo(nVeh, Hp, obj.plant.all_vehicle_ids);

            directed_graph = digraph(obj.iter.directed_coupling);
            [obj.belonging_vector_total, ~] = conncomp(directed_graph, 'Type', 'weak'); % graph decomposition

            obj.iter.num_couplings_between_grps = 0; % number of couplings between groups
            obj.iter.num_couplings_between_grps_ignored = 0; % ignored number of couplings between groups by using lanelet crossing lanelets

            [row_coupling, col_coupling] = find(obj.iter.directed_coupling);
            n_couplings = length(row_coupling);

            for i_coupling = 1:n_couplings
                veh_i = row_coupling(i_coupling);
                veh_j = col_coupling(i_coupling);
                veh_ij = [veh_i, veh_j];
                is_same_grp = any(cellfun(@(c) all(ismember(veh_ij, c)), {obj.iter.parl_groups_info.vertices}));

                if ~is_same_grp
                    obj.iter.num_couplings_between_grps = obj.iter.num_couplings_between_grps + 1;

                    if ~isempty(obj.iter.coupling_info{veh_i, veh_j}) && obj.iter.coupling_info{veh_i, veh_j}.is_virtual_obstacle
                        obj.iter.num_couplings_between_grps_ignored = obj.iter.num_couplings_between_grps_ignored + 1;
                    end

                end

            end

            runtime_others = toc(runtime_others_tic); % subcontroller runtime except for runtime of graph search and msg send time
        end

        function subcontroller_time = plan_single_vehicle(obj, vehicle_idx)
            subcontroller_timer = tic;

            % only keep self
            filter_self = false(1, obj.scenario.options.amount);
            filter_self(vehicle_idx) = true;
            iter_v = filter_iter(obj.iter, filter_self);

            grp_idx = arrayfun(@(array) ismember(vehicle_idx, array.vertices), iter_v.parl_groups_info);
            all_vehs_same_grp = iter_v.parl_groups_info(grp_idx).vertices; % all vehicles in the same group

            all_coupled_vehs_with_HP = find(iter_v.directed_coupling_reduced(:, vehicle_idx) == 1)'; % all coupled vehicles with higher priorities
            all_coupled_vehs_with_LP = find(iter_v.directed_coupling_reduced(vehicle_idx, :) == 1); % all coupled vehicles with lower priorities

            coupled_vehs_same_grp_with_HP = intersect(all_coupled_vehs_with_HP, all_vehs_same_grp); % coupled vehicles with higher priorities in the same group
            coupled_vehs_other_grps_with_HP = setdiff(all_coupled_vehs_with_HP, coupled_vehs_same_grp_with_HP); % coupled vehicles with higher priorities in other groups

            for veh_with_HP_i = all_coupled_vehs_with_HP

                if ismember(veh_with_HP_i, coupled_vehs_same_grp_with_HP)
                    % if in the same group, read the current message and set the predicted occupied areas as dynamic obstacles
                    latest_msg = obj.predictions_communication{vehicle_idx}.read_message( ...
                        obj.plant.all_vehicle_ids(veh_with_HP_i), ...
                        obj.k, ...
                        true ...
                    );
                    obj.info.vehs_fallback = union(obj.info.vehs_fallback, latest_msg.vehs_fallback');

                    if ismember(vehicle_idx, obj.info.vehs_fallback)
                        % if the selected vehicle should take fallback
                        subcontroller_time = toc(subcontroller_timer);
                        obj.info.runtime_graph_search_each_veh(vehicle_idx) = 0;
                        return;
                    end

                    predicted_areas_i = arrayfun(@(array) {[array.x(:)'; array.y(:)']}, latest_msg.predicted_areas);

                    iter_v.dynamic_obstacle_area(end + 1, :) = predicted_areas_i;
                else
                    % if they are in different groups
                    [iter_v, should_fallback] = obj.consider_parallel_coupling(iter_v, vehicle_idx, veh_with_HP_i);

                    if should_fallback
                        subcontroller_time = toc(subcontroller_timer);
                        obj.info.runtime_graph_search_each_veh(vehicle_idx) = 0;
                        return;
                    end

                end

            end

            if ~strcmp(obj.scenario.options.strategy_enter_lanelet_crossing_area, '1')
                % Set lanelet intersecting areas as static obstacles if vehicle with lower priorities is not allowed to enter those area
                iter_v.lanelet_crossing_areas = obj.iter.lanelet_crossing_areas{vehicle_idx};

                if isempty(iter_v.lanelet_crossing_areas)
                    iter_v.lanelet_crossing_areas = {}; % convert from 'double' to 'cell'
                end

                assert(iscell(iter_v.lanelet_crossing_areas));
            end

            % consider coupled vehicles with lower priorities
            iter_v = obj.consider_vehs_with_LP(iter_v, vehicle_idx, all_coupled_vehs_with_LP);

            %% Plan for vehicle vehicle_idx
            % execute sub controller for 1-veh scenario
            [info_v, graph_search_time] = obj.optimizer.run_optimizer(iter_v, vehicle_idx);
            obj.info.runtime_graph_search_each_veh(vehicle_idx) = graph_search_time;

            if info_v.is_exhausted
                info_v = handle_graph_search_exhaustion(info_v, obj.scenario, iter_v, obj.mpa);
            end

            if info_v.needs_fallback
                % if graph search is exhausted, this vehicles and all its weakly coupled vehicles will use their fallback trajectories
                %                 disp(['Graph search exhausted after expending node ' num2str(info_v.n_expanded) ' times for vehicle ' num2str(vehicle_idx) ', at time step: ' num2str(scenario.k) '.'])
                switch obj.scenario.options.fallback_type
                    case FallbackType.local_fallback
                        sub_graph_fallback = obj.belonging_vector_total(vehicle_idx);
                        obj.info.vehs_fallback = [obj.info.vehs_fallback; find(obj.belonging_vector_total == sub_graph_fallback).'];
                        obj.info.vehs_fallback = unique(obj.info.vehs_fallback, 'stable');
                    case FallbackType.global_fallback
                        % global fallback: all vehicles take fallback
                        obj.info.vehs_fallback = int32(1):int32(obj.scenario.options.amount);
                    case FallbackType.no_fallback
                        % Fallback is disabled. Simulation will end.
                        obj.info.vehs_fallback = int32(1):int32(obj.scenario.options.amount);
                    otherwise
                        warning("Please define one of the follows as fallback strategy: 'no_fallback', 'local_fallback', and 'global_fallback'.")
                end

            else
                obj.info = store_control_info(obj.info, info_v, obj.scenario, obj.mpa);
            end

            if obj.iter.k == inf
                plot_obstacles(obj.scenario)
                plot_obstacles(info_v.shapes)
                plot_partitioned_graph(obj.iter.belonging_vector, obj.iter.weighted_coupling, 'ShowWeights', true)
            end

            subcontroller_time = toc(subcontroller_timer);
        end

        function msg_send_time = publish_predictions(obj, vehicle_idx)

            if ~ismember(vehicle_idx, obj.info.vehs_fallback)
                % if the selected vehicle should take fallback

                msg_send_tic = tic;
                predicted_areas_k = obj.info.shapes(vehicle_idx, :);
                % send message
                obj.predictions_communication{vehicle_idx}.send_message( ...
                    obj.k, ...
                    predicted_areas_k, ...
                    obj.info.vehs_fallback ...
                );
                msg_send_time = toc(msg_send_tic);

            else
                msg_send_tic = tic;
                obj.predictions_communication{vehicle_idx}.send_message( ...
                    obj.k, ...
                    {}, ...
                    obj.info.vehs_fallback ...
                );
                msg_send_time = toc(msg_send_tic);
            end

        end

        function couple(obj)
            obj.iter.adjacency = obj.coupler.couple(obj.iter);
            obj.iter.coupling_info = obj.coupler.calculate_coupling_info(obj.scenario, obj.mpa, obj.iter);
        end

        function prioritize(obj)
            obj.iter.directed_coupling = obj.prioritizer.prioritize(obj.scenario, obj.iter);
            obj.iter.priority_list = obj.prioritizer.get_priority_list(obj.iter.directed_coupling);
        end

        function weigh(obj)
            obj.iter.weighted_coupling = obj.weighter.weigh(obj.scenario, obj.mpa, obj.iter);
        end

        function reduce(obj)
            [obj.iter.weighted_coupling_reduced, obj.iter.coupling_info, obj.iter.lanelet_crossing_areas] = ...
                reduce_coupling_lanelet_crossing_area(obj.iter, obj.scenario.options.strategy_enter_lanelet_crossing_area);
            obj.iter.directed_coupling_reduced = (obj.iter.weighted_coupling_reduced ~= 0);
        end

        function group(obj)
            method = 's-t-cut'; % 's-t-cut' or 'MILP'
            [obj.CL_based_hierarchy, obj.iter.parl_groups_info, obj.iter.belonging_vector] = form_parallel_groups(obj.iter.weighted_coupling_reduced, obj.scenario.options.max_num_CLs, obj.iter.coupling_info, method, obj.scenario.options);
        end

        function [iter_v, should_fallback] = parallel_coupling_reachability(obj, iter_v, vehicle_idx, veh_with_HP_i)
            % Collisions with coupled vehicles with higher priorities in different groups will be avoided by two ways depending on the time step at which
            % their latest messages are sent:
            % 1. Their predicted occupied areas will be considered as dynamic obstacles if the latest messages come from the current time step.
            % 2. Their reachable sets will be considered as dynamic obstacles if the latest messages come from past time step.
            should_fallback = false;
            latest_msg = obj.predictions_communication{vehicle_idx}.read_latest_message( ...
                obj.plant.all_vehicle_ids(veh_with_HP_i) ...
            );

            if latest_msg.time_step == obj.k
                obj.info.vehs_fallback = union(obj.info.vehs_fallback, latest_msg.vehs_fallback');

                if ismember(vehicle_idx, obj.info.vehs_fallback)
                    should_fallback = true;
                    return
                end

                predicted_areas_i = arrayfun(@(array) {[array.x(:)'; array.y(:)']}, latest_msg.predicted_areas);
                iter_v.dynamic_obstacle_area(end + 1, :) = predicted_areas_i;

            else
                % Add their reachable sets as dynamic obstacles to deal with the prediction inconsistency
                reachable_sets_i = obj.iter.reachable_sets(veh_with_HP_i, :);
                % turn polyshape to plain array (repeat the first row to enclosed the shape)
                reachable_sets_i_array = cellfun(@(c) {[c.Vertices(:, 1)', c.Vertices(1, 1)'; c.Vertices(:, 2)', c.Vertices(1, 2)']}, reachable_sets_i);
                iter_v.dynamic_obstacle_reachableSets(end + 1, :) = reachable_sets_i_array;
            end

        end

        function [iter_v, should_fallback] = parallel_coupling_previous_trajectory(obj, iter_v, vehicle_idx, veh_with_HP_i)
            should_fallback = false; % no fallback since no recursive feasibility guarantee
            % otherwise add one-step delayed trajectories as dynamic obstacles
            if obj.k > 1
                % the old trajectories are available from the second time step onwards
                old_msg = obj.predictions_communication{vehicle_idx}.read_message( ...
                    obj.plant.all_vehicle_ids(veh_with_HP_i), ...
                    obj.k - 1, ...
                    true ...
                );
                predicted_areas_i = arrayfun(@(array) {[array.x(:)'; array.y(:)']}, old_msg.predicted_areas);
                oldness_msg = obj.k - old_msg.time_step;

                if oldness_msg ~= 0
                    % consider the oldness of the message: delete the first n entries and repeat the last entry for n times
                    predicted_areas_i = del_first_rpt_last(predicted_areas_i', oldness_msg);
                end

                iter_v.dynamic_obstacle_area(end + 1, :) = predicted_areas_i;
            end

        end

        function iter_v = consider_vehs_with_LP(obj, iter_v, vehicle_idx, all_coupling_vehs_without_ROW)
            % CONSIDER_VEHS_WITH_LP Stategies to let vehicle with the right-of-way
            % consider vehicle without the right-of-way
            % '1': do not consider
            % '2': consider currently occupied area as static obstacle
            % '3': consider the occupied area of emergency braking maneuver as static obstacle
            % '4': consider one-step reachable sets as static obstacle
            % '5': consider old trajectory as dynamic obstacle

            for i_LP = 1:length(all_coupling_vehs_without_ROW)
                veh_without_ROW = all_coupling_vehs_without_ROW(i_LP);

                % stategies to let vehicle with the right-of-way consider vehicle without the right-of-way
                switch obj.scenario.options.strategy_consider_veh_without_ROW
                    case '1'
                        % do not consider

                    case '2'
                        % consider currently occupied area as static obstacle
                        iter_v.obstacles{end + 1} = iter_v.occupied_areas{veh_without_ROW}.normal_offset; % add as static obstacles

                    case '3'
                        % consider the occupied area of emergency braking maneuver
                        % as static obstacle (only if their couplings are not
                        % ignored by forbidding one vehicle entering their lanelet
                        % crossing area, and they have side-impact collision
                        % possibility). Cases that vehicles drive successively are not
                        % included to avoid that vehicles behind push vehicles in
                        % front to move forward.
                        switch obj.scenario.options.priority
                            case PriorityStrategies.STAC_priority

                                if ~iter_v.coupling_info{vehicle_idx, veh_without_ROW}.is_ignored && iter_v.coupling_info{vehicle_idx, veh_without_ROW}.collision_type == CollisionType.from_side ...
                                        && iter_v.coupling_info{vehicle_idx, veh_without_ROW}.lanelet_relationship == LaneletRelationshipType.crossing
                                    % the emergency braking maneuver is only considered if
                                    % two coupled vehicles at crossing-adjacent lanelets have side-impact collision that is not ignored
                                    iter_v.obstacles{end + 1} = iter_v.emergency_maneuvers{veh_without_ROW}.braking_area;
                                else
                                    iter_v.obstacles{end + 1} = iter_v.occupied_areas{veh_without_ROW}.normal_offset;
                                end

                            otherwise
                                iter_v.obstacles{end + 1} = iter_v.occupied_areas{veh_without_ROW}.normal_offset;
                        end

                    case '4'
                        % consider one-step reachable sets as static obstacle
                        reachable_sets = iter_v.reachable_sets{veh_without_ROW, 1};
                        % get boundary of the polygon
                        [x_reachable_sets, y_reachable_sets] = boundary(reachable_sets);
                        iter_v.obstacles(end + 1) = {[x_reachable_sets'; y_reachable_sets']};
                    case '5'
                        % consider old trajectory as dynamic obstacle
                        latest_msg = obj.predictions_communication{vehicle_idx}.read_latest_message( ...
                            obj.plant.all_vehicle_ids(veh_without_ROW) ...
                        );

                        if latest_msg.time_step > 0
                            % the message does not come from the initial time step
                            predicted_areas = arrayfun(@(array) {[array.x'; array.y']}, latest_msg.predicted_areas);
                            shift_step = iter_v.k - latest_msg.time_step; % times that the prediction should be shifted and the last prediction should be repeated

                            if shift_step > 1
                                disp(['shift step is ' num2str(shift_step) ', ego vehicle: ' num2str(vehicle_i) ', considered vehicle: ' num2str(veh_without_ROW)])
                            end

                            predicted_areas = del_first_rpt_last(predicted_areas(:)', shift_step);
                            iter_v.dynamic_obstacle_area(end + 1, :) = predicted_areas;
                        end

                    otherwise
                        warning("Please specify one of the following strategies to let vehicle with a higher priority also consider vehicle with a lower priority: '1', '2', '3', '4', '5'.")
                end

            end

        end

        function check_fallback(obj)
            % check for fallback in other controllers

            for vehicle_index_hlc = obj.plant.indices_in_vehicle_list
                % own vehicle and vehicles that are already remembered to take fallback
                irrelevant_vehicles = union(vehicle_index_hlc, obj.info.vehs_fallback);

                if obj.scenario.options.fallback_type == FallbackType.local_fallback
                    sub_graph_fallback = obj.belonging_vector_total(vehicle_index_hlc);
                    % vehicles in the subgraph to check for fallback
                    other_vehicles = find(obj.belonging_vector_total == sub_graph_fallback);
                    % remove irrelevant vehicles which have not to be checked for fallback
                    other_vehicles = setdiff(other_vehicles, irrelevant_vehicles, 'stable');

                    for veh_id = other_vehicles
                        latest_msg = obj.predictions_communication{vehicle_index_hlc}.read_message( ...
                            obj.plant.all_vehicle_ids(veh_id), ...
                            obj.k, ...
                            true ...
                        );
                        fallback_info_veh_id = latest_msg.vehs_fallback';
                        obj.info.vehs_fallback = union(obj.info.vehs_fallback, fallback_info_veh_id);
                    end

                else
                    % vehicles in the total graph to check for fallback
                    other_vehicles = 1:obj.scenario.options.amount;
                    % remove irrelevant vehicles which have not to be checked for fallback
                    other_vehicles = setdiff(other_vehicles, irrelevant_vehicles);

                    for veh_id = other_vehicles
                        latest_msg = obj.predictions_communication{vehicle_index_hlc}.read_message( ...
                            obj.plant.all_vehicle_ids(veh_id), ...
                            obj.k, ...
                            true ...
                        );
                        fallback_info_veh_id = latest_msg.vehs_fallback';
                        obj.info.vehs_fallback = union(obj.info.vehs_fallback, fallback_info_veh_id);
                    end

                end

            end

        end

        function handle_fallback(obj)
            % planning by using last priority and trajectories directly

            tick_per_step = obj.scenario.options.tick_per_step + 1;

            for vehicle_idx = obj.plant.indices_in_vehicle_list

                if ~ismember(vehicle_idx, obj.info.vehs_fallback)
                    continue
                end

                % initialize
                info_v = ControlResultsInfo(1, obj.scenario.options.Hp, obj.plant.all_vehicle_ids(vehicle_idx));

                info_v.tree = obj.info_old.tree{vehicle_idx};
                info_v.tree_path = del_first_rpt_last(obj.info_old.tree_path(vehicle_idx, :));
                info_v.shapes = del_first_rpt_last(obj.info_old.shapes(vehicle_idx, :));
                info_v.predicted_trims = del_first_rpt_last(obj.info_old.predicted_trims(vehicle_idx, :));

                % predicted trajectory of the next time step
                y_pred_v = obj.info_old.y_predicted{vehicle_idx};
                y_pred_v = [y_pred_v(tick_per_step + 1:end, :); y_pred_v(tick_per_step * (obj.scenario.options.Hp - 1) + 1:end, :)];
                info_v.y_predicted = {y_pred_v};

                % prepare output data
                obj.info = store_control_info(obj.info, info_v, obj.scenario, obj.mpa);

                % data only need to be updated if isDealPredictionInconsistency
                % is off, because only old reachable sets but no old predicted areas
                % are used by controller
                if obj.scenario.options.isDealPredictionInconsistency == false
                    % send message
                    obj.predictions_communication{vehicle_idx}.send_message( ...
                        obj.iter.k, ...
                        obj.info.shapes(vehicle_idx, :), ...
                        obj.info.vehs_fallback ...
                    );
                end

            end

        end

        function clean_up(obj)
            % delete ros2 objects
            obj.traffic_communication = {};
            obj.predictions_communication = {};
            % clean up hlc in reverse order than constructing
            clean_up@HighLevelController(obj);
        end

    end

end
