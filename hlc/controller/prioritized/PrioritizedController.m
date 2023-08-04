classdef (Abstract) PrioritizedController < HighLevelController

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

            if obj.scenario.options.use_cpp
                obj.optimizer = GraphSearchMexPB(obj.scenario, obj.plant.indices_in_vehicle_list);
            else
                obj.optimizer = GraphSearch(obj.scenario);
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
            obj.info = ControlResultsInfo(nVeh, Hp, obj.plant.all_veh_ids);

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

                    if ~isempty(obj.iter.coupling_info{veh_i, veh_j}) && obj.iter.coupling_info{veh_i, veh_j}.is_ignored
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
                    latest_msg = read_message(obj.scenario.vehicles(vehicle_idx).communicate.predictions, obj.ros_subscribers.predictions{veh_with_HP_i}, obj.k, true);
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
            iter_v = consider_vehs_with_LP(obj.scenario, iter_v, vehicle_idx, all_coupled_vehs_with_LP, obj.ros_subscribers);

            %% Plan for vehicle vehicle_idx
            % execute sub controller for 1-veh scenario
            [info_v, graph_search_time] = obj.optimizer.run_optimizer(iter_v, vehicle_idx);
            obj.info.runtime_graph_search_each_veh(vehicle_idx) = graph_search_time;

            if info_v.is_exhausted
                info_v = handle_graph_search_exhaustion(info_v, obj.scenario, iter_v);
            end

            if info_v.needs_fallback
                % if graph search is exhausted, this vehicles and all its weakly coupled vehicles will use their fallback trajectories
                %                 disp(['Graph search exhausted after expending node ' num2str(info_v.n_expanded) ' times for vehicle ' num2str(vehicle_idx) ', at time step: ' num2str(scenario.k) '.'])
                switch obj.scenario.options.fallback_type
                    case FallbackType.local_fallback
                        sub_graph_fallback = obj.belonging_vector_total(vehicle_idx);
                        obj.info.vehs_fallback = [obj.info.vehs_fallback, find(obj.belonging_vector_total == sub_graph_fallback)];
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
                obj.info = store_control_info(obj.info, info_v, obj.scenario);
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
                obj.scenario.vehicles(vehicle_idx).communicate.predictions.send_message(obj.k, predicted_areas_k, obj.info.vehs_fallback);
                msg_send_time = toc(msg_send_tic);

            else
                msg_send_tic = tic;
                obj.scenario.vehicles(vehicle_idx).communicate.predictions.send_message(obj.k, {}, obj.info.vehs_fallback);
                msg_send_time = toc(msg_send_tic);
            end

        end

        function couple(obj)
            obj.iter.adjacency = obj.coupler.couple(obj.iter);
            obj.iter.coupling_info = obj.coupler.calculate_coupling_info(obj.scenario, obj.iter);
        end

        function prioritize(obj)
            obj.iter.directed_coupling = obj.prioritizer.prioritize(obj.scenario, obj.iter);
            obj.iter.priority_list = obj.prioritizer.get_priority_list(obj.iter.directed_coupling);
        end

        function weigh(obj)
            obj.iter.weighted_coupling = obj.weighter.weigh(obj.scenario, obj.iter);
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
            latest_msg = obj.ros_subscribers.predictions{veh_with_HP_i}.LatestMessage;

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
                old_msg = read_message(obj.scenario.vehicles(vehicle_idx).communicate.predictions, obj.ros_subscribers.predictions{veh_with_HP_i}, obj.k - 1, false);
                predicted_areas_i = arrayfun(@(array) {[array.x(:)'; array.y(:)']}, old_msg.predicted_areas);
                oldness_msg = obj.k - old_msg.time_step;

                if oldness_msg ~= 0
                    % consider the oldness of the message: delete the first n entries and repeat the last entry for n times
                    predicted_areas_i = del_first_rpt_last(predicted_areas_i', oldness_msg);
                end

                iter_v.dynamic_obstacle_area(end + 1, :) = predicted_areas_i;
            end

        end

    end

end
