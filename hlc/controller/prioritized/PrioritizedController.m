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

        prioritizer
        weigher
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
            obj.weigher = Weigher.get_weigher(obj.scenario.options.weight);

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

                if obj.scenario.options.scenario_type == ScenarioType.circle
                    % In circle scenarios there are no lanelets
                    predicted_lanelets = [];
                else
                    % Compute the reference path
                    reference = get_reference_trajectory( ...
                        obj.mpa, ...
                        obj.scenario.vehicles(vehicle_index).reference_path, ...
                        states_measured(vehicle_index, state_indices.x), ...
                        states_measured(vehicle_index, state_indices.y), ...
                        trims_measured(vehicle_index), ...
                        obj.scenario.options.dt_seconds ...
                    );

                    % Compute the predicted lanelets of iVeh vehicle
                    predicted_lanelets = get_predicted_lanelets(obj.scenario, vehicle_index, reference);
                end

                % get vehicles currently occupied areas
                occupied_areas = obj.scenario.vehicles(vehicle_index).get_occupied_areas( ...
                    states_measured(vehicle_index, state_indices.x), ...
                    states_measured(vehicle_index, state_indices.y), ...
                    states_measured(vehicle_index, state_indices.heading), ...
                    obj.scenario.options.offset ...
                );

                % for initial time step, reachable_sets and predicted areas do not exist yet
                reachable_sets = {};
                predicted_occupied_areas = {};

                % send messages
                obj.traffic_communication{vehicle_index}.send_message( ...
                    obj.k, ...
                    states_measured(vehicle_index, :), ...
                    trims_measured(vehicle_index), ...
                    predicted_lanelets, ...
                    occupied_areas, ...
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

        function update_controlled_vehicles_traffic_info(obj, states_measured, trims_measured)
            % compute vehicles traffic info in HighLevelController
            update_controlled_vehicles_traffic_info@HighLevelController(obj, states_measured, trims_measured);

            for iVeh = obj.plant.indices_in_vehicle_list
                % Send data to sync obj.iter for all vehicles
                obj.traffic_communication{iVeh}.send_message( ...
                    obj.k, ...
                    obj.iter.x0(iVeh, :), ...
                    obj.iter.trim_indices(iVeh), ...
                    obj.iter.predicted_lanelets{iVeh}, ...
                    obj.iter.occupied_areas{iVeh}, ...
                    obj.iter.reachable_sets(iVeh, :) ...
                );
            end

        end

        function update_other_vehicles_traffic_info(obj)
            % read the traffic messages from the other vehicles and
            % store the information in the IterationData object

            % create index struct only once for efficiency
            state_index = indices();

            % index of communication class
            % note: only one is taken since each communication class receives
            % the messages from all other vehicles
            jVeh = obj.plant.indices_in_vehicle_list(1);

            % read messages from other vehicles
            other_vehicle_indices = setdiff(1:obj.scenario.options.amount, obj.plant.indices_in_vehicle_list);

            % loop over vehicle from which the messages are read
            for kVeh = other_vehicle_indices
                latest_msg_i = obj.traffic_communication{jVeh}.read_message( ...
                    obj.plant.all_vehicle_ids(kVeh), ...
                    obj.k, ...
                    true ...
                );

                % take state and trim of vehicle kVeh
                obj.iter.x0(kVeh, :) = [latest_msg_i.current_pose.x, latest_msg_i.current_pose.y, latest_msg_i.current_pose.heading, latest_msg_i.current_pose.speed];
                obj.iter.trim_indices(kVeh) = latest_msg_i.current_trim_index;

                % transform occupied areas
                occupied_areas = latest_msg_i.occupied_areas;
                obj.iter.occupied_areas{kVeh}.normal_offset(1, :) = occupied_areas(1).x;
                obj.iter.occupied_areas{kVeh}.normal_offset(2, :) = occupied_areas(1).y;
                obj.iter.occupied_areas{kVeh}.without_offset(1, :) = occupied_areas(2).x;
                obj.iter.occupied_areas{kVeh}.without_offset(2, :) = occupied_areas(2).y;

                % transform reachable sets to polyshape object
                obj.iter.reachable_sets(kVeh, :) = (arrayfun(@(array) {polyshape(array.x, array.y)}, latest_msg_i.reachable_sets))';

                % transform predicted lanelets
                obj.iter.predicted_lanelets{kVeh} = latest_msg_i.predicted_lanelets';

                % calculate the predicted lanelet boundary of vehicle kVeh based on its predicted lanelets
                if obj.scenario.options.scenario_type ~= ScenarioType.circle
                    obj.iter.predicted_lanelet_boundary(kVeh, :) = get_lanelets_boundary( ...
                        obj.iter.predicted_lanelets{kVeh}, ...
                        obj.scenario.lanelet_boundary, ...
                        obj.scenario.vehicles(kVeh).lanelets_index, ...
                        obj.scenario.vehicles(kVeh).is_loop ...
                    );
                end

                % get occupied areas of emergency maneuvers for vehicle kVeh
                obj.iter.emergency_maneuvers{kVeh} = obj.mpa.emergency_maneuvers_at_pose( ...
                    obj.iter.x0(kVeh, state_index.x), ...
                    obj.iter.x0(kVeh, state_index.y), ...
                    obj.iter.x0(kVeh, state_index.heading), ...
                    obj.iter.trim_indices(kVeh) ...
                );
            end

        end

        function create_coupling_graph(obj)

            obj.update_other_vehicles_traffic_info();

            obj.timing.start("determine_couplings_time", obj.k);
            obj.couple();
            obj.timing.stop("determine_couplings_time", obj.k);

            obj.timing.start("assign_priority_time", obj.k);
            obj.prioritize();
            obj.timing.stop("assign_priority_time", obj.k);

            obj.reduce_computation_levels();

        end

        function plan_single_vehicle(obj, vehicle_idx)

            % only keep self
            filter_self = false(1, obj.scenario.options.amount);
            filter_self(vehicle_idx) = true;
            iter_v = filter_iter(obj.iter, filter_self);

            all_coupled_vehs_with_HP = find(iter_v.directed_coupling_reduced(:, vehicle_idx) == 1)'; % all coupled vehicles with higher priorities
            all_coupled_vehs_with_LP = find(iter_v.directed_coupling_reduced(vehicle_idx, :) == 1); % all coupled vehicles with lower priorities

            % consider vehicles with higher priority
            [dynamic_obstacle_area_HP, is_fallback_triggered] = consider_vehs_with_HP(obj, vehicle_idx, all_coupled_vehs_with_HP);

            % if vehicle with higher priority triggered fallback, do not plan
            if is_fallback_triggered
                return
            end

            % consider coupled vehicles with lower priorities
            [obstacles_LP, dynamic_obstacle_area_LP] = obj.consider_vehs_with_LP(vehicle_idx, all_coupled_vehs_with_LP);

            % add obstacles for considering other vehicles
            iter_v.obstacles = [iter_v.obstacles; obstacles_LP];
            iter_v.dynamic_obstacle_area = [iter_v.dynamic_obstacle_area; dynamic_obstacle_area_HP; dynamic_obstacle_area_LP];

            %% Plan for vehicle vehicle_idx
            % execute sub controller for 1-veh scenario
            [info_v, graph_search_time] = obj.optimizer.run_optimizer(iter_v, vehicle_idx);
            obj.info.runtime_graph_search_each_veh(vehicle_idx) = graph_search_time;

            if info_v.is_exhausted
                info_v = handle_graph_search_exhaustion(info_v, obj.scenario, iter_v, obj.mpa);
            end

            if info_v.needs_fallback
                % if graph search is exhausted, this vehicles and all its weakly coupled vehicles will use their fallback trajectories

                switch obj.scenario.options.fallback_type
                    case FallbackType.local_fallback
                        % local fallback: only vehicles in same subgraph take fallback
                        belonging_vector_total = conncomp(digraph(obj.iter.directed_coupling), 'Type', 'weak');
                        sub_graph_fallback = belonging_vector_total(vehicle_idx);
                        obj.info.vehs_fallback = [obj.info.vehs_fallback; find(belonging_vector_total == sub_graph_fallback).'];
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

        end

        function publish_predictions(obj, vehicle_idx)

            if ~ismember(vehicle_idx, obj.info.vehs_fallback)
                % if the selected vehicle should take fallback
                predicted_areas_k = obj.info.shapes(vehicle_idx, :);
                % send message
                obj.predictions_communication{vehicle_idx}.send_message( ...
                    obj.k, ...
                    predicted_areas_k, ...
                    obj.info.vehs_fallback ...
                );
            else
                obj.predictions_communication{vehicle_idx}.send_message( ...
                    obj.k, ...
                    {}, ...
                    obj.info.vehs_fallback ...
                );
            end

        end

        function couple(obj)
            obj.iter.adjacency = obj.coupler.couple(obj.iter);
            obj.iter.coupling_info = obj.coupler.calculate_coupling_info(obj.k, obj.scenario, obj.mpa, obj.iter);
        end

        function prioritize(obj)
            obj.iter.directed_coupling = obj.prioritizer.prioritize(obj.k, obj.scenario, obj.iter);
            obj.iter.priority_list = obj.prioritizer.get_priority_list(obj.iter.directed_coupling);
        end

        function reduce_computation_levels(obj)
            % weigh
            obj.iter.weighted_coupling = obj.weigher.weigh(obj.k, obj.scenario, obj.mpa, obj.iter);

            % reduce by replacing with lanelet crossing area obstacles
            obj.iter.directed_coupling_reduced = obj.iter.directed_coupling;
            obj.iter.weighted_coupling_reduced = obj.iter.weighted_coupling;

            if obj.scenario.options.scenario_type ~= ScenarioType.circle
                % reduce only if no circle scenario

                % get ignored couplings and lanelet_crossing_areas
                [ ...
                     obj.iter.coupling_info, ...
                     obj.iter.directed_coupling_reduced, ...
                     obj.iter.weighted_coupling_reduced, ...
                     obj.iter.lanelet_crossing_areas ...
                 ] = reduce_coupling_lanelet_crossing_area( ...
                    obj.iter, ...
                    obj.scenario.options.strategy_enter_lanelet_crossing_area ...
                );

            end

            obj.timing.start("group_vehs_time", obj.k);
            % reduce by grouping and cutting edges
            method = 's-t-cut'; % 's-t-cut' or 'MILP'
            [obj.CL_based_hierarchy, obj.iter.parl_groups_info, obj.iter.belonging_vector ...
             ] = form_parallel_groups( ...
                obj.iter.weighted_coupling_reduced, ...
                obj.scenario.options.max_num_CLs, ...
                obj.iter.coupling_info, ...
                method, ...
                obj.scenario.options ...
            );
            obj.timing.stop("group_vehs_time", obj.k);

        end

        function dynamic_obstacle_area = parallel_coupling_reachability(obj, ~, veh_with_HP_i)
            % collisions with coupled vehicles with higher priorities in
            % different groups will be avoided by considering
            % their reachable sets as dynamic obstacles
            %
            % Output:
            %   dynamic_obstacle_area (1, Hp) cell of areas [x; y]

            arguments
                obj (1, 1) PrioritizedController
                ~% index of the current vehicle
                veh_with_HP_i (1, 1) double % index of the vehicle with higher priority
            end

            % Add their reachable sets as dynamic obstacles to deal with the prediction inconsistency
            reachable_sets_i = obj.iter.reachable_sets(veh_with_HP_i, :);
            % turn polyshape to plain array (repeat the first row to enclosed the shape)
            reachable_sets_i_cell_array = cellfun(@(c) {[c.Vertices(:, 1)', c.Vertices(1, 1)'; c.Vertices(:, 2)', c.Vertices(1, 2)']}, reachable_sets_i);
            dynamic_obstacle_area = reachable_sets_i_cell_array;

        end

        function dynamic_obstacle_area = parallel_coupling_previous_trajectory(obj, vehicle_idx, veh_with_HP_i)
            % collisions with coupled vehicles with higher priorities in
            % different groups will be avoided by considering
            % their one-step delayed predicted trajectories as dynamic obstacle
            %
            % Output:
            %   dynamic_obstacle_area (1, Hp) cell of areas [x; y]

            arguments
                obj (1, 1) PrioritizedController
                vehicle_idx (1, 1) double % index of the current vehicle
                veh_with_HP_i (1, 1) double % index of the vehicle with higher priority
            end

            % initialize the returned variable with dimension 0 that it does
            % not extend the list of dynamic_obstacle_area
            dynamic_obstacle_area = cell(0, obj.scenario.options.Hp);

            if obj.k <= 1
                return
            end

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

            dynamic_obstacle_area = predicted_areas_i;

        end

        function [dynamic_obstacle_area, is_fallback_triggered] = consider_vehs_with_HP(obj, vehicle_idx, all_coupled_vehs_with_HP)
            % consider_vehs_with_HP considering vehicles with higher priority
            % strategy depends on whether the vehicles are in the same group
            % and if the prediction consistency is should be guaranteed
            %
            % Output:
            %   dynamic_obstacle_area (1, Hp) cell of areas [x; y]
            %   is_fallback_triggered (1, 1) logical
            %       true if higher prioritized vehicle has a fallback

            arguments
                obj (1, 1) PrioritizedController
                vehicle_idx (1, 1) double % index of the current vehicle
                all_coupled_vehs_with_HP double % indices of the vehicles with higher priority
            end

            is_fallback_triggered = false;

            % preallocate cell array entries
            dynamic_obstacle_area = cell(length(all_coupled_vehs_with_HP), obj.scenario.options.Hp);

            grp_idx = arrayfun(@(array) ismember(vehicle_idx, array.vertices), obj.iter.parl_groups_info);
            all_vehs_same_grp = obj.iter.parl_groups_info(grp_idx).vertices; % all vehicles in the same group

            % coupled vehicles with higher priorities in the same group
            coupled_vehs_same_grp_with_HP = intersect(all_coupled_vehs_with_HP, all_vehs_same_grp);

            for i_vehicle = 1:length(all_coupled_vehs_with_HP)
                veh_with_HP_i = all_coupled_vehs_with_HP(i_vehicle);

                if ismember(veh_with_HP_i, coupled_vehs_same_grp_with_HP)
                    % if in the same group, read the current message and set the
                    % predicted occupied areas as dynamic obstacles
                    latest_msg = obj.predictions_communication{vehicle_idx}.read_message( ...
                        obj.plant.all_vehicle_ids(veh_with_HP_i), ...
                        obj.k, ...
                        true ...
                    );
                    obj.info.vehs_fallback = union(obj.info.vehs_fallback, latest_msg.vehs_fallback');

                    if ismember(vehicle_idx, obj.info.vehs_fallback)
                        % if the selected vehicle should take fallback
                        is_fallback_triggered = true;
                    else
                        predicted_areas_i = arrayfun(@(array) {[array.x(:)'; array.y(:)']}, latest_msg.predicted_areas);
                        dynamic_obstacle_area(i_vehicle, :) = predicted_areas_i;
                    end

                else
                    % if they are in different groups, read the latest available
                    % message and check it is from the current time step
                    latest_msg = obj.predictions_communication{vehicle_idx}.read_latest_message( ...
                        obj.plant.all_vehicle_ids(veh_with_HP_i) ...
                    );

                    % if the current message is available no less precise
                    % information must be used to consider the vehicle
                    if latest_msg.time_step == obj.k
                        obj.info.vehs_fallback = union(obj.info.vehs_fallback, latest_msg.vehs_fallback');

                        if ismember(vehicle_idx, obj.info.vehs_fallback)
                            is_fallback_triggered = true;
                        else
                            predicted_areas_i = arrayfun(@(array) {[array.x(:)'; array.y(:)']}, latest_msg.predicted_areas);
                            dynamic_obstacle_area(i_vehicle, :) = predicted_areas_i;

                        end

                    else
                        % if they are in different groups and message of
                        % current time step is not available
                        dynamic_obstacle_area(i_vehicle, :) = obj.consider_parallel_coupling(vehicle_idx, veh_with_HP_i);
                    end

                end

            end

            % release preallocated cell array entries
            dynamic_obstacle_area(cellfun(@isempty, dynamic_obstacle_area(:, 1)), :) = [];

        end

        function [obstacles, dynamic_obstacle_area] = consider_vehs_with_LP(obj, vehicle_idx, all_coupling_vehs_without_ROW)
            % CONSIDER_VEHS_WITH_LP Strategies to let vehicle with the right-of-way
            % consider vehicle without the right-of-way
            % '1': do not consider
            % '2': consider currently occupied area as static obstacle
            % '3': consider the occupied area of emergency braking maneuver as static obstacle
            % '4': consider one-step reachable sets as static obstacle
            % '5': consider old trajectory as dynamic obstacle
            %
            % Output:
            %   obstacles (:, 1) cell of areas [x; y]
            %   dynamic_obstacle_area (:, Hp) cell of areas [x; y]

            arguments
                obj (1, 1) PrioritizedController
                vehicle_idx (1, 1) double % index of the current vehicle
                all_coupling_vehs_without_ROW double % indices of the vehicles with lower priority
            end

            % preallocate cell array entries
            obstacles = cell(length(all_coupling_vehs_without_ROW), 1);
            dynamic_obstacle_area = cell(length(all_coupling_vehs_without_ROW), obj.scenario.options.Hp);

            for i_LP = 1:length(all_coupling_vehs_without_ROW)
                veh_without_ROW = all_coupling_vehs_without_ROW(i_LP);

                % strategies to let vehicle with the right-of-way consider vehicle without the right-of-way
                switch obj.scenario.options.strategy_consider_veh_without_ROW
                    case '1'
                        % do not consider

                    case '2'
                        % consider currently occupied area as static obstacle
                        obstacles{i_LP} = obj.iter.occupied_areas{veh_without_ROW}.normal_offset;

                    case '3'
                        % consider the occupied area of emergency braking maneuver
                        % as static obstacle (only if their couplings are not
                        % ignored by forbidding one vehicle entering their lanelet
                        % crossing area, and they have side-impact collision
                        % possibility). Cases that vehicles drive successively are not
                        % included to avoid that vehicles behind push vehicles in
                        % front to move forward.

                        if ( ...
                                obj.scenario.options.scenario_type ~= ScenarioType.circle && ...
                                obj.scenario.options.priority == PriorityStrategies.STAC_priority && ...
                                obj.iter.directed_coupling_reduced(vehicle_idx, veh_without_ROW) == 1 && ...
                                obj.iter.coupling_info{vehicle_idx, veh_without_ROW}.collision_type == CollisionType.from_side && ...
                                obj.iter.coupling_info{vehicle_idx, veh_without_ROW}.lanelet_relationship == LaneletRelationshipType.crossing ...
                            )
                            % the emergency braking maneuver is only considered if
                            % two coupled vehicles at crossing-adjacent lanelets have side-impact collision that is not ignored
                            obstacles{i_LP} = obj.iter.emergency_maneuvers{veh_without_ROW}.braking_area;
                            continue
                        end

                        obstacles{i_LP} = obj.iter.occupied_areas{veh_without_ROW}.normal_offset;

                    case '4'
                        % consider one-step reachable sets as static obstacle
                        reachable_sets = obj.iter.reachable_sets{veh_without_ROW, 1};
                        % get boundary of the polygon
                        [x_reachable_sets, y_reachable_sets] = boundary(reachable_sets);
                        obstacles{i_LP} = [x_reachable_sets'; y_reachable_sets'];

                    case '5'
                        % consider old trajectory as dynamic obstacle
                        latest_msg = obj.predictions_communication{vehicle_idx}.read_latest_message( ...
                            obj.plant.all_vehicle_ids(veh_without_ROW) ...
                        );

                        if latest_msg.time_step <= 0
                            continue
                        end

                        % the message does not come from the initial time step
                        predicted_areas = arrayfun(@(array) {[array.x'; array.y']}, latest_msg.predicted_areas);
                        shift_step = obj.k - latest_msg.time_step; % times that the prediction should be shifted and the last prediction should be repeated

                        if shift_step > 1
                            disp(['shift step is ' num2str(shift_step) ', ego vehicle: ' num2str(vehicle_idx) ', considered vehicle: ' num2str(veh_without_ROW)])
                        end

                        predicted_areas = del_first_rpt_last(predicted_areas(:)', shift_step);
                        dynamic_obstacle_area(i_LP, :) = predicted_areas;

                    otherwise
                        warning("Please specify one of the following strategies to let vehicle with a higher priority also consider vehicle with a lower priority: '1', '2', '3', '4', '5'.")
                end

            end

            % release preallocated cell array entries
            obstacles(cellfun(@isempty, obstacles), :) = [];
            dynamic_obstacle_area(cellfun(@isempty, dynamic_obstacle_area(:, 1)), :) = [];

        end

        function check_others_fallback(obj)
            % the function checks the messages from other controllers
            % whether they take a fallback or not

            for vehicle_index_hlc = obj.plant.indices_in_vehicle_list
                % own vehicle and vehicles that are already remembered to take fallback
                irrelevant_vehicles = union(vehicle_index_hlc, obj.info.vehs_fallback);

                if obj.scenario.options.fallback_type == FallbackType.local_fallback
                    belonging_vector_total = conncomp(digraph(obj.iter.directed_coupling), 'Type', 'weak');
                    sub_graph_fallback = belonging_vector_total(vehicle_index_hlc);
                    % vehicles in the subgraph to check for fallback
                    other_vehicles = find(belonging_vector_total == sub_graph_fallback);
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

        function plan_for_fallback(obj)
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
                        obj.k, ...
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
