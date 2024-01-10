classdef (Abstract) PrioritizedController < HighLevelController

    properties (Access = public)
        % instances of communication classes stored in a cell array
        % contains one instance for each vehicle that is controlled by the hlc
        % parallel/distributed execution: same size but only on entry that is not empty
        traffic_communication (1, :) cell
        predictions_communication (1, :) cell
        solution_cost_communication (1, :) cell
    end

    properties (Access = protected)
        ros2_node ros2node; % (1, 1)

        prioritizer
        weigher
        cutter

        consider_parallel_coupling (1, 1) function_handle = @()[];
    end

    methods

        function obj = PrioritizedController(options, plant, ros2_node)
            obj = obj@HighLevelController(options, plant);

            obj.ros2_node = ros2_node;

            obj.prioritizer = Prioritizer.get_prioritizer(obj.options.priority);
            obj.weigher = Weigher.get_weigher(obj.options.weight);
            obj.cutter = Cutter.get_cutter(options.cut);

            if obj.options.isDealPredictionInconsistency
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

            % construct optimizer
            obj.optimizer = OptimizerInterface.get_optimizer(obj.options, obj.mpa, obj.scenario_adapter.scenario, obj.plant.vehicle_indices_controlled);

            % in prioritized computation, vehicles communicate via ROS 2
            timer = tic;
            fprintf('Creating ROS2 objects... ');
            obj.create_ros2_objects();
            fprintf('done (%.2f s).\n', toc(timer));

            % initialize the communication network of ROS 2
            obj.init_communication();
        end

        function create_ros2_objects(obj)

            for vehicle_index = obj.plant.vehicle_indices_controlled

                % create instance of the communication class
                obj.traffic_communication{vehicle_index} = TrafficCommunication( ...
                    vehicle_index, ...
                    obj.ros2_node, ...
                    '/vehicle_traffic', ...
                    'veh_msgs/Traffic' ...
                );
                obj.predictions_communication{vehicle_index} = PredictionsCommunication( ...
                    vehicle_index, ...
                    obj.ros2_node, ...
                    '/vehicle_prediction', ...
                    'veh_msgs/Predictions' ...
                );
                obj.solution_cost_communication{vehicle_index} = SolutionCostCommunication( ...
                    vehicle_index, ...
                    obj.ros2_node, ...
                    '/vehicle_solution_cost', ...
                    'veh_msgs/SolutionCost' ...
                );
            end

            if length(obj.plant.vehicle_indices_controlled) == 1
                % wait for all subscribers to be created in distributed case,
                % because otherwise early sent messages will be lost.
                pause(5.0);
            end

        end

        function init_communication(obj)
            % communicate initial traffic and predictions message
            % synchronize the prioritized controllers in parallel/distributed execution

            % measure vehicles' initial poses and trims
            [cav_measurements] = obj.plant.measure();

            for vehicle_index = obj.plant.vehicle_indices_controlled
                % store state and trim in iteration data
                obj.iter.x0(vehicle_index, :) = [ ...
                                                     cav_measurements(vehicle_index).x, ...
                                                     cav_measurements(vehicle_index).y, ...
                                                     cav_measurements(vehicle_index).yaw, ...
                                                     cav_measurements(vehicle_index).speed ...
                                                 ];

                % get own trim
                obj.iter.trim_indices(vehicle_index) = obj.mpa.trim_from_values( ...
                    cav_measurements(vehicle_index).speed, ...
                    cav_measurements(vehicle_index).steering ...
                );

                if obj.options.scenario_type == ScenarioType.circle
                    % In circle scenarios there are no lanelets
                    predicted_lanelets = [];
                    current_lanelet = 0;
                else
                    % Compute the reference trajectory
                    [reference_trajectory_struct, ~, current_point_index] = get_reference_trajectory( ...
                        obj.mpa, ...
                        obj.scenario_adapter.scenario.vehicles(vehicle_index).reference_path, ...
                        cav_measurements(vehicle_index).x, ...
                        cav_measurements(vehicle_index).y, ...
                        obj.iter.trim_indices(vehicle_index), ...
                        obj.options.dt_seconds ...
                    );

                    % Compute the predicted lanelets of iVeh vehicle
                    [predicted_lanelets, current_lanelet] = get_predicted_lanelets( ...
                        obj.scenario_adapter.scenario.vehicles(vehicle_index).reference_path, ...
                        obj.scenario_adapter.scenario.vehicles(vehicle_index).points_index, ...
                        obj.scenario_adapter.scenario.vehicles(vehicle_index).lanelets_index, ...
                        reference_trajectory_struct, ...
                        current_point_index ...
                    );
                end

                % get vehicles currently occupied areas
                occupied_areas = get_occupied_areas( ...
                    cav_measurements(vehicle_index).x, ...
                    cav_measurements(vehicle_index).y, ...
                    cav_measurements(vehicle_index).yaw, ...
                    obj.scenario_adapter.scenario.vehicles(vehicle_index).Length, ...
                    obj.scenario_adapter.scenario.vehicles(vehicle_index).Width, ...
                    obj.options.offset ...
                );

                % for initial time step, reachable_sets and predicted areas do not exist yet
                reachable_sets = {};
                predicted_occupied_areas = {};

                % send messages
                obj.traffic_communication{vehicle_index}.send_message( ...
                    obj.k, ...
                    obj.iter.x0(vehicle_index, :), ...
                    obj.iter.trim_indices(vehicle_index), ...
                    current_lanelet, ...
                    predicted_lanelets, ...
                    squeeze(obj.iter.reference_trajectory_points(vehicle_index, :, :)), ...
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
            for vehicle_index = obj.plant.vehicle_indices_controlled
                % loop over vehicles that read messages
                other_vehicles = setdiff(1:obj.options.amount, vehicle_index);

                for vehicle_index_subscribed = other_vehicles
                    % loop over controllers that are subscribed
                    obj.traffic_communication{vehicle_index}.read_message( ...
                        vehicle_index_subscribed, ...
                        obj.k, ...
                        throw_error = true, ...
                        timeout = 40.0 ...
                    );
                    obj.predictions_communication{vehicle_index}.read_message( ...
                        vehicle_index_subscribed, ...
                        obj.k, ...
                        throw_error = true, ...
                        timeout = 40.0 ...
                    );
                end

            end

        end

        function update_controlled_vehicles_traffic_info(obj, cav_measurements)
            % compute vehicles traffic info in HighLevelController
            update_controlled_vehicles_traffic_info@HighLevelController(obj, cav_measurements);

            for iVeh = obj.plant.vehicle_indices_controlled
                % Send data to sync obj.iter for all vehicles
                obj.traffic_communication{iVeh}.send_message( ...
                    obj.k, ...
                    obj.iter.x0(iVeh, :), ...
                    obj.iter.trim_indices(iVeh), ...
                    obj.iter.current_lanelet(iVeh), ...
                    obj.iter.predicted_lanelets{iVeh}, ...
                    squeeze(obj.iter.reference_trajectory_points(iVeh, :, :)), ...
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
            jVeh = obj.plant.vehicle_indices_controlled(1);

            % read messages from other vehicles
            other_vehicle_indices = setdiff(1:obj.options.amount, obj.plant.vehicle_indices_controlled);

            % loop over vehicle from which the messages are read
            for kVeh = other_vehicle_indices
                latest_msg_i = obj.traffic_communication{jVeh}.read_message( ...
                    kVeh, ...
                    obj.k, ...
                    throw_error = true ...
                );

                % take state and trim of vehicle kVeh
                obj.iter.x0(kVeh, :) = [latest_msg_i.current_pose.x, latest_msg_i.current_pose.y, latest_msg_i.current_pose.heading, latest_msg_i.current_pose.speed];
                obj.iter.trim_indices(kVeh) = latest_msg_i.current_trim_index;
                obj.iter.current_lanelet(kVeh) = latest_msg_i.current_lanelet;

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

                obj.iter.reference_trajectory_points(kVeh, :, 1) = latest_msg_i.reference_trajectory_points.x;
                obj.iter.reference_trajectory_points(kVeh, :, 2) = latest_msg_i.reference_trajectory_points.y;

                % calculate the predicted lanelet boundary of vehicle kVeh based on its predicted lanelets
                if obj.options.scenario_type ~= ScenarioType.circle
                    obj.iter.predicted_lanelet_boundary(kVeh, :) = get_lanelets_boundary( ...
                        obj.iter.predicted_lanelets{kVeh}, ...
                        obj.scenario_adapter.scenario.lanelet_boundary, ...
                        obj.scenario_adapter.scenario.vehicles(kVeh).lanelets_index, ...
                        obj.scenario_adapter.scenario.vehicles(kVeh).is_loop ...
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
            obj.timing_general.start('create_coupling_graph', obj.k);

            obj.update_other_vehicles_traffic_info();

            obj.timing_general.start('coupling', obj.k);
            obj.couple();
            obj.timing_general.stop("coupling", obj.k);

            obj.timing_general.start("prioritization", obj.k);
            obj.prioritize();
            obj.timing_general.stop("prioritization", obj.k);

            obj.timing_general.start('reduce_computation_levels', obj.k);
            obj.reduce_computation_levels();
            obj.timing_general.stop('reduce_computation_levels', obj.k);
            obj.timing_general.stop('create_coupling_graph', obj.k);

        end

        function plan_single_vehicle(obj, vehicle_idx)

            % only keep self
            filter_self = false(1, obj.options.amount);
            filter_self(vehicle_idx) = true;
            iter_v = filter_iter(obj.iter, filter_self);

            % coupled vehicles with higher priorities
            predecessors = find(iter_v.directed_coupling_reduced(:, vehicle_idx) == 1)';
            % coupled vehicles with higher priorities that vehicl_idx computes in sequence with
            predecessors_sequential = find(iter_v.directed_coupling_sequential(:, vehicle_idx))';
            % coupled vehicles with lower priorities
            successors = find(iter_v.directed_coupling_reduced(vehicle_idx, :) == 1);

            % consider vehicles with higher priority
            [dynamic_obstacle_area_predecessors, is_fallback_triggered] = consider_predecessors( ...
                obj, ...
                vehicle_idx, ...
                predecessors, ...
                predecessors_sequential ...
            );

            % if vehicle with higher priority triggered fallback, do not plan
            if is_fallback_triggered
                return
            end

            % consider coupled vehicles with lower priorities
            [obstacles_successors, dynamic_obstacle_area_successors] = obj.consider_successors(vehicle_idx, successors);

            % add obstacles for considering other vehicles
            iter_v.obstacles = [iter_v.obstacles; obstacles_successors];
            iter_v.dynamic_obstacle_area = [iter_v.dynamic_obstacle_area; dynamic_obstacle_area_predecessors; dynamic_obstacle_area_successors];

            %% Plan for vehicle vehicle_idx
            % execute sub controller for 1-veh scenario
            obj.timing_per_vehicle(vehicle_idx).start('optimizer', obj.k);
            info_v = obj.optimizer.run_optimizer(vehicle_idx, iter_v, obj.mpa, obj.options);
            obj.timing_per_vehicle(vehicle_idx).stop('optimizer', obj.k);

            if info_v.is_exhausted
                info_v = handle_graph_search_exhaustion(info_v, obj.options, iter_v, obj.mpa);
            end

            if info_v.needs_fallback
                % if graph search is exhausted, this vehicles and all its weakly coupled vehicles will use their fallback trajectories
                obj.info.needs_fallback(vehicle_idx) = true;

                switch obj.options.fallback_type
                    case FallbackType.local_fallback
                        % local fallback: only vehicles in same subgraph take fallback
                        belonging_vector_total = conncomp(digraph(obj.iter.directed_coupling), 'Type', 'weak');
                        sub_graph_fallback = belonging_vector_total(vehicle_idx);
                        obj.info.vehicles_fallback = [obj.info.vehicles_fallback; find(belonging_vector_total == sub_graph_fallback).'];
                        obj.info.vehicles_fallback = unique(obj.info.vehicles_fallback, 'stable');
                    case FallbackType.no_fallback
                        % Fallback is disabled. Simulation will end.
                        obj.info.vehicles_fallback = int32(1):int32(obj.options.amount);
                end

            else
                obj.info = store_control_info(obj.info, info_v, obj.options);
            end

        end

        function publish_predictions(obj, vehicle_idx)

            if ~ismember(vehicle_idx, obj.info.vehicles_fallback)
                % if the selected vehicle should take fallback

                predicted_areas_k = obj.info.shapes(vehicle_idx, :);
                % send message
                obj.predictions_communication{vehicle_idx}.send_message( ...
                    obj.k, ...
                    predicted_areas_k, ...
                    obj.info.vehicles_fallback, ...
                    obj.iter.priority_permutation ...
                );

            else
                obj.predictions_communication{vehicle_idx}.send_message( ...
                    obj.k, ...
                    {}, ...
                    obj.info.vehicles_fallback, ...
                    obj.iter.priority_permutation ...
                );
            end

        end

        function couple(obj)
            obj.iter.adjacency = obj.coupler.couple(obj.options, obj.mpa.get_max_speed_of_mpa(), obj.scenario_adapter.scenario.adjacency_lanelets, obj.iter);
            obj.iter.coupling_info = obj.coupler.calculate_coupling_info(obj.options, obj.mpa, obj.scenario_adapter.scenario, obj.iter, obj.k);
        end

        function prioritize(obj)
            obj.iter.directed_coupling = obj.prioritizer.prioritize(obj.iter, obj.k, obj.options, obj.scenario_adapter.scenario.intersection_center);
        end

        function reduce_computation_levels(obj)
            % weigh
            obj.iter.weighted_coupling = obj.weigher.weigh(obj.iter, obj.k, obj.options, obj.mpa.get_max_speed_of_mpa());

            % reduce by replacing with lanelet crossing area obstacles
            obj.iter.directed_coupling_reduced = obj.iter.directed_coupling;
            obj.iter.weighted_coupling_reduced = obj.iter.weighted_coupling;

            if obj.options.scenario_type ~= ScenarioType.circle
                % reduce only if no circle scenario

                % get ignored couplings and lanelet_crossing_areas
                [ ...
                     obj.iter.coupling_info, ...
                     obj.iter.directed_coupling_reduced, ...
                     obj.iter.weighted_coupling_reduced, ...
                     obj.iter.lanelet_crossing_areas ...
                 ] = reduce_coupling_lanelet_crossing_area( ...
                    obj.iter, ...
                    obj.options.strategy_enter_lanelet_crossing_area ...
                );

            end

            obj.timing_general.start("group_vehs_time", obj.k);
            % reduce by grouping and cutting edges
            obj.iter.directed_coupling_sequential = obj.cutter.cut( ...
                obj.iter.weighted_coupling_reduced, ...
                obj.options.max_num_CLs ...
            );
            obj.timing_general.stop("group_vehs_time", obj.k);

        end

        function dynamic_obstacle_area = parallel_coupling_reachability(obj, ~, i_predecessor)
            % collisions with coupled vehicles with higher priorities in
            % different groups will be avoided by considering
            % their reachable sets as dynamic obstacles
            %
            % Output:
            %   dynamic_obstacle_area (1, Hp) cell of areas [x; y]

            arguments
                obj (1, 1) PrioritizedController
                ~% index of the current vehicle
                i_predecessor (1, 1) double % index of the vehicle with higher priority
            end

            % Add their reachable sets as dynamic obstacles to deal with the prediction inconsistency
            reachable_sets_i = obj.iter.reachable_sets(i_predecessor, :);
            % turn polyshape to plain array (repeat the first row to enclosed the shape)
            reachable_sets_i_cell_array = cellfun(@(c) {[c.Vertices(:, 1)', c.Vertices(1, 1)'; c.Vertices(:, 2)', c.Vertices(1, 2)']}, reachable_sets_i);
            dynamic_obstacle_area = reachable_sets_i_cell_array;

        end

        function dynamic_obstacle_area = parallel_coupling_previous_trajectory(obj, vehicle_idx, i_predecessor)
            % collisions with coupled vehicles with higher priorities in
            % different groups will be avoided by considering
            % their one-step delayed predicted trajectories as dynamic obstacle
            %
            % Output:
            %   dynamic_obstacle_area (1, Hp) cell of areas [x; y]

            arguments
                obj (1, 1) PrioritizedController
                vehicle_idx (1, 1) double % index of the current vehicle
                i_predecessor (1, 1) double % index of the vehicle with higher priority
            end

            % initialize the returned variable with dimension 0 that it does
            % not extend the list of dynamic_obstacle_area
            dynamic_obstacle_area = cell(0, obj.options.Hp);

            if obj.k <= 1
                return
            end

            % the old trajectories are available from the second time step onwards
            old_msg = obj.predictions_communication{vehicle_idx}.read_message( ...
                i_predecessor, ...
                obj.k - 1, ...
                priority_permutation = obj.iter.priority_permutation, ...
                throw_error = true ...
            );
            predicted_areas_i = arrayfun(@(array) {[array.x(:)'; array.y(:)']}, old_msg.predicted_areas);
            oldness_msg = obj.k - old_msg.time_step;

            if oldness_msg ~= 0
                % consider the oldness of the message: delete the first n entries and repeat the last entry for n times
                predicted_areas_i = del_first_rpt_last(predicted_areas_i', oldness_msg);
            end

            dynamic_obstacle_area = predicted_areas_i;

        end

        function [dynamic_obstacle_area, is_fallback_triggered] = consider_predecessors( ...
                obj, ...
                vehicle_idx, ...
                predecessors, ...
                predecessors_sequential ...
            )
            % consider_predecessors considering vehicles with higher priority
            % strategy depends on whether the vehicles are in the same group
            % and if prediction consistency should be guaranteed
            %
            % Output:
            %   dynamic_obstacle_area (1, Hp) cell of areas [x; y]
            %   is_fallback_triggered (1, 1) logical
            %       true if higher prioritized vehicle has a fallback

            arguments
                obj (1, 1) PrioritizedController
                vehicle_idx (1, 1) double % index of the current vehicle
                % indices of coupled vehicles with higher priority
                predecessors (1, :) double
                % indices of sequentially computing vehicles with higher priority
                predecessors_sequential (1, :) double
            end

            is_fallback_triggered = false;

            % preallocate cell array entries
            dynamic_obstacle_area = cell(length(predecessors), obj.options.Hp);

            predecessors_parallel = setdiff(predecessors, predecessors_sequential);

            for i_vehicle = predecessors_sequential
                % if in the same group, read the current message and set the
                % predicted occupied areas as dynamic obstacles
                latest_msg = obj.predictions_communication{vehicle_idx}.read_message( ...
                    i_vehicle, ...
                    obj.k, ...
                    priority_permutation = obj.iter.priority_permutation, ...
                    throw_error = true ...
                );
                obj.info.vehicles_fallback = union(obj.info.vehicles_fallback, latest_msg.vehicles_fallback');

                if ismember(vehicle_idx, obj.info.vehicles_fallback)
                    % if the selected vehicle should take fallback
                    is_fallback_triggered = true;
                    break
                else
                    predicted_areas_i = arrayfun(@(array) {[array.x(:)'; array.y(:)']}, latest_msg.predicted_areas);
                    i_predecessor = predecessors == i_vehicle;
                    dynamic_obstacle_area(i_predecessor, :) = predicted_areas_i;
                end

            end

            for i_vehicle = predecessors_parallel

                if is_fallback_triggered
                    break
                end

                % if they are in different groups, use the message
                % from the previous time step for reprodicibility
                i_predecessor = predecessors == i_vehicle;
                dynamic_obstacle_area(i_predecessor, :) = ...
                    obj.consider_parallel_coupling(vehicle_idx, i_vehicle);

            end

            % release preallocated cell array entries
            dynamic_obstacle_area(cellfun(@isempty, dynamic_obstacle_area(:, 1)), :) = [];

        end

        function [obstacles, dynamic_obstacle_area] = consider_successors(obj, vehicle_idx, successors)
            % consider_successors Consider coupled vehicles with lower priority
            %
            % Output:
            %   obstacles (:, 1) cell of areas [x; y]
            %   dynamic_obstacle_area (:, Hp) cell of areas [x; y]

            arguments
                obj (1, 1) PrioritizedController
                vehicle_idx (1, 1) double % index of the current vehicle
                successors double % indices of the vehicles with lower priority
            end

            % preallocate cell array entries
            obstacles = cell(length(successors), 1);
            dynamic_obstacle_area = cell(length(successors), obj.options.Hp);
            state_indices = indices();

            for i_successor = 1:length(successors)
                successor_vehicle = successors(i_successor);

                % strategies to let vehicle with the right-of-way consider vehicle without the right-of-way
                switch obj.options.constraint_from_successor
                    case ConstraintFromSuccessor.none
                        % do not consider

                    case ConstraintFromSuccessor.area_of_standstill
                        % consider currently occupied area as static obstacle,
                        % if the vehicle is at standstill
                        standstill_speed_meter_per_second = 0.01;

                        if abs(obj.iter.x0(successor_vehicle, state_indices.speed)) < standstill_speed_meter_per_second
                            obstacles{i_successor} = obj.iter.occupied_areas{successor_vehicle}.normal_offset;
                        end

                    case ConstraintFromSuccessor.area_of_previous_trajectory
                        % consider old trajectory as dynamic obstacle
                        latest_msg = obj.predictions_communication{vehicle_idx}.read_latest_message( ...
                            successor_vehicle ...
                        );

                        if latest_msg.time_step <= 0
                            continue
                        end

                        % the message does not come from the initial time step
                        predicted_areas = arrayfun(@(array) {[array.x'; array.y']}, latest_msg.predicted_areas);
                        shift_step = obj.k - latest_msg.time_step; % times that the prediction should be shifted and the last prediction should be repeated

                        if shift_step > 1
                            disp(['shift step is ' num2str(shift_step) ', ego vehicle: ' num2str(vehicle_idx) ', considered vehicle: ' num2str(successor_vehicle)])
                        end

                        predicted_areas = del_first_rpt_last(predicted_areas(:)', shift_step);
                        dynamic_obstacle_area(i_successor, :) = predicted_areas;
                end

            end

            % release preallocated cell array entries
            obstacles(cellfun(@isempty, obstacles), :) = [];
            dynamic_obstacle_area(cellfun(@isempty, dynamic_obstacle_area(:, 1)), :) = [];

        end

        function check_others_fallback(obj)
            % the function checks the messages from other controllers
            % whether they take a fallback or not

            for vehicle_index_hlc = obj.plant.vehicle_indices_controlled
                % own vehicle and vehicles that are already remembered to take fallback
                irrelevant_vehicles = union(vehicle_index_hlc, obj.info.vehicles_fallback);

                if obj.options.fallback_type == FallbackType.local_fallback
                    belonging_vector_total = conncomp(digraph(obj.iter.directed_coupling), 'Type', 'weak');
                    sub_graph_fallback = belonging_vector_total(vehicle_index_hlc);
                    % vehicles in the subgraph to check for fallback
                    other_vehicles = find(belonging_vector_total == sub_graph_fallback);
                    % remove irrelevant vehicles which have not to be checked for fallback
                    other_vehicles = setdiff(other_vehicles, irrelevant_vehicles, 'stable');

                    for veh_id = other_vehicles
                        latest_msg = obj.predictions_communication{vehicle_index_hlc}.read_message( ...
                            veh_id, ...
                            obj.k, ...
                            priority_permutation = obj.iter.priority_permutation, ...
                            throw_error = true ...
                        );
                        fallback_info_veh_id = latest_msg.vehicles_fallback';
                        obj.info.vehicles_fallback = union(obj.info.vehicles_fallback, fallback_info_veh_id);
                    end

                else
                    % vehicles in the total graph to check for fallback
                    other_vehicles = 1:obj.options.amount;
                    % remove irrelevant vehicles which have not to be checked for fallback
                    other_vehicles = setdiff(other_vehicles, irrelevant_vehicles);

                    for veh_id = other_vehicles
                        latest_msg = obj.predictions_communication{vehicle_index_hlc}.read_message( ...
                            veh_id, ...
                            obj.k, ...
                            priority_permutation = obj.iter.priority_permutation, ...
                            throw_error = true ...
                        );
                        fallback_info_veh_id = latest_msg.vehicles_fallback';
                        obj.info.vehicles_fallback = union(obj.info.vehicles_fallback, fallback_info_veh_id);
                    end

                end

            end

        end

        function plan_for_fallback(obj)
            % planning by using last priority and trajectories directly

            tick_per_step = obj.options.tick_per_step + 1;

            for vehicle_idx = obj.plant.vehicle_indices_controlled

                if ~ismember(vehicle_idx, obj.info.vehicles_fallback)
                    continue
                end

                % initialize
                info_v = ControlResultsInfo(1, obj.options.Hp, vehicle_idx);

                info_v.tree = obj.info_old.tree{vehicle_idx};
                info_v.tree_path = del_first_rpt_last(obj.info_old.tree_path(vehicle_idx, :));
                info_v.shapes = del_first_rpt_last(obj.info_old.shapes(vehicle_idx, :));
                info_v.predicted_trims = del_first_rpt_last(obj.info_old.predicted_trims(vehicle_idx, :));

                % predicted trajectory of the next time step
                y_pred_v = obj.info_old.y_predicted{vehicle_idx};
                y_pred_v = [y_pred_v(tick_per_step + 1:end, :); y_pred_v(tick_per_step * (obj.options.Hp - 1) + 1:end, :)];
                info_v.y_predicted = {y_pred_v};

                % prepare output data
                obj.info = store_control_info(obj.info, info_v, obj.options);

                % data only need to be updated if isDealPredictionInconsistency
                % is off, because only old reachable sets but no old predicted areas
                % are used by controller
                if obj.options.isDealPredictionInconsistency == false
                    % send message
                    obj.predictions_communication{vehicle_idx}.send_message( ...
                        obj.k, ...
                        obj.info.shapes(vehicle_idx, :), ...
                        obj.info.vehicles_fallback, ...
                        obj.iter.priority_permutation ...
                    );
                end

            end

        end

        function clean_up(obj)
            % delete ros2 objects
            obj.traffic_communication = {};
            obj.predictions_communication = {};
            obj.solution_cost_communication = {};
            delete(obj.ros2_node);
            % clean up hlc in reverse order than constructing
            clean_up@HighLevelController(obj);
        end

    end

end
