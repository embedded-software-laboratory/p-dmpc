classdef PrioritizedController < HighLevelController

    properties (Access = public)
        traffic_communication TrafficCommunication % (1, 1)
        predictions_communication PredictionsCommunication % (1, 1)
        solution_cost_communication SolutionCostCommunication % (1, 1)
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

            % send initial traffic and predictions message
            obj.send_initial_messages();
        end

        function create_ros2_objects(obj)

            i_vehicle = obj.plant.vehicle_indices_controlled;

            % create instance of the communication class
            obj.traffic_communication = TrafficCommunication( ...
                i_vehicle, ...
                obj.ros2_node, ...
                '/vehicle_traffic', ...
                'veh_msgs/Traffic' ...
            );
            obj.predictions_communication = PredictionsCommunication( ...
                i_vehicle, ...
                obj.ros2_node, ...
                '/vehicle_prediction', ...
                'veh_msgs/Predictions' ...
            );
            obj.solution_cost_communication = SolutionCostCommunication( ...
                i_vehicle, ...
                obj.ros2_node, ...
                '/vehicle_solution_cost', ...
                'veh_msgs/SolutionCost' ...
            );

        end

        function send_initial_messages(obj)
            % measure vehicles' initial poses and trims
            [cav_measurements] = obj.plant.measure();

            i_vehicle = obj.plant.vehicle_indices_controlled;
            % store state and trim in iteration data
            obj.iter.x0(i_vehicle, :) = [ ...
                                             cav_measurements(i_vehicle).x, ...
                                             cav_measurements(i_vehicle).y, ...
                                             cav_measurements(i_vehicle).yaw, ...
                                             cav_measurements(i_vehicle).speed ...
                                         ];

            % get own trim
            obj.iter.trim_indices(i_vehicle) = obj.mpa.trim_from_values( ...
                cav_measurements(i_vehicle).speed, ...
                cav_measurements(i_vehicle).steering ...
            );

            if obj.options.scenario_type == ScenarioType.circle
                % In circle scenarios there are no lanelets
                predicted_lanelets = [];
                current_lanelet = 0;
            else
                % Compute the reference trajectory
                [reference_trajectory_struct, ~, current_point_index] = get_reference_trajectory( ...
                    obj.mpa, ...
                    obj.scenario_adapter.scenario.vehicles(i_vehicle).reference_path, ...
                    obj.scenario_adapter.scenario.vehicles(i_vehicle).reference_speed, ...
                    cav_measurements(i_vehicle).x, ...
                    cav_measurements(i_vehicle).y, ...
                    obj.iter.trim_indices(i_vehicle), ...
                    obj.options.dt_seconds ...
                );

                % Compute the predicted lanelets of iVeh vehicle
                [predicted_lanelets, current_lanelet] = get_predicted_lanelets( ...
                    obj.scenario_adapter.scenario.vehicles(i_vehicle).reference_path, ...
                    obj.scenario_adapter.scenario.vehicles(i_vehicle).points_index, ...
                    obj.scenario_adapter.scenario.vehicles(i_vehicle).lanelets_index, ...
                    reference_trajectory_struct, ...
                    current_point_index ...
                );
            end

            % get vehicles currently occupied areas
            occupied_areas = get_occupied_areas( ...
                cav_measurements(i_vehicle).x, ...
                cav_measurements(i_vehicle).y, ...
                cav_measurements(i_vehicle).yaw, ...
                obj.scenario_adapter.scenario.vehicles(i_vehicle).Length, ...
                obj.scenario_adapter.scenario.vehicles(i_vehicle).Width, ...
                obj.options.offset ...
            );

            % for initial time step, reachable_sets and predicted areas do not exist yet
            reachable_sets = {};
            predicted_occupied_areas = {};

            % send messages
            obj.traffic_communication.send_message( ...
                obj.k, ...
                obj.iter.x0(i_vehicle, :), ...
                obj.iter.trim_indices(i_vehicle), ...
                current_lanelet, ...
                predicted_lanelets, ...
                squeeze(obj.iter.reference_trajectory_points(i_vehicle, :, :)), ...
                occupied_areas, ...
                reachable_sets ...
            );
            obj.predictions_communication.send_message( ...
                obj.k, ...
                predicted_occupied_areas ...
            );
        end

        function synchronize_start_with_other_controllers(obj)

            % for synchronization read from all other controllers
            % to ensure that they are ready
            % loop over vehicles that read messages
            other_vehicle_indices = setdiff( ...
                1:obj.options.amount, ...
                obj.plant.vehicle_indices_controlled ...
            );

            for j_vehicle = other_vehicle_indices
                % loop over controllers that are subscribed
                obj.traffic_communication.read_message( ...
                    j_vehicle, ...
                    obj.k, ...
                    throw_error = true, ...
                    timeout_seconds = 40.0 ...
                );
                obj.predictions_communication.read_message( ...
                    j_vehicle, ...
                    obj.k, ...
                    throw_error = true, ...
                    timeout_seconds = 40.0 ...
                );
            end

        end

        function update_controlled_vehicles_traffic_info(obj, cav_measurements)
            % compute vehicles traffic info in HighLevelController
            update_controlled_vehicles_traffic_info@HighLevelController(obj, cav_measurements);

            i_vehicle = obj.plant.vehicle_indices_controlled;
            % Send data to sync obj.iter for all vehicles
            obj.traffic_communication.send_message( ...
                obj.k, ...
                obj.iter.x0(i_vehicle, :), ...
                obj.iter.trim_indices(i_vehicle), ...
                obj.iter.current_lanelet(i_vehicle), ...
                obj.iter.predicted_lanelets{i_vehicle}, ...
                squeeze(obj.iter.reference_trajectory_points(i_vehicle, :, :)), ...
                obj.iter.occupied_areas{i_vehicle}, ...
                obj.iter.reachable_sets(i_vehicle, :) ...
            );

        end

        function update_other_vehicles_traffic_info(obj)
            % read the traffic messages from the other vehicles and
            % store the information in the IterationData object

            % create index struct only once for efficiency
            state_index = indices();

            % read messages from other vehicles
            other_vehicle_indices = setdiff( ...
                1:obj.options.amount, ...
                obj.plant.vehicle_indices_controlled ...
            );

            % loop over vehicle from which the messages are read
            for j_vehicle = other_vehicle_indices
                latest_msg_i = obj.traffic_communication.read_message( ...
                    j_vehicle, ...
                    obj.k, ...
                    throw_error = true ...
                );

                % take state and trim of vehicle j_vehicle
                obj.iter.x0(j_vehicle, :) = [latest_msg_i.current_pose.x, latest_msg_i.current_pose.y, latest_msg_i.current_pose.heading, latest_msg_i.current_pose.speed];
                obj.iter.trim_indices(j_vehicle) = latest_msg_i.current_trim_index;
                obj.iter.current_lanelet(j_vehicle) = latest_msg_i.current_lanelet;

                % transform occupied areas
                occupied_areas = latest_msg_i.occupied_areas;
                obj.iter.occupied_areas{j_vehicle}.normal_offset(1, :) = occupied_areas(1).x;
                obj.iter.occupied_areas{j_vehicle}.normal_offset(2, :) = occupied_areas(1).y;
                obj.iter.occupied_areas{j_vehicle}.without_offset(1, :) = occupied_areas(2).x;
                obj.iter.occupied_areas{j_vehicle}.without_offset(2, :) = occupied_areas(2).y;

                % transform reachable sets to polyshape object
                obj.iter.reachable_sets(j_vehicle, :) = (arrayfun( ...
                    @(array) {polyshape(array.x, array.y, Simplify = false)}, ...
                    latest_msg_i.reachable_sets ...
                ))';

                % transform predicted lanelets
                obj.iter.predicted_lanelets{j_vehicle} = latest_msg_i.predicted_lanelets';

                obj.iter.reference_trajectory_points(j_vehicle, :, 1) = [latest_msg_i.reference_trajectory_points.x];
                obj.iter.reference_trajectory_points(j_vehicle, :, 2) = [latest_msg_i.reference_trajectory_points.y];

                % calculate the predicted lanelet boundary of vehicle j_vehicle based on its predicted lanelets
                if obj.options.scenario_type ~= ScenarioType.circle
                    obj.iter.predicted_lanelet_boundary(j_vehicle, :) = get_lanelets_boundary( ...
                        obj.iter.predicted_lanelets{j_vehicle}, ...
                        obj.scenario_adapter.scenario.lanelet_boundary, ...
                        obj.scenario_adapter.scenario.vehicles(j_vehicle).lanelets_index, ...
                        obj.scenario_adapter.scenario.vehicles(j_vehicle).is_loop ...
                    );
                end

                % get occupied areas of emergency maneuvers for vehicle j_vehicle
                obj.iter.emergency_maneuvers{j_vehicle} = obj.mpa.emergency_maneuvers_at_pose( ...
                    obj.iter.x0(j_vehicle, state_index.x), ...
                    obj.iter.x0(j_vehicle, state_index.y), ...
                    obj.iter.x0(j_vehicle, state_index.heading), ...
                    obj.iter.trim_indices(j_vehicle) ...
                );
            end

        end

        function create_coupling_graph(obj)

            obj.timing.start('receive_from_others', obj.k);
            obj.update_other_vehicles_traffic_info();
            obj.timing.stop('receive_from_others', obj.k);

            obj.timing.start('couple', obj.k);
            obj.couple();
            obj.timing.stop('couple', obj.k);

            obj.timing.start('prioritize', obj.k);
            obj.prioritize();
            obj.timing.stop('prioritize', obj.k);

            obj.timing.start('group', obj.k);
            obj.group();
            obj.timing.stop('group', obj.k);

        end

        function controller(obj)
            % CONTROLLER Plan trajectory for one time step using a
            % prioritized controller. Vehicles inside one group plan in sequence and
            % between groups plan in parallel.

            % initialize variable to store control results
            obj.info = ControlResultsInfo( ...
                1, ...
                obj.options.Hp ...
            );

            obj.timing.start('plan', obj.k);
            obj.plan();
            obj.timing.stop('plan', obj.k);

            obj.timing.start('publish_predictions', obj.k);
            obj.publish_predictions();
            obj.timing.stop('publish_predictions', obj.k);
        end

        function plan(obj)

            % only keep self
            vehicle_index = obj.plant.vehicle_indices_controlled;
            filter_self = false(1, obj.options.amount);
            filter_self(vehicle_index) = true;
            iter_v = IterationData.filter(obj.iter, filter_self);

            % coupled vehicles with higher priorities
            predecessors = find(iter_v.directed_coupling(:, vehicle_index) == 1)';
            % coupled vehicles with higher priorities that vehicle_index computes in sequence with
            predecessors_sequential = find(iter_v.directed_coupling_sequential(:, vehicle_index))';
            % coupled vehicles with lower priorities
            successors = find(iter_v.directed_coupling(vehicle_index, :) == 1);

            % consider vehicles with higher priority
            dynamic_obstacle_area_predecessors = consider_predecessors( ...
                obj, ...
                predecessors, ...
                predecessors_sequential ...
            );

            % consider coupled vehicles with lower priorities
            [obstacles_successors, dynamic_obstacle_area_successors] = obj.consider_successors(successors);

            % add obstacles for considering other vehicles
            iter_v.obstacles = [iter_v.obstacles; obstacles_successors];
            iter_v.dynamic_obstacle_area = [iter_v.dynamic_obstacle_area; dynamic_obstacle_area_predecessors; dynamic_obstacle_area_successors];

            %% Plan for vehicle i_vehicle
            % execute sub controller for 1-veh scenario
            obj.timing.start('optimize', obj.k);
            obj.info = obj.optimizer.run_optimizer( ...
                vehicle_index, ...
                iter_v, ...
                obj.mpa, ...
                obj.options ...
            );
            obj.timing.stop('optimize', obj.k);

            if obj.info.is_exhausted
                % this function checks if a fallback is required
                obj.info = obj.handle_graph_search_exhaustion(obj.info, iter_v);

                if obj.info.needs_fallback
                    obj.plan_fallback();
                end

            end

        end

        function publish_predictions(obj)
            predicted_areas = obj.info.shapes(1, :);
            % send message
            obj.predictions_communication.send_message( ...
                obj.k, ...
                predicted_areas, ...
                obj.info.needs_fallback, ...
                obj.iter.priority_permutation ...
            );
        end

        function couple(obj)
            obj.iter.adjacency = obj.coupler.couple(obj.options, obj.mpa.get_max_speed_of_mpa(), obj.iter);
            obj.iter.coupling_info = obj.coupler.calculate_coupling_info(obj.options, obj.mpa, obj.scenario_adapter.scenario, obj.iter, obj.k);
        end

        function prioritize(obj)
            obj.iter.directed_coupling = obj.prioritizer.prioritize(obj.iter, obj.k, obj.options, obj.scenario_adapter.scenario.intersection_center);
        end

        function group(obj)
            % weigh
            obj.timing.start('weigh', obj.k);
            obj.iter.weighted_coupling = obj.weigher.weigh(obj.iter, obj.k, obj.options, obj.mpa.get_max_speed_of_mpa());
            obj.timing.stop('weigh', obj.k);

            obj.timing.start('cut', obj.k);
            % reduce by grouping and cutting edges
            obj.iter.directed_coupling_sequential = obj.cutter.cut( ...
                obj.iter.weighted_coupling, ...
                obj.options.max_num_CLs ...
            );
            obj.timing.stop('cut', obj.k);

        end

        function dynamic_obstacle_area = parallel_coupling_reachability(obj, j_predecessor)
            % collisions with coupled vehicles with higher priorities in
            % different groups will be avoided by considering
            % their reachable sets as dynamic obstacles
            %
            % Output:
            %   dynamic_obstacle_area (1, Hp) cell of areas [x; y]

            arguments
                obj (1, 1) PrioritizedController
                j_predecessor (1, 1) double % index of the vehicle with higher priority
            end

            % Add their reachable sets as dynamic obstacles to deal with the prediction inconsistency
            reachable_sets_i = obj.iter.reachable_sets(j_predecessor, :);
            % turn polyshape to plain array (repeat the first row to enclosed the shape)
            reachable_sets_i_cell_array = cellfun(@(c) {[c.Vertices(:, 1)', c.Vertices(1, 1)'; c.Vertices(:, 2)', c.Vertices(1, 2)']}, reachable_sets_i);
            dynamic_obstacle_area = reachable_sets_i_cell_array;

        end

        function dynamic_obstacle_area = parallel_coupling_previous_trajectory(obj, j_predecessor)
            % collisions with coupled vehicles with higher priorities in
            % different groups will be avoided by considering
            % their one-step delayed predicted trajectories as dynamic obstacle
            %
            % Output:
            %   dynamic_obstacle_area (1, Hp) cell of areas [x; y]

            arguments
                obj (1, 1) PrioritizedController
                j_predecessor (1, 1) double % index of the vehicle with higher priority
            end

            % initialize the returned variable with dimension 0 that it does
            % not extend the list of dynamic_obstacle_area
            dynamic_obstacle_area = cell(0, obj.options.Hp);

            if obj.k <= 1
                return
            end

            % the old trajectories are available from the second time step onwards
            old_msg = obj.predictions_communication.read_message( ...
                j_predecessor, ...
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

        function dynamic_obstacle_area = consider_predecessors( ...
                obj, ...
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
                % indices of coupled vehicles with higher priority
                predecessors (1, :) double
                % indices of sequentially computing vehicles with higher priority
                predecessors_sequential (1, :) double
            end

            % preallocate cell array entries
            dynamic_obstacle_area = cell(length(predecessors), obj.options.Hp);

            predecessors_parallel = setdiff(predecessors, predecessors_sequential);

            for j_vehicle = predecessors_sequential
                % if in the same group, read the current message and set the
                % predicted occupied areas as dynamic obstacles
                latest_msg = obj.predictions_communication.read_message( ...
                    j_vehicle, ...
                    obj.k, ...
                    priority_permutation = obj.iter.priority_permutation, ...
                    throw_error = true ...
                );

                obj.iter.fallbacks(latest_msg.vehicle_index) = latest_msg.needs_fallback;

                predicted_areas = arrayfun(@(array) {[array.x(:)'; array.y(:)']}, latest_msg.predicted_areas);
                j_predecessor = predecessors == j_vehicle;
                dynamic_obstacle_area(j_predecessor, :) = predicted_areas;
            end

            for j_vehicle = predecessors_parallel

                % if they are in different groups, use the message
                % from the previous time step
                j_predecessor = predecessors == j_vehicle;
                dynamic_obstacle_area(j_predecessor, :) = ...
                    obj.consider_parallel_coupling(j_vehicle);

            end

            % release preallocated cell array entries
            dynamic_obstacle_area(cellfun(@isempty, dynamic_obstacle_area(:, 1)), :) = [];

        end

        function [obstacles, dynamic_obstacle_area] = consider_successors(obj, successors)
            % consider_successors Consider coupled vehicles with lower priority
            %
            % Output:
            %   obstacles (:, 1) cell of areas [x; y]
            %   dynamic_obstacle_area (:, Hp) cell of areas [x; y]

            arguments
                obj (1, 1) PrioritizedController
                successors double % indices of the vehicles with lower priority
            end

            % preallocate cell array entries
            obstacles = cell(length(successors), 1);
            dynamic_obstacle_area = cell(length(successors), obj.options.Hp);
            state_indices = indices();

            for j_successor = 1:length(successors)
                successor_vehicle = successors(j_successor);

                % strategies to let vehicle with the right-of-way consider vehicle without the right-of-way
                switch obj.options.constraint_from_successor
                    case ConstraintFromSuccessor.none
                        % do not consider

                    case ConstraintFromSuccessor.area_of_standstill
                        % consider currently occupied area as static obstacle,
                        % if the vehicle is at standstill
                        standstill_speed_meter_per_second = 0.01;

                        if abs(obj.iter.x0(successor_vehicle, state_indices.speed)) < standstill_speed_meter_per_second
                            obstacles{j_successor} = obj.iter.occupied_areas{successor_vehicle}.normal_offset;
                        end

                    case ConstraintFromSuccessor.area_of_previous_trajectory
                        % consider old trajectory as dynamic obstacle
                        latest_msg = obj.predictions_communication.read_latest_message( ...
                            successor_vehicle ...
                        );

                        if latest_msg.time_step <= 0
                            continue
                        end

                        % the message does not come from the initial time step
                        predicted_areas = arrayfun(@(array) {[array.x'; array.y']}, latest_msg.predicted_areas);
                        shift_step = obj.k - latest_msg.time_step; % times that the prediction should be shifted and the last prediction should be repeated

                        predicted_areas = del_first_rpt_last(predicted_areas(:)', shift_step);
                        dynamic_obstacle_area(j_successor, :) = predicted_areas;
                end

            end

            % release preallocated cell array entries
            obstacles(cellfun(@isempty, obstacles), :) = [];
            dynamic_obstacle_area(cellfun(@isempty, dynamic_obstacle_area(:, 1)), :) = [];

        end

        function info_v = handle_graph_search_exhaustion(obj, info_v, iter)
            trim = iter.trim_indices;

            if obj.mpa.trims(trim).speed == 0 ...
                    && ~(obj.options.constraint_from_successor == ConstraintFromSuccessor.none)
                % If a vehicle at a standstill cannot find a feasible
                % trajectory, but higher-priority vehicles avoid at least its
                % standstill position, it will stay there without
                % triggering a fallback. This behavior can happen when the
                % reachable set of a higher-priority vehicle cannot be avoided.
                Hp = obj.options.Hp;
                x = iter.x0(:, 1);
                y = iter.x0(:, 2);
                yaw = iter.x0(:, 3);
                info_v.tree_path = ones(size(x, 1), Hp + 1);
                info_v.y_predicted = repmat( ...
                    [x; y; yaw], ...
                    1, ...
                    Hp ...
                );

                vehiclePolygon = transformed_rectangle( ...
                    x, ...
                    y, ...
                    yaw, ...
                    obj.scenario_adapter.scenario.vehicles(1).Length, ...
                    obj.scenario_adapter.scenario.vehicles(1).Width ...
                );
                shape_veh = {[vehiclePolygon, vehiclePolygon(:, 1)]}; % close shape

                info_v.shapes = repmat(shape_veh, 1, Hp);
                % Predicted trims in the future Hp time steps. The first entry is the current trims
                info_v.predicted_trims = repmat(trim, 1, Hp);
                info_v.needs_fallback = false;
            else
                info_v.needs_fallback = true;
            end

        end

        function check_others_fallback(obj)
            % Determine if there is a fallback which this vehicles has not considered.
            % If so, it needs to take the fallback solution.
            i_vehicle = obj.plant.vehicle_indices_controlled;

            belonging_vector_total = conncomp(digraph(obj.iter.directed_coupling), 'Type', 'weak');
            sub_graph_fallback = belonging_vector_total(i_vehicle);
            % vehicles in the subgraph to check for fallback
            other_vehicle_indices = find(belonging_vector_total == sub_graph_fallback);
            % remove irrelevant vehicles which have not to be checked for fallback
            % irrelevant are those from which we have read the message already
            predecessors_sequential = find(obj.iter.directed_coupling_sequential(:, i_vehicle))';
            irrelevant_vehicles = [i_vehicle, predecessors_sequential];
            other_vehicle_indices = setdiff(other_vehicle_indices, irrelevant_vehicles, 'stable');

            for j_vehicle = other_vehicle_indices
                latest_msg = obj.predictions_communication.read_message( ...
                    j_vehicle, ...
                    obj.k, ...
                    priority_permutation = obj.iter.priority_permutation, ...
                    throw_error = true ...
                );

                obj.iter.fallbacks(latest_msg.vehicle_index) = latest_msg.needs_fallback;

            end

            fallback_matrix = obj.iter.adjacency;
            % remove outgoing edges of fallback vehicles;
            % These fallback edges have already been considered during planning
            outgoing_fallback = obj.iter.directed_coupling_sequential;
            outgoing_fallback(~obj.iter.fallbacks, :) = 0;
            outgoing_fallback = outgoing_fallback + outgoing_fallback';
            fallback_matrix = fallback_matrix - outgoing_fallback;

            fallback_graph = digraph(fallback_matrix);
            fallback_vehicle_indices = find(obj.iter.fallbacks);

            for i = 1:numel(fallback_vehicle_indices)
                i_fallback_vehicle = fallback_vehicle_indices(i);
                graph_path = shortestpath( ...
                    fallback_graph, ...
                    i_fallback_vehicle, ...
                    i_vehicle ...
                );

                if ~isempty(graph_path)
                    obj.info.needs_fallback = true;
                    break;
                end

            end

        end

        function plan_fallback(obj, optional)

            arguments
                obj (1, 1) PrioritizedController
                optional.is_fallback_while_planning (1, 1) logical = true;
            end

            % initialize
            obj.info = ControlResultsInfo(1, obj.options.Hp);

            obj.info.tree = obj.info_old.tree;
            obj.info.tree_path = del_first_rpt_last(obj.info_old.tree_path);
            obj.info.shapes = del_first_rpt_last(obj.info_old.shapes);
            obj.info.predicted_trims = del_first_rpt_last(obj.info_old.predicted_trims);
            obj.info.y_predicted = del_first_rpt_last(obj.info_old.y_predicted);
            obj.info.needs_fallback = optional.is_fallback_while_planning;
        end

        function controller_fallback(obj, optional)

            arguments
                obj (1, 1) PrioritizedController
                optional.is_fallback_while_planning (1, 1) logical = true;
            end

            % planning by using last priority and trajectories directly

            obj.plan_fallback(is_fallback_while_planning = optional.is_fallback_while_planning);
            obj.publish_predictions();

        end

        function clean_up(obj)
            % delete ros2 objects
            delete(obj.ros2_node);
            % clean up hlc in reverse order than constructing
            clean_up@HighLevelController(obj);
        end

    end

end
