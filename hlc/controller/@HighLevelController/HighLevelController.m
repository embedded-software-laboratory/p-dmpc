classdef (Abstract) HighLevelController < handle
    %TODO check for private/protected vars
    properties (Access = public)
        % scenario
        scenario;

        % Adapter for the lab
        % or one for a local simulation
        plant;

        optimizer;
        mpa;
        controller_name;
        result;
        k;
        iter;
        info;

        manual_vehicles;
    end

    properties (Access = protected)
        timing (1, 1) ControllerTiming;

        % old control results used for taking a fallback
        info_old
    end

    properties (Access = private)
        % member variable that is used to execute steps on error
        is_run_succeeded (1, 1) logical = false

        got_stop;
        vehs_fallback_times; % record the number of successive fallback times of each vehicle % record the number of successive fallback times of each vehicle
        total_fallback_times; % total times of fallbacks
        vehs_stop_duration;
    end

    methods (Abstract = true, Access = protected)
        controller(obj);
        plan_for_fallback(obj);
    end

    methods
        % Set default settings
        function obj = HighLevelController(scenario, plant)
            % Some default values are invalid and thus they're easily spotted when they haven't been explicitly set
            % We can then either throw an exception or use an arbitrary option when we find a default value
            % Or should we make valid and useful default values?
            obj.k = 0;
            obj.controller_name = '';
            obj.got_stop = false;
            obj.total_fallback_times = 0;
            obj.scenario = scenario;
            obj.timing = ControllerTiming();
            obj.plant = plant;

            obj.mpa = MotionPrimitiveAutomaton(scenario.model, scenario.options);

            initial_state = find([obj.mpa.trims.speed] == 0 & [obj.mpa.trims.steering] == 0, 1);

            for iVeh = 1:scenario.options.amount
                % initialize vehicle ids of all vehicles
                scenario.vehicles(iVeh).trim_config = initial_state;

            end

            % create fallback for first time step
            obj.info_old = ControlResultsInfo(scenario.options.amount, scenario.options.Hp, plant.all_vehicle_ids);

            for vehicle_idx = obj.plant.indices_in_vehicle_list
                k = 1;
                x0 = [[scenario.vehicles.x_start]', [scenario.vehicles.y_start]', [scenario.vehicles.yaw_start]'];
                trim_indices = [scenario.vehicles.trim_config];
                obj.info_old.tree{vehicle_idx} = Tree(x0(vehicle_idx, 1), x0(vehicle_idx, 2), x0(vehicle_idx, 3), trim_indices(vehicle_idx), k, inf, inf);
                obj.info_old.tree_path(vehicle_idx, :) = ones(1, scenario.options.Hp + 1);
                obj.info_old.y_predicted(vehicle_idx) = {repmat([x0(vehicle_idx, 1), x0(vehicle_idx, 2), x0(vehicle_idx, 3), trim_indices(vehicle_idx)], ...
                                                             (scenario.options.tick_per_step + 1) * scenario.options.Hp, 1)};
            end

        end

        function [result, scenario] = run(obj)
            % run the controller

            % object that executes the specified function on destruction
            % this is done at the end of the current function
            cleanupObj = onCleanup(@obj.end_run);

            % initialize the controller and its adapters
            obj.main_init();

            % start controllers main control loop
            obj.main_control_loop();

            % set to true if the controller ran properly
            obj.is_run_succeeded = true;

            % specify returned variables
            result = obj.result;
            scenario = obj.scenario;
        end

        function set_controller_name(obj, name)
            obj.controller_name = name;
        end

    end

    methods (Access = protected)

        function init(obj)
            % initialize high level controller itself

            % initialize all manually controlled vehicles
            for hdv_id = obj.scenario.options.manual_control_config.hdv_ids
                obj.manual_vehicles = ManualVehicle(hdv_id, obj.scenario);
            end

            % initialize iteration data
            obj.iter = IterationData(obj.scenario, obj.plant.all_vehicle_ids);

            % initialize result struct
            obj.result = get_result_struct(obj);

            % record the number of time steps that vehicles
            % consecutively stop and take fallback
            obj.vehs_stop_duration = zeros(obj.scenario.options.amount, 1);
            obj.vehs_fallback_times = zeros(1, obj.scenario.options.amount);
        end

        function update_controlled_vehicles_traffic_info(obj, states_measured, trims_measured)

            % create indices struct only once for efficiency
            idx = indices();

            for iVeh = obj.plant.indices_in_vehicle_list
                % states of controlled vehicles can be measured directly
                obj.iter.x0(iVeh, :) = states_measured(iVeh, :);
                % get own trim
                obj.iter.trim_indices(iVeh) = trims_measured(iVeh);

                % get vehicles currently occupied areas
                obj.iter.occupied_areas{iVeh} = obj.scenario.vehicles(iVeh).get_occupied_areas( ...
                    states_measured(iVeh, idx.x), ...
                    states_measured(iVeh, idx.y), ...
                    states_measured(iVeh, idx.heading), ...
                    obj.scenario.options.offset ...
                );

                % get occupied area of emergency maneuvers for vehicle iVeh
                obj.iter.emergency_maneuvers{iVeh} = obj.mpa.get_global_emergency_maneuvers( ...
                    states_measured(iVeh, idx.x), ...
                    states_measured(iVeh, idx.y), ...
                    states_measured(iVeh, idx.heading), ...
                    trims_measured(iVeh) ...
                );

                % compute reachable sets for vehicle iVeh
                obj.iter.reachable_sets(iVeh, :) = obj.mpa.get_global_reachable_sets( ...
                    states_measured(iVeh, idx.x), ...
                    states_measured(iVeh, idx.y), ...
                    states_measured(iVeh, idx.heading), ...
                    trims_measured(iVeh) ...
                );

                % compute the reference path and speed
                [reference_path_struct, v_ref] = get_reference_trajectory( ...
                    obj.mpa, ...
                    obj.scenario.vehicles(iVeh).reference_path, ...
                    states_measured(iVeh, idx.x), ...
                    states_measured(iVeh, idx.y), ...
                    trims_measured(iVeh), ...
                    obj.scenario.options.dt_seconds ...
                );

                % reference speed
                obj.iter.v_ref(iVeh, :) = v_ref;
                % equidistant points on the reference trajectory.
                obj.iter.reference_trajectory_points(iVeh, :, :) = reference_path_struct.path;
                obj.iter.reference_trajectory_index(iVeh, :, :) = reference_path_struct.points_index;

                if obj.scenario.options.scenario_type ~= ScenarioType.circle

                    % compute the predicted lanelets of vehicle iVeh
                    predicted_lanelets = get_predicted_lanelets(obj.scenario, iVeh, reference_path_struct);
                    obj.iter.predicted_lanelets{iVeh} = predicted_lanelets;

                    % calculate the predicted lanelet boundary of vehicle iVeh based on its predicted lanelets
                    obj.iter.predicted_lanelet_boundary(iVeh, :) = get_lanelets_boundary( ...
                        predicted_lanelets, ...
                        obj.scenario.lanelet_boundary, ...
                        obj.scenario.vehicles(iVeh).lanelets_index, ...
                        obj.scenario.vehicles(iVeh).is_loop ...
                    );

                    % constrain the reachable sets by the boundaries of the predicted lanelets
                    if obj.scenario.options.bound_reachable_sets
                        obj.iter.reachable_sets(iVeh, :) = get_bounded_reachable_sets( ...
                            obj.iter.reachable_sets(iVeh, :), ...
                            obj.iter.predicted_lanelet_boundary{iVeh, 3} ...
                        );
                    end

                end

                % force convex reachable sets if non-convex polygons are not allowed
                if ~obj.scenario.options.is_allow_non_convex
                    obj.iter.reachable_sets(iVeh, :) = cellfun(@(c) convhull(c), ...
                        obj.iter.reachable_sets(iVeh, :), ...
                        'UniformOutput', false ...
                    );
                end

            end

        end

        function create_coupling_graph(~)
            % method that can be overwritten by child classes if necessary
        end

        function check_others_fallback(~)
            % method that can be overwritten by child classes if necessary
        end

        function clean_up(~)
            % release memory allocated by mex functions

            % clear mex on all other computers
            if ~ismac()
                clear mex %#ok
                return
            end

            % clear mex does not work on Mac with ARM chip
            [~, cmdout] = system('sysctl machdep.cpu.brand_string');
            matches = regexp(cmdout, 'machdep.cpu.brand_string: Apple M[1-9]( Pro| Max)?', 'match');

            if isempty(matches)
                clear mex %#ok
                return
            end

            % TODO alternative for clear mex on Mac with ARM chip
            warning('Memory allocation of mex functions is not freed.')
        end

    end

    methods (Access = private)

        function main_init(obj)
            % this function initializes sequentially
            % the main components of the high level controller
            % future: the function initializes the plant and the scenario

            % turn off warning if intersections are detected and fixed, collinear points or
            % overlapping points are removed when using MATLAB function `polyshape`
            warning('off', 'MATLAB:polyshape:repairedBySimplify')

            % start initialization timer
            obj.timing.start("init_all_time");

            % initialize high level controller itself
            obj.init();

            % synchronize with plant
            obj.plant.synchronize_start_with_plant();

            % stop initialization timer
            obj.timing.stop("init_all_time");
        end

        function main_control_loop(obj)

            %% Main control loop
            while (~obj.got_stop)
                % increment interation counter
                obj.k = obj.k + 1;

                if mod(obj.k, 10) == 0
                    % only display 0, 10, 20, ...
                    disp(['>>> Time step ' num2str(obj.k)])
                end

                obj.timing.start("hlc_step_time", obj.k);
                obj.timing.start("iter_runtime", obj.k);

                % Measurement
                % -------------------------------------------------------------------------
                [states_measured, trims_measured] = obj.plant.measure(obj.mpa);

                % Control
                % ----------------------------------------------------------------------

                % update the traffic situation
                obj.update_hdv_traffic_info(states_measured);
                obj.update_controlled_vehicles_traffic_info(states_measured, trims_measured);

                obj.timing.stop("iter_runtime", obj.k);

                % The controller computes plans
                obj.timing.start("controller_time", obj.k);
                obj.timing.start("relation_time", obj.k);

                % determine couplings between the controlled vehicles
                obj.create_coupling_graph();

                obj.timing.stop("relation_time", obj.k);

                %% controller %%
                obj.controller();

                % handle fallback of controller
                if ~obj.handle_fallback()
                    % if fallback is not handled break the main control loop
                    break
                end

                obj.info_old = obj.info; % save variable in case of fallback

                % stop timer of current time step
                obj.timing.stop("controller_time", obj.k);
                obj.timing.stop("hlc_step_time", obj.k);

                % store results from iteration in results struct
                obj.store_iteration_results();

                % reset iter obstacles to scenario default/static obstacles
                obj.iter.obstacles = obj.scenario.obstacles;
                % important: reset lanelet crossing areas
                obj.iter.lanelet_crossing_areas = repmat({{}}, obj.scenario.options.amount, 1);

                % Apply control action
                % -------------------------------------------------------------------------
                obj.plant.apply(obj.info, obj.result, obj.k, obj.mpa);

                % Check for stop signal
                % -------------------------------------------------------------------------
                obj.got_stop = obj.plant.is_stop() || obj.got_stop;
            end

        end

        function update_hdv_traffic_info(obj, states_measured)
            % compute information about traffic situation and coupling with HDVs
            % computed variables are hdv_adjacency, hdv_reachable_sets

            % create indices struct only once for efficiency
            idx = indices();

            % calculate adjacency between CAVs and HDVs and HDV reachable sets
            for iHdv = 1:obj.scenario.options.manual_control_config.amount

                % determine HDV lanelet id based on HDV's position
                state_hdv = states_measured(obj.scenario.options.amount + iHdv, :);
                lanelet_id_hdv = map_position_to_closest_lanelets( ...
                    obj.scenario.lanelets, ...
                    state_hdv(idx.x), ...
                    state_hdv(idx.y) ...
                );

                % calculate the intersection of reachable sets with the current and
                % the successor lanelet (returned as cell array of polyshape objects
                % for each step in the prediction horizon)
                reachable_sets = obj.manual_vehicles(iHdv).compute_reachable_lane( ...
                    state_hdv, ...
                    lanelet_id_hdv ...
                );

                % determine empty polyshape objects
                empty_sets = cellfun(@(c) c.NumRegions == 0, reachable_sets);
                % fill empty reachable sets with a cell array containing an empty array
                obj.iter.hdv_reachable_sets(iHdv, empty_sets) = {[]};
                % convert polyshape in plain array (repeat first point to enclose the shape)
                obj.iter.hdv_reachable_sets(iHdv, ~empty_sets) = cellfun(@(c) ...
                    [c.Vertices(:, 1)', c.Vertices(1, 1)'; c.Vertices(:, 2)', c.Vertices(1, 2)'], ...
                    reachable_sets(~empty_sets), ...
                    'UniformOutput', false ...
                );

                % update reduced coupling adjacency for cav/hdv-pairs
                for iVeh = obj.plant.indices_in_vehicle_list
                    % determine CAV lanelet id based on CAV's position
                    state_cav = states_measured(iVeh, :);
                    % TODO isn't the lanelet_id_cav already available?
                    lanelet_id_cav = map_position_to_closest_lanelets( ...
                        obj.scenario.lanelets, ...
                        state_cav(idx.x), ...
                        state_cav(idx.y) ...
                    );

                    % note coupling if HDV is not behind CAV
                    % if HDV is behind CAV and coupling is noted
                    % the optimizer will probably not find a solution
                    % since the CAV is totally in HDV's reachable set
                    obj.iter.hdv_adjacency(iVeh, iHdv) = ~is_hdv_behind( ...
                        lanelet_id_cav, ...
                        state_cav, ...
                        lanelet_id_hdv, ...
                        state_hdv, ...
                        obj.scenario.lanelets, ...
                        obj.scenario.lanelet_relationships ...
                    );
                end

            end

        end

        function is_fallback_handled = handle_fallback(obj)
            % handle the fallback of the controller
            % if fallback is disabled return
            % boolean to break the main control loop

            % check fallback of other controllers
            obj.check_others_fallback();

            % boolean that is used to break the main control loop
            % initialize it with value false
            is_fallback_handled = false;

            if isempty(obj.info.vehs_fallback)
                % increase counter of vehicles that take fallback
                obj.vehs_fallback_times(obj.info.vehs_fallback) = ...
                    obj.vehs_fallback_times(obj.info.vehs_fallback) + 1;

                % reset fallback counter of vehicles that have no fallback
                obj.vehs_fallback_times(setdiff( ...
                    1:obj.scenario.options.amount, ...
                    obj.info.vehs_fallback ...
                )) = 0;

                % if no fallback occurs return that fallback is handled
                is_fallback_handled = true;
                return
            end

            if obj.scenario.options.fallback_type == FallbackType.no_fallback
                % disabled fallback
                disp('Fallback is disabled. Simulation ends.')
                return
            end

            % print information about occurred fallback
            str_trigger_vehicles = sprintf(' %d', find(obj.info.needs_fallback));
            str_fallback_vehicles = sprintf(' %d', obj.info.vehs_fallback);
            fprintf('%s triggered by%s affects%s\n', ...
                obj.scenario.options.fallback_type, ...
                str_trigger_vehicles, ...
                str_fallback_vehicles ...
            )

            % plan for fallback case
            obj.plan_for_fallback();

            % increase counter of total fallbacks
            obj.total_fallback_times = obj.total_fallback_times + 1;

            % if fallback is handled return that
            is_fallback_handled = true;
        end

        function store_iteration_results(obj)
            % store iteration results like iter and info in the results struct

            % summarize timings from subcontroller
            runtime_relation = obj.timing.get_elapsed_time("relation_time", obj.k);
            obj.info.runtime_subcontroller_each_veh = obj.info.runtime_subcontroller_each_veh + runtime_relation;
            obj.info.runtime_subcontroller_each_grp = obj.info.runtime_subcontroller_each_grp + runtime_relation;

            % calculate deadlock
            % if a vehicle stops for more than a defined time, assume deadlock
            % TODO check if deadlocked vehicles are coupled. Sometimes single
            % vehicles stop because trajectory planner fails to work as intended

            % vehicles that stop at the current time step
            is_vehicle_stopped = ismember(obj.info.trim_indices, obj.mpa.trims_stop);
            % increase couter of vehicles that stop
            obj.vehs_stop_duration(is_vehicle_stopped) = ...
                obj.vehs_stop_duration(is_vehicle_stopped) + 1;
            % reset vehicles that do not stop anymore
            obj.vehs_stop_duration(~is_vehicle_stopped) = 0;

            % check for deadlock
            threshold_stop_steps = 3 * obj.scenario.options.Hp;
            is_vehicle_deadlocked = (obj.vehs_stop_duration > threshold_stop_steps);

            % TODO n_coupled_deadlocked_vehicles
            if any(is_vehicle_deadlocked)
                str_vehicles_deadlocked = sprintf('%4d', find(is_vehicle_deadlocked));
                str_steps_deadlocked = sprintf('%4d', obj.vehs_stop_duration(is_vehicle_deadlocked));
                fprintf('Deadlock vehicle:%s\n', str_vehicles_deadlocked);
                fprintf('       For steps:%s\n', str_steps_deadlocked);
            end

            % update total number of steps and total runtime
            obj.result.nSteps = obj.k;
            obj.result.t_total = obj.k * obj.scenario.options.dt_seconds;

            % store timings in result struct
            obj.result.iter_runtime(obj.k) = obj.timing.get_elapsed_time("iter_runtime", obj.k);
            obj.result.controller_runtime(obj.k) = obj.timing.get_elapsed_time("controller_time", obj.k);
            obj.result.step_time(obj.k) = obj.timing.get_elapsed_time("hlc_step_time", obj.k);

            % store iteration data
            obj.result.obstacles = obj.iter.obstacles;
            obj.result.iteration_structs{obj.k} = obj.iter;
            obj.result.coupling_adjacency(:, :, obj.k) = obj.iter.adjacency;
            obj.result.coupling_info(:, :, obj.k) = obj.iter.coupling_info;
            obj.result.priority_list(:, obj.k) = obj.iter.priority_list;
            obj.result.directed_coupling(:, :, obj.k) = obj.iter.directed_coupling;
            obj.result.weighted_coupling(:, :, obj.k) = obj.iter.weighted_coupling;
            obj.result.directed_coupling_reduced(:, :, obj.k) = obj.iter.directed_coupling_reduced;
            obj.result.weighted_coupling_reduced(:, :, obj.k) = obj.iter.weighted_coupling_reduced;
            obj.result.lanelet_crossing_areas(:, obj.k) = obj.iter.lanelet_crossing_areas;
            obj.result.belonging_vector(:, obj.k) = obj.iter.belonging_vector;
            obj.result.parl_groups_info{obj.k} = obj.iter.parl_groups_info;

            % store graph search results
            obj.result.trajectory_predictions(:, obj.k) = obj.info.y_predicted;
            obj.result.controller_outputs{obj.k} = obj.info.u;
            obj.result.vehicle_path_fullres(:, obj.k) = obj.info.vehicle_fullres_path(:);
            obj.result.n_expanded(:, obj.k) = obj.info.n_expanded;
            obj.result.vehs_fallback{obj.k} = obj.info.vehs_fallback;

            % store graph search timings
            obj.result.computation_levels(obj.k) = obj.info.computation_levels;
            obj.result.subcontroller_runtime_each_veh(:, obj.k) = obj.info.runtime_subcontroller_each_veh;
            obj.result.graph_search_runtime_each_veh(:, obj.k) = obj.info.runtime_graph_search_each_veh;
            obj.result.runtime_subcontroller_max(obj.k) = obj.info.runtime_subcontroller_max;
            obj.result.runtime_graph_search_max(obj.k) = obj.info.runtime_graph_search_max;

            % store calculated values
            obj.result.is_deadlock(obj.k) = any(is_vehicle_deadlocked);

        end

        function end_run(obj)
            % end run of controller
            % this function is executed in every case

            % if the controller did not succeed
            if ~obj.is_run_succeeded
                % force saving of unfinished results for inspection
                disp("Saving of unfinished results on error.")
                obj.scenario.options.should_save_result = true;

                % define output path on error
                obj.result.output_path = 'results/unfinished_result.mat';
            else
                % define output path on success
                obj.result.output_path = FileNameConstructor.get_results_full_path( ...
                    obj.scenario.options, ...
                    obj.plant.indices_in_vehicle_list ...
                );
            end

            % save finished or unfinished results
            obj.save_results();

            % run plant's end_run function
            obj.plant.end_run();

            % clean up controller
            obj.clean_up();
        end

        function save_results(obj)
            % save results at end of experiment

            % print information about final values of counter
            fprintf('Total times of fallback: %d\n', obj.total_fallback_times);
            fprintf('Total runtime: %f seconds\n', obj.result.t_total);

            if ~obj.scenario.options.should_save_result
                % return results should not be saved
                fprintf('As required, results were not saved\n');
                return
            end

            obj.result.mpa = obj.mpa;
            obj.result.scenario = obj.scenario;
            obj.result.timings = obj.timing.get_all_timings();
            obj.result.total_fallback_times = obj.total_fallback_times;

            if obj.scenario.options.should_reduce_result
                % delete large data fields of to reduce file size

                obj.result.mpa = [];

                for i_step = 1:length(obj.result.iteration_structs)
                    obj.result.iteration_structs{i_step}.predicted_lanelets = [];
                    obj.result.iteration_structs{i_step}.predicted_lanelet_boundary = [];
                    obj.result.iteration_structs{i_step}.reachable_sets = [];
                    obj.result.iteration_structs{i_step}.emergency_maneuvers = [];
                    obj.result.iteration_structs{i_step}.occupied_areas = [];
                end

            end

            result = obj.result;
            save(obj.result.output_path, 'result');
            fprintf('Results were saved under: %s\n', obj.result.output_path);

        end

    end

end
