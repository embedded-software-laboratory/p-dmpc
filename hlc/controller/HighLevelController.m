classdef (Abstract) HighLevelController < handle

    properties (Access = public)
        % config
        options;

        % Adapter for creating the scenario
        scenario_adapter;

        % Adapter for the lab
        % or one for a local simulation
        plant;

        optimizer;
        mpa;
        experiment_result;
        k;
        iter;
        info;

    end

    properties (Access = protected)
        timing (1, 1) ControllerTiming;
        coupler

        % old control results used for taking a fallback
        info_old
    end

    properties (Access = private)
        % member variable that is used to execute steps on error
        is_run_succeeded (1, 1) logical = false

        vehicles_fallback_times; % record the number of successive fallback times of each vehicle % record the number of successive fallback times of each vehicle
        total_fallback_times; % total times of fallbacks
        vehs_stop_duration;
    end

    methods (Abstract = true, Access = protected)
        controller(obj);
        plan_for_fallback(obj);
    end

    methods
        % Set default settings
        function obj = HighLevelController(options, plant)

            if nargin == 0
                return
            end

            % remove step time from options to avoid usage
            % before it is received from the plant
            options.dt_seconds = [];

            obj.options = options;
            obj.scenario_adapter = ScenarioAdapter.get_scenario_adapter(options.scenario_type);
            obj.plant = plant;

            obj.k = 0;
            obj.total_fallback_times = 0;

            obj.coupler = Coupler.get_coupler(obj.options.coupling, obj.options.amount);
            obj.timing = ControllerTiming();

        end

        function experiment_result = run(obj)
            % run the controller

            % object that executes the specified function on destruction
            cleanupObj = onCleanup(@obj.end_run);

            % initialize the controller and its adapters
            obj.main_init();

            % synchronize with other controllers
            obj.synchronize_start_with_other_controllers();

            % synchronize with plant
            obj.synchronize_start_with_plant();

            % start controllers main control loop
            obj.main_control_loop();

            % set to true if the controller ran properly
            obj.set_run_succeeded(true);

            % This triggers obj.end_run such that the returned experiment_result contains all information.
            % It is only reached if onCleanup was not already destroyed before because of an error.
            delete(cleanupObj);

            % specify returned variables
            experiment_result = obj.experiment_result;
        end

    end

    methods (Access = protected)

        function set_run_succeeded(obj, is_run_succeeded)
            obj.is_run_succeeded = is_run_succeeded;
        end

        function init(obj)
            % initialize high level controller itself

            % create vehicle model from information of controlled vehicle
            % if more than one vehicle is controlled assume that they all have
            % the same dimensions and take the dimensions of the first one
            bicycle_model = BicycleModel( ...
                obj.scenario_adapter.scenario.vehicles(obj.plant.vehicle_indices_controlled(1)).Lf, ...
                obj.scenario_adapter.scenario.vehicles(obj.plant.vehicle_indices_controlled(1)).Lr ...
            );

            % construct mpa
            obj.mpa = MotionPrimitiveAutomaton(bicycle_model, obj.options);

            % initialize ExperimentResult object
            obj.experiment_result = ExperimentResult(obj.options, obj.scenario_adapter.scenario, obj.mpa, obj.plant.vehicle_indices_controlled);

            % initialize iteration data
            obj.iter = IterationData( ...
                obj.options, ...
                obj.scenario_adapter.scenario ...
            );

            % create old control results info in case of fallback at first time step
            obj.info_old = ControlResultsInfo( ...
                numel(obj.plant.vehicle_indices_controlled), ...
                obj.options.Hp ...
            );

            % measure vehicles' initial poses and trims
            [cav_measurements] = obj.plant.measure();

            % fill control results info for each controlled vehicle with measurement information
            for i_vehicle = 1:numel(obj.plant.vehicle_indices_controlled)
                vehicle_index = obj.plant.vehicle_indices_controlled(i_vehicle);
                % get initial pose from measurement
                initial_pose = [ ...
                                    cav_measurements(vehicle_index).x, ...
                                    cav_measurements(vehicle_index).y, ...
                                    cav_measurements(vehicle_index).yaw, ...
                                ];
                % find initial trim of the controlled vehicle
                initial_trim = obj.mpa.trim_from_values( ...
                    cav_measurements(vehicle_index).speed, ...
                    cav_measurements(vehicle_index).steering ...
                );

                % initialize info_old
                obj.info_old.tree = AStarTree(initial_pose(1), initial_pose(2), initial_pose(3), initial_trim, 1, inf, inf);
                obj.info_old.tree_path = ones(1, obj.options.Hp + 1);
                obj.info_old.y_predicted(:, :, i_vehicle) = repmat( ...
                    [initial_pose(1); initial_pose(2); initial_pose(3)], ...
                    1, ...
                    obj.options.Hp ...
                );
            end

            % record the number of time steps that vehicles
            % consecutively stop and take fallback
            obj.vehs_stop_duration = zeros(obj.options.amount, 1);
            obj.vehicles_fallback_times = zeros(1, obj.options.amount);
        end

        function update_controlled_vehicles_traffic_info(obj, cav_measurements)

            obj.timing.start("update_controlled_vehicles_traffic_info", obj.k);

            for iVeh = obj.plant.vehicle_indices_controlled
                % states of controlled vehicles can be measured directly
                obj.iter.x0(iVeh, :) = [ ...
                                            cav_measurements(iVeh).x, ...
                                            cav_measurements(iVeh).y, ...
                                            cav_measurements(iVeh).yaw, ...
                                            cav_measurements(iVeh).speed ...
                                        ];

                % get own trim
                obj.iter.trim_indices(iVeh) = obj.mpa.trim_from_values( ...
                    cav_measurements(iVeh).speed, ...
                    cav_measurements(iVeh).steering ...
                );

                % get vehicles currently occupied areas
                obj.iter.occupied_areas{iVeh} = get_occupied_areas( ...
                    cav_measurements(iVeh).x, ...
                    cav_measurements(iVeh).y, ...
                    cav_measurements(iVeh).yaw, ...
                    obj.scenario_adapter.scenario.vehicles(iVeh).Length, ...
                    obj.scenario_adapter.scenario.vehicles(iVeh).Width, ...
                    obj.options.offset ...
                );

                % get occupied area of emergency maneuvers for vehicle iVeh
                obj.iter.emergency_maneuvers{iVeh} = obj.mpa.emergency_maneuvers_at_pose( ...
                    cav_measurements(iVeh).x, ...
                    cav_measurements(iVeh).y, ...
                    cav_measurements(iVeh).yaw, ...
                    obj.iter.trim_indices(iVeh) ...
                );

                % compute reachable sets for vehicle iVeh
                obj.iter.reachable_sets(iVeh, :) = obj.mpa.reachable_sets_at_pose( ...
                    cav_measurements(iVeh).x, ...
                    cav_measurements(iVeh).y, ...
                    cav_measurements(iVeh).yaw, ...
                    obj.iter.trim_indices(iVeh) ...
                );

                % compute the reference path and speed
                [reference_trajectory_struct, v_ref, current_point_index] = get_reference_trajectory( ...
                    obj.mpa, ...
                    obj.scenario_adapter.scenario.vehicles(iVeh).reference_path, ...
                    cav_measurements(iVeh).x, ...
                    cav_measurements(iVeh).y, ...
                    obj.iter.trim_indices(iVeh), ...
                    obj.options.dt_seconds ...
                );

                % reference speed
                obj.iter.v_ref(iVeh, :) = v_ref;
                % equidistant points on the reference trajectory.
                obj.iter.reference_trajectory_points(iVeh, :, :) = reference_trajectory_struct.path;
                obj.iter.reference_trajectory_index(iVeh, :, :) = reference_trajectory_struct.points_index;

                if obj.options.scenario_type ~= ScenarioType.circle

                    % compute the predicted lanelets of vehicle iVeh
                    [predicted_lanelets, current_lanelet] = get_predicted_lanelets( ...
                        obj.scenario_adapter.scenario.vehicles(iVeh).reference_path, ...
                        obj.scenario_adapter.scenario.vehicles(iVeh).points_index, ...
                        obj.scenario_adapter.scenario.vehicles(iVeh).lanelets_index, ...
                        reference_trajectory_struct, ...
                        current_point_index ...
                    );
                    obj.iter.predicted_lanelets{iVeh} = predicted_lanelets;
                    obj.iter.current_lanelet(iVeh) = current_lanelet;

                    % calculate the predicted lanelet boundary of vehicle iVeh based on its predicted lanelets
                    obj.iter.predicted_lanelet_boundary(iVeh, :) = get_lanelets_boundary( ...
                        predicted_lanelets, ...
                        obj.scenario_adapter.scenario.lanelet_boundary, ...
                        obj.scenario_adapter.scenario.vehicles(iVeh).lanelets_index, ...
                        obj.scenario_adapter.scenario.vehicles(iVeh).is_loop ...
                    );

                    % constrain the reachable sets by the boundaries of the predicted lanelets
                    if obj.options.is_bounded_reachable_set_used
                        obj.iter.reachable_sets(iVeh, :) = bound_reachable_sets( ...
                            obj.iter.reachable_sets(iVeh, :), ...
                            obj.iter.predicted_lanelet_boundary{iVeh, 3} ...
                        );
                    end

                end

                % force convex reachable sets if non-convex polygons are not allowed
                if ~obj.options.are_any_obstacles_non_convex
                    obj.iter.reachable_sets(iVeh, :) = cellfun(@(c) convhull(c), ...
                        obj.iter.reachable_sets(iVeh, :), ...
                        UniformOutput = false ...
                    );
                end

            end

            obj.timing.stop("update_controlled_vehicles_traffic_info", obj.k);

        end

        function create_coupling_graph(~)
            % method that can be overwritten by child classes if necessary
        end

        function synchronize_start_with_other_controllers(~)
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

            warning('Memory allocation of mex functions is not freed.')
        end

        function main_init(obj)
            % this function initializes sequentially
            % the main components of the high level controller
            % future: the function initializes the plant and the scenario

            % turn off warning if intersections are detected and fixed, collinear points or
            % overlapping points are removed when using MATLAB function `polyshape`
            warning('off', 'MATLAB:polyshape:repairedBySimplify')

            % start initialization timer
            obj.timing.start("hlc_init_all");

            % receive step time from the plant
            obj.options.dt_seconds = obj.plant.get_step_time();

            % initialize scenario adapter
            obj.scenario_adapter.init(obj.options, obj.plant);

            % initialize high level controller itself
            obj.init();

            % stop initialization timer
            obj.timing.stop("hlc_init_all");
        end

        % end

        % methods (Access = private)

        function synchronize_start_with_plant(obj)
            obj.plant.synchronize_start_with_plant();
        end

        function main_control_loop(obj)

            %% Main control loop
            while (~obj.should_stop())
                % increment interation counter
                obj.increment_time_step();

                if mod(obj.k, 10) == 0
                    % only display 0, 10, 20, ...
                    disp(['>>> Time step ' num2str(obj.k)])
                end

                % Measurement
                % -------------------------------------------------------------------------
                obj.update_traffic();

                % Control
                % ----------------------------------------------------------------------

                % determine couplings between the controlled vehicles
                obj.create_coupling_graph();

                % controller
                obj.controller();

                % handle fallback of controller

                if ~obj.handle_fallback()
                    % if fallback is not handled break the main control loop
                    break
                end

                % store results from iteration in ExperimentResult
                obj.store_control_info();
                obj.store_iteration_results();
                obj.reset_control_loop_data();

                % Apply control action
                % -------------------------------------------------------------------------
                obj.apply();

            end

        end

        function increment_time_step(obj)
            obj.k = obj.k + 1;
        end

        function update_traffic(obj)
            obj.timing.start("control_loop", obj.k);

            [cav_measurements, hdv_measurements] = obj.measure();
            % update the traffic situation
            obj.update_controlled_vehicles_traffic_info(cav_measurements);
            obj.update_hdv_traffic_info(cav_measurements, hdv_measurements);
        end

        function [cav_measurements, hdv_measurements] = measure(obj)
            obj.timing.start("measure", obj.k);
            [cav_measurements, hdv_measurements] = obj.plant.measure();
            obj.timing.stop("measure", obj.k);
        end

        function update_hdv_traffic_info(obj, cav_measurements, hdv_measurements)
            % compute information about traffic situation and coupling with HDVs
            % computed variables are hdv_adjacency, hdv_reachable_sets

            % calculate adjacency between CAVs and HDVs and HDV reachable sets
            for iHdv = 1:obj.options.manual_control_config.amount

                % determine HDV lanelet id based on HDV's position
                lanelet_id_hdv = map_position_to_closest_lanelets( ...
                    obj.scenario_adapter.scenario.lanelets, ...
                    hdv_measurements(iHdv).x, ...
                    hdv_measurements(iHdv).y ...
                );

                % calculate the intersection of reachable sets with the current and
                % the successor lanelet (returned as cell array of polyshape objects
                % for each step in the prediction horizon)
                reachable_sets = obj.scenario_adapter.manual_vehicles(iHdv).compute_reachable_lane( ...
                    hdv_measurements(iHdv), ...
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
                    UniformOutput = false ...
                );

                % update reduced coupling adjacency for cav/hdv-pairs
                for iVeh = obj.plant.vehicle_indices_controlled
                    lanelet_id_cav = obj.iter.current_lanelet(iVeh);

                    % note coupling if HDV is not behind CAV
                    % if HDV is behind CAV and coupling is noted
                    % the optimizer will probably not find a solution
                    % since the CAV is totally in HDV's reachable set
                    obj.iter.hdv_adjacency(iVeh, iHdv) = ~is_hdv_behind( ...
                        lanelet_id_cav, ...
                        cav_measurements(iHdv), ...
                        lanelet_id_hdv, ...
                        hdv_measurements(iHdv), ...
                        obj.scenario_adapter.scenario.lanelets, ...
                        obj.scenario_adapter.scenario.lanelet_relationships ...
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

            if isempty(obj.info.vehicles_fallback)
                % increase counter of vehicles that take fallback
                obj.vehicles_fallback_times(obj.info.vehicles_fallback) = ...
                    obj.vehicles_fallback_times(obj.info.vehicles_fallback) + 1;

                % reset fallback counter of vehicles that have no fallback
                obj.vehicles_fallback_times(setdiff( ...
                    1:obj.options.amount, ...
                    obj.info.vehicles_fallback ...
                )) = 0;

                % if no fallback occurs return that fallback is handled
                is_fallback_handled = true;
                return
            end

            if obj.options.fallback_type == FallbackType.no_fallback
                % disabled fallback
                disp('Fallback is disabled. Simulation ends.')
                return
            end

            % plan for fallback case
            obj.plan_for_fallback();

            % increase counter of total fallbacks
            obj.total_fallback_times = obj.total_fallback_times + 1;

            % if fallback is handled return that
            is_fallback_handled = true;
        end

        function store_control_info(obj)
            % store control results info of previous time step
            obj.info_old = obj.info;
        end

        function reset_control_loop_data(obj)
            % reset iter obstacles to scenario default/static obstacles
            obj.iter.obstacles = obj.scenario_adapter.scenario.obstacles;
            % important: reset lanelet crossing areas
            obj.iter.lanelet_crossing_areas = repmat({{}}, obj.options.amount, 1);
        end

        function store_iteration_results(obj)
            % store iteration results like iter and info in the ExperimentResult object

            % calculate deadlock
            % if a vehicle stops for more than a defined time, assume deadlock

            % vehicles that stop at the current time step
            is_vehicle_stopped = ismember( ...
                obj.info.predicted_trims(:, 1), ...
                obj.mpa.trims_stop ...
            );
            % increase couter of vehicles that stop
            obj.vehs_stop_duration(is_vehicle_stopped) = ...
                obj.vehs_stop_duration(is_vehicle_stopped) + 1;
            % reset vehicles that do not stop anymore
            obj.vehs_stop_duration(~is_vehicle_stopped) = 0;

            % check for deadlock
            threshold_stop_steps = 3 * obj.options.Hp;
            is_vehicle_deadlocked = (obj.vehs_stop_duration > threshold_stop_steps);

            if any(is_vehicle_deadlocked)
                str_vehicles_deadlocked = sprintf('%4d', find(is_vehicle_deadlocked));
                str_steps_deadlocked = sprintf('%4d', obj.vehs_stop_duration(is_vehicle_deadlocked));
                fprintf('Deadlock vehicle:%s\n', str_vehicles_deadlocked);
                fprintf('       For steps:%s\n', str_steps_deadlocked);
            end

            % store iteration data
            obj.experiment_result.iteration_data(obj.k) = IterationData.clean(obj.iter);

            % store graph search results
            obj.experiment_result.control_results_info(obj.k) = ControlResultsInfo.clean(obj.info);
        end

        function apply(obj)
            obj.plant.apply(obj.info, obj.experiment_result, obj.k, obj.mpa);
            obj.timing.stop('control_loop', obj.k);
        end

        function result = should_stop(obj)
            % Check for stop signal
            % -------------------------------------------------------------------------
            if obj.plant.should_stop()
                result = true;
                disp('HLC stopped by plant adapter.');
            elseif obj.k >= obj.options.k_end
                result = true;
                disp('HLC stopped as the experiment is finished.')
            else
                result = false;
            end

        end

        function end_run(obj)
            % end run of controller
            % this function is executed in every case

            % save finished or unfinished ExperimentResult
            obj.save_results();

            % run plant's end_run function
            obj.plant.end_run();

            % clean up controller
            obj.clean_up();
        end

        function save_results(obj)
            % save ExperimentResult object at end of experiment

            % print information about final values of counter
            fprintf('Total times of fallback: %d\n', obj.total_fallback_times);
            fprintf('Total runtime: %f seconds\n', obj.experiment_result.t_total);

            obj.experiment_result.mpa = obj.mpa;
            obj.experiment_result.scenario = obj.scenario_adapter.scenario;

            obj.experiment_result.timing = obj.timing.get_all_timings();

            % if the controller did not succeed
            if ~obj.is_run_succeeded
                % force saving of unfinished ExperimentResult object for inspection
                disp("Saving of unfinished results on error.")
                obj.options.should_save_result = true;

                % define output path on error
                vehicle_indices_string = sprintf('_%02d', obj.plant.vehicle_indices_controlled);
                output_path = ['results/unfinished_result', vehicle_indices_string, '.mat'];
            else
                % define output path on success
                output_path = FileNameConstructor.get_results_full_path( ...
                    obj.options, ...
                    obj.plant.vehicle_indices_controlled ...
                );
            end

            if ~obj.options.should_save_result
                % return ExperimentResult object should not be saved
                fprintf('As required, experiment_result was not saved\n');
                return
            end

            experiment_result = obj.experiment_result; %#ok<PROP>
            save(output_path, 'experiment_result');
            fprintf('Results were saved in: %s\n', output_path);

        end

    end

end
