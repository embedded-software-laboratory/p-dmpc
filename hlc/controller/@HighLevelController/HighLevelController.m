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

        function relate_vehicles(~)
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

                obj.timing.start("hlc_step_time", obj.k);
                obj.timing.start("iter_runtime", obj.k);

                % Measurement
                % -------------------------------------------------------------------------
                [x0_measured, trims_measured] = obj.plant.measure(obj.mpa); % trims_measured： which trim

                if mod(obj.k, 10) == 0
                    % only display 0, 10, 20, ...
                    disp(['>>> Time step ' num2str(obj.k)])
                end

                % Control
                % ----------------------------------------------------------------------

                % Update the iteration data and sample reference trajectory
                obj.rhc_init(x0_measured, trims_measured);

                obj.timing.stop("iter_runtime", obj.k);

                % The controller computes plans
                obj.timing.start("controller_time", obj.k);
                obj.timing.start("relation_time", obj.k);

                % calculate information for the relations between the controlled vehicles
                obj.relate_vehicles();

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
                obj.iter.lanelet_crossing_areas = cell(obj.scenario.options.amount, 1);

                % Apply control action
                % -------------------------------------------------------------------------
                obj.plant.apply(obj.info, obj.result, obj.k, obj.mpa);

                % Check for stop signal
                % -------------------------------------------------------------------------
                obj.got_stop = obj.plant.is_stop() || obj.got_stop;
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
