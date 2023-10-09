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
        belonging_vector_total;
        info_old; % old control results for fallback
    end

    properties (Access = private)
        initialized_reference_path;
        got_stop;
        success; % to store unfinished results on error; set to true at the end of the main control loop
        vehs_fallback_times; % record the number of successive fallback times of each vehicle % record the number of successive fallback times of each vehicle
        total_fallback_times; % total times of fallbacks
        vehs_stop_duration;
        timing (1, 1) ControllerTiming;
    end

    methods (Abstract = true, Access = protected)
        controller(obj);
        handle_fallback(obj);
    end

    methods
        % Set default settings
        function obj = HighLevelController(scenario, plant)
            % Some default values are invalid and thus they're easily spotted when they haven't been explicitly set
            % We can then either throw an exception or use an arbitrary option when we find a default value
            % Or should we make valid and useful default values?
            obj.k = 0;
            obj.controller_name = '';
            obj.initialized_reference_path = false;
            obj.got_stop = false;
            obj.success = false;
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
            obj.init_all();
            obj.main_control_loop();
            obj.save_results();

            if obj.scenario.options.use_cpp()

                if ismac()
                    % clear mex dont work on ARM Mac
                    [~, result] = system('sysctl machdep.cpu.brand_string');
                    matches = regexp(result, 'machdep.cpu.brand_string: Apple M[1-9]( Pro| Max)?', 'match');

                    if isempty(matches)
                        clear mex;
                    end

                else
                    clear mex;
                end

            end

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
            obj.iter = IterationData(obj.scenario, obj.k, obj.plant.all_vehicle_ids);

            % initialize result struct
            obj.result = get_result_struct(obj);

            % record the number of time steps that vehicles
            % consecutively stop and take fallback
            obj.vehs_stop_duration = zeros(obj.scenario.options.amount, 1);
            obj.vehs_fallback_times = zeros(1, obj.scenario.options.amount);
        end

        function check_fallback(~)
            % method that can be overwritten by child classes if necessary
        end

        function clean_up(obj)

            if ~obj.success
                disp("Storing unfinished results up on error:")
                % Don't store the last time step with erroneous data.
                obj.k = obj.k - 1;
                % Save the unfinished results.
                obj.scenario.options.should_save_result = true;
                obj.result.output_path = 'results/unfinished_result.mat';
                obj.save_results();
            end

        end

    end

    methods (Access = private)

        function init_all(obj)
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
            cleanup = onCleanup(@obj.clean_up);

            %% Main control loop
            while (~obj.got_stop)
                % increment interation counter
                obj.k = obj.k + 1;

                obj.iter.k = obj.k;

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
                obj.initialized_reference_path = true;

                % calculate the distance
                distance = zeros(obj.scenario.options.amount, obj.scenario.options.amount);
                adjacency = obj.iter.adjacency;

                for jVeh = 1:obj.scenario.options.amount - 1
                    adjacent_vehicle = find(adjacency(jVeh, :));
                    adjacent_vehicle = adjacent_vehicle(adjacent_vehicle > jVeh);

                    for vehn = adjacent_vehicle
                        distance(jVeh, vehn) = check_distance(obj.iter, jVeh, vehn);
                    end

                end

                obj.result.distance(:, :, obj.k) = distance;
                obj.result.iter_runtime(obj.k) = obj.timing.stop("iter_runtime", obj.k);

                % The controller computes plans
                obj.timing.start("controller_time", obj.k);

                %% controller %%
                obj.controller();

                %% collect fallbacks %%
                obj.check_fallback();

                %% fallback
                if obj.scenario.options.fallback_type == FallbackType.no_fallback
                    % disabled fallback
                    if ~isempty(obj.info.vehs_fallback)
                        disp('Fallback is disabled. Simulation ends.')
                        obj.result.distance(:, :, obj.k) = [];
                        obj.result.iter_runtime(obj.k) = [];
                        break
                    end

                else
                    obj.vehs_fallback_times(obj.info.vehs_fallback) = obj.vehs_fallback_times(obj.info.vehs_fallback) + 1;
                    vehs_not_fallback = setdiff(1:obj.scenario.options.amount, obj.info.vehs_fallback);
                    obj.vehs_fallback_times(vehs_not_fallback) = 0; % reset

                    % check whether at least one vehicle has fallen back Hp times successively

                    if ~isempty(obj.info.vehs_fallback)
                        i_triggering_vehicles = find(obj.info.needs_fallback);

                        str_veh = sprintf('%d ', i_triggering_vehicles);
                        str_fb_type = sprintf('triggering %s', char(obj.scenario.options.fallback_type));
                        disp_tmp = sprintf(' %d,', obj.info.vehs_fallback); disp_tmp(end) = [];
                        disp(['Vehicle ', str_veh, str_fb_type, ', affecting vehicle' disp_tmp '.'])

                        % plan for fallback case
                        obj.handle_fallback();
                        obj.total_fallback_times = obj.total_fallback_times + 1;
                    end

                end

                obj.info_old = obj.info; % save variable in case of fallback
                %% save result of current time step
                obj.result.controller_runtime(obj.k) = obj.timing.stop("controller_time", obj.k);
                obj.timing.stop("hlc_step_time", obj.k);

                % save controller outputs in result struct
                obj.result.scenario = obj.scenario;
                obj.result.is_deadlock(obj.k) = 0;
                obj.result.iteration_structs{obj.k} = obj.iter;
                obj.result.trajectory_predictions(:, obj.k) = obj.info.y_predicted;
                obj.result.controller_outputs{obj.k} = obj.info.u;
                obj.result.subcontroller_runtime_each_veh(:, obj.k) = obj.info.runtime_subcontroller_each_veh;
                obj.result.graph_search_runtime_each_veh(:, obj.k) = obj.info.runtime_graph_search_each_veh;
                obj.result.vehicle_path_fullres(:, obj.k) = obj.info.vehicle_fullres_path(:);
                obj.result.n_expanded(:, obj.k) = obj.info.n_expanded;
                obj.result.priority_list(:, obj.k) = obj.iter.priority_list;
                obj.result.coupling_adjacency(:, :, obj.k) = obj.iter.adjacency;
                obj.result.computation_levels(obj.k) = obj.info.computation_levels;
                obj.result.step_time(obj.k) = obj.timing.get_elapsed_time("hlc_step_time", obj.k);
                obj.result.obstacles = obj.iter.obstacles;
                % reset iter obstacles to scenario default/static obstacles
                obj.iter.obstacles = obj.scenario.obstacles;

                obj.result.runtime_subcontroller_max(obj.k) = obj.info.runtime_subcontroller_max;
                obj.result.runtime_graph_search_max(obj.k) = obj.info.runtime_graph_search_max;
                obj.result.directed_coupling{obj.k} = obj.iter.directed_coupling;

                if obj.scenario.options.is_prioritized && obj.scenario.options.scenario_type == ScenarioType.commonroad
                    obj.result.determine_couplings_time(obj.k) = obj.iter.timer.determine_couplings;
                    obj.result.group_vehs_time(obj.k) = obj.iter.timer.group_vehs;
                    obj.result.assign_priority_time(obj.k) = obj.iter.timer.assign_priority;
                    obj.result.num_couplings(obj.k) = nnz(obj.iter.directed_coupling);
                    obj.result.num_couplings_ignored(obj.k) = nnz(obj.iter.directed_coupling) - nnz(obj.iter.directed_coupling_reduced);
                    obj.result.num_couplings_between_grps(obj.k) = obj.iter.num_couplings_between_grps;
                    obj.result.num_couplings_between_grps_ignored(obj.k) = obj.iter.num_couplings_between_grps_ignored;
                    obj.result.belonging_vector(:, obj.k) = obj.iter.belonging_vector;
                    obj.result.weighted_coupling_reduced{obj.k} = obj.iter.weighted_coupling_reduced;
                    obj.result.coupling_info{obj.k} = obj.iter.coupling_info;
                    obj.result.parl_groups_info{obj.k} = obj.iter.parl_groups_info;
                    obj.result.lanelet_crossing_areas{obj.k} = obj.iter.lanelet_crossing_areas;
                    % important: reset lanelet crossing areas
                    obj.iter.lanelet_crossing_areas = {};
                end

                obj.result.vehs_fallback{obj.k} = obj.info.vehs_fallback;

                % check if deadlock occurs
                % if a vehicle stops for more than a defined time, assume deadlock
                % TODO check if deadlocked vehicles are coupled. Sometimes single
                % vehicles stop because trajectory planner fails to work as intended
                vehs_stop = any(ismember(obj.info.trim_indices, obj.mpa.trims_stop), 2); % vehicles stop at the current time step
                obj.vehs_stop_duration(vehs_stop) = obj.vehs_stop_duration(vehs_stop) + 1;
                obj.vehs_stop_duration(~vehs_stop) = 0; % reset others

                threshold_stop_steps = 3 * obj.scenario.options.Hp;
                vehs_deadlocked = (obj.vehs_stop_duration > threshold_stop_steps);
                n_deadlocked_vehicles = nnz(vehs_deadlocked);
                % TODO n_coupled_deadlocked_vehicles
                if n_deadlocked_vehicles > 0
                    veh_idcs_deadlocked = find(vehs_deadlocked);
                    veh_str = sprintf("%4d", veh_idcs_deadlocked);
                    t_str = sprintf("%4d", obj.vehs_stop_duration(vehs_deadlocked));
                    fprintf("Deadlock. Vehicle:%s\n", veh_str);
                    fprintf("    For timesteps:%s\n", t_str)
                    obj.result.is_deadlock(obj.k) = 1;
                end

                % Apply control action
                % -------------------------------------------------------------------------
                obj.plant.apply(obj.info, obj.result, obj.k, obj.mpa);

                % Check for stop signal
                % -------------------------------------------------------------------------
                obj.got_stop = obj.plant.is_stop() || obj.got_stop;
            end

            obj.success = true;
        end

        function save_results(obj)
            %% save results at end of experiment
            obj.result.total_fallback_times = obj.total_fallback_times;
            disp(['Total times of fallback: ' num2str(obj.total_fallback_times) '.'])

            obj.result.t_total = obj.k * obj.scenario.options.dt_seconds;
            obj.result.nSteps = obj.k;
            obj.result.timings = obj.timing.get_all_timings();

            disp(['Total runtime: ' num2str(round(obj.result.t_total, 2)) ' seconds.'])

            if obj.scenario.options.should_save_result
                obj.result.mpa = obj.mpa;

                % Delete unimportant data
                if obj.scenario.options.should_reduce_result

                    for iIter = 1:length(obj.result.iteration_structs)
                        obj.result.iteration_structs{iIter}.predicted_lanelet_boundary = [];
                        obj.result.iteration_structs{iIter}.predicted_lanelets = [];
                        obj.result.iteration_structs{iIter}.reachable_sets = [];
                        obj.result.iteration_structs{iIter}.occupied_areas = [];
                        obj.result.iteration_structs{iIter}.emergency_maneuvers = [];
                    end

                    obj.result.mpa = [];
                end

                % check if file with the same name exists
                % while isfile(result.output_path)
                %     warning('File with the same name exists, timestamp will be added to the file name.')
                %     result.output_path = [result.output_path(1:end-4), '_', datestr(now,'yyyymmddTHHMMSS'), '.mat']; % move '.mat' to end
                % end

                result = obj.result;
                save(obj.result.output_path, 'result');
                disp(['Simulation results were saved under ' obj.result.output_path])
            else
                disp('As required, simulation/Experiment Results were not saved.')
                % exportVideo( result );
            end

            obj.plant.end_run()

        end

    end

end
