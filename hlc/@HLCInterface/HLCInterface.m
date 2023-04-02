classdef (Abstract) HLCInterface < handle
    %TODO check for private/protected vars
    properties (Access=public)
        % For which vehicle IDs to start the HLC
        % Must be an array of vehicles. In distributed HLC case the array contains
        % exactly 1 integer
        vehicle_ids;
        
        % amount of vehicle controlled by this HLC
        amount;

        % indices of vehicleList relevant for this HLC (e.g contains 1 index iff hlc.amount = 1)
        indices_in_vehicle_list;
        
        % scenario
        scenario;

        % Adapter for the lab
        % or one for a local simulation
        hlc_adapter;

        % Ros Subscribers for Inter HLC Communication (distributed HLCs) or
        % to simualate distributed communication in pb-sequential controller
        ros_subscribers;

        sub_controller = @graph_search;
        controller_name;
        result;
        k;
        iter;
        info;

        manual_vehicles;

    end
    properties (Access=protected)
        belonging_vector_total;
    end
    properties (Access=private)
        initialized_reference_path;
        got_stop;
        success; % to store unfinished results on error; set to true at the end of the main control loop
        speedProfileMPAsInitialized;
        cooldown_after_lane_change;
        cooldown_second_manual_vehicle_after_lane_change;
        vehs_fallback_times; % record the number of successive fallback times of each vehicle% record the number of successive fallback times of each vehicle
        info_old; % old information for fallback
        total_fallback_times; % total times of fallbacks
        vehs_stop_duration;
    end

    methods
        % Set default settings
        function obj = HLCInterface()
            % Some default values are invalid and thus they're easily spotted when they haven't been explicitly set
            % We can then either throw an exception or use an arbitrary option when we find a default value
            % Or should we make valid and useful default values?
            obj.vehicle_ids = [];
            obj.amount = 0;
            obj.indices_in_vehicle_list = [];
            obj.ros_subscribers = {};
            obj.k = 0;
            obj.controller_name = '';
            obj.initialized_reference_path = false;
            obj.got_stop = false;
            obj.success = false;
            obj.speedProfileMPAsInitialized = false;
            obj.cooldown_after_lane_change = 0;
            obj.cooldown_second_manual_vehicle_after_lane_change = 0;
            obj.info_old = [];
            obj.total_fallback_times = 0;
        end

        function [result,scenario] = run(obj)
            obj.init_hlc();
            obj.hlc_main_control_loop();
            obj.save_results();
            result = obj.result;
            scenario = obj.scenario;
        end

        % 1 vehicle ID implies distributed
        function set_vehicle_ids( obj, vehicle_ids )
            obj.vehicle_ids = vehicle_ids;
            obj.amount = length(vehicle_ids);
            if obj.amount == 1
                obj.indices_in_vehicle_list = [find([obj.scenario.vehicles.ID] == obj.vehicle_ids(1),1)];
            else
                obj.indices_in_vehicle_list = 1:obj.amount;
            end
        end

        function set_scenario( obj, scenario )
            obj.scenario = scenario;
        end

        function set_controller_name( obj, name )
            obj.controller_name = name;
        end

        function set_hlc_adapter( obj, interfaceExperiment)
            obj.hlc_adapter = interfaceExperiment;
        end
    end

    methods (Abstract = true, Access = protected)
        controller(obj);
    end

    methods (Access = private)

        function init_hlc( obj )

            % init result struct
            obj.result = get_result_struct(obj);

            % record the number of time steps that vehicles continually stop
            obj.vehs_stop_duration = zeros(obj.scenario.options.amount,1);

            %TODO Shouldn't this value be already set (to 0)?
            obj.iter.k = obj.k;

            % turn off warning if intersections are detected and fixed, collinear points or
            % overlapping points are removed when using MATLAB function `polyshape`
            warning('off','MATLAB:polyshape:repairedBySimplify')

            obj.iter = IterationData(obj.scenario,obj.k);

            obj.vehs_fallback_times = zeros(1,obj.scenario.options.amount);

            if obj.scenario.options.isPB
                % In priority-based computation, vehicles communicate via ROS 2
                % Create publishers and subscribers before experiment setup
                create_publishers(obj);
                create_subscribers(obj);
            end

            % init all manually controlled vehicles
            for hdv_id = obj.scenario.options.manual_control_config.hdv_ids
                obj.manual_vehicles = ManualVehicle(hdv_id, obj.scenario);
            end

            obj.hlc_adapter.setup(obj.scenario, obj.vehicle_ids);

            if obj.scenario.options.isPB
                % In priority-based computation, vehicles communicate via ROS 2
                % Initialize the communication network of ROS 2
                communication_init(obj);
            end

            if ~obj.scenario.options.isPB && obj.scenario.options.use_cpp
                % When using C++, you don't want to send the scenario over
                % and over again, so it is done in the init function
                optimizer(Function.InitializeWithScenario, obj.scenario);
            end
        end

        function clean_up(obj)
            if ~obj.success
                disp("Storing unfinished results up on error:")
                % Don't store the last time step with erroneous data.
                obj.k = obj.k - 1;
                % Save the unfinished results.
                obj.scenario.options.isSaveResult = true;
                obj.result.output_path = 'results/unfinished_result.mat';
                obj.save_results();
            end
        end

        function hlc_main_control_loop(obj)
            cleanup = onCleanup(@obj.clean_up);

            %% Main control loop
            while (~obj.got_stop)
                % TODO separate static scenario from dynamic
                % entries such as adjacency
                obj.result.step_timer = tic;

                % increment interation counter
                obj.k = obj.k+1;
                
                obj.iter.k = obj.k;

                % Measurement
                % -------------------------------------------------------------------------
                [x0_measured, trims_measured] = obj.hlc_adapter.measure();% trims_measured： which trim
                
                if mod(obj.k,10)==0
                    % only display 0, 10, 20, ...
                    disp(['>>> Time step ' num2str(obj.k)])
                end

                % Control
                % ----------------------------------------------------------------------

                % Update the iteration data and sample reference trajectory
                obj.rhc_init(x0_measured,trims_measured);
                obj.initialized_reference_path = true;

                % collision checking
                is_collision_occur = false;
                for iiVeh = 1:obj.scenario.options.amount-1
                    for jjVeh = iiVeh+1:obj.scenario.options.amount
                        if InterX(obj.iter.occupied_areas{iiVeh}.without_offset,obj.iter.occupied_areas{jjVeh}.without_offset)
                            warning(['Collision between vehicle ' num2str(iiVeh) ' and vehicle ' num2str(jjVeh) ' occur! Simulation ends.'])
                            is_collision_occur = true;
                            break
                        end
                    end
                    if is_collision_occur
                        break
                    end
                end

                if is_collision_occur
                    break
                end

                % calculate the distance
                distance = zeros(obj.scenario.options.amount,obj.scenario.options.amount);
                adjacency = obj.iter.adjacency;

                for jVeh = 1:obj.scenario.options.amount-1
                    adjacent_vehicle = find(adjacency(jVeh,:));
                    adjacent_vehicle = adjacent_vehicle(adjacent_vehicle > jVeh);
                    for vehn = adjacent_vehicle
                        distance(jVeh,vehn) = check_distance(obj.iter,jVeh,vehn);
                    end
                end
                obj.result.distance(:,:,obj.k) = distance;
                obj.result.iter_runtime(obj.k) = toc(obj.result.step_timer);

                % The controller computes plans
                controller_timer = tic;

                %% controller %%
                obj.controller();

                % If using distributed hlcs, collect fallback info from
                % other vehicles as required
                if obj.amount == 1
                    irrelevant_vehicles = union(obj.indices_in_vehicle_list(1), obj.info.vehs_fallback);
                    if obj.scenario.options.fallback_type == "localFallback"
                        sub_graph_fallback = obj.belonging_vector_total(obj.indices_in_vehicle_list(1));
                        other_vehicles = find(obj.belonging_vector_total==sub_graph_fallback);
                        % remove own vehicle. No need to read from own
                        % publisher
                        other_vehicles = setdiff(other_vehicles, irrelevant_vehicles,'stable');
                        for veh_id = other_vehicles
                            latest_msg = read_message(obj.scenario.vehicles(obj.indices_in_vehicle_list(1)).communicate.predictions, obj.ros_subscribers.predictions{veh_id}, obj.k);
                            fallback_info_veh_id = latest_msg.vehs_fallback';
                            obj.info.vehs_fallback = union(obj.info.vehs_fallback, fallback_info_veh_id);
                        end
                    else 
                        other_vehicles = setdiff(1:obj.scenario.options.amount, irrelevant_vehicles);
                        for veh_id = other_vehicles
                            latest_msg = read_message(obj.scenario.vehicles(obj.indices_in_vehicle_list(1)).communicate.predictions, obj.ros_subscribers.predictions{veh_id}, obj.k);
                            fallback_info_veh_id = latest_msg.vehs_fallback';
                            obj.info.vehs_fallback = union(obj.info.vehs_fallback, fallback_info_veh_id);
                        end
                    end
                end

                %% fallback
                if strcmp(obj.scenario.options.fallback_type,'noFallback')
                    % disabled fallback
                    if ~isempty(obj.info.vehs_fallback)
                        disp('Fallback is disabled. Simulation ends.')
                        obj.result.distance(:,:,obj.k) = [];
                        obj.result.iter_runtime(obj.k) = [];
                        break
                    end
                else
                    obj.vehs_fallback_times(obj.info.vehs_fallback) = obj.vehs_fallback_times(obj.info.vehs_fallback) + 1;
                    vehs_not_fallback = setdiff(1:obj.scenario.options.amount, obj.info.vehs_fallback);
                    obj.vehs_fallback_times(vehs_not_fallback) = 0; % reset

                    % check whether at least one vehicle has fallen back Hp times successively

                    if ~isempty(obj.info.vehs_fallback)
                        i_triggering_vehicles = find(obj.info.is_exhausted);

                        str_veh = sprintf('%d ', i_triggering_vehicles);
                        str_fb_type = sprintf('triggering %s', obj.scenario.options.fallback_type);
                        disp_tmp = sprintf(' %d,',obj.info.vehs_fallback); disp_tmp(end) = [];
                        disp(['Vehicle ', str_veh, str_fb_type, ', affecting vehicle' disp_tmp '.'])
                        obj.info = pb_controller_fallback(obj.iter, obj.info, obj.info_old, obj.scenario, obj.indices_in_vehicle_list);
                        obj.total_fallback_times = obj.total_fallback_times + 1;
                    end
                end

                obj.info_old = obj.info; % save variable in case of fallback
                %% save result of current time step
                obj.result.controller_runtime(obj.k) = toc(controller_timer);

                % save controller outputs in result struct
                obj.result.scenario = obj.scenario;
                obj.result.is_deadlock(obj.k) = 0;
                obj.result.iteration_structs{obj.k} = obj.iter;
                obj.result.trajectory_predictions(:,obj.k) = obj.info.y_predicted;
                obj.result.controller_outputs{obj.k} = obj.info.u;
                obj.result.subcontroller_runtime_each_veh(:,obj.k) = obj.info.runtime_subcontroller_each_veh;
                obj.result.graph_search_runtime_each_veh(:,obj.k) = obj.info.runtime_graph_search_each_veh;
                obj.result.vehicle_path_fullres(:,obj.k) = obj.info.vehicle_fullres_path(:);
                obj.result.n_expanded(:,obj.k) = obj.info.n_expanded;
                obj.result.priority_list(:,obj.k) = obj.iter.priority_list;
                obj.result.coupling_adjacency(:,:,obj.k) = obj.iter.adjacency;
                obj.result.computation_levels(obj.k) = obj.info.computation_levels;
                obj.result.step_time(obj.k) = toc(obj.result.step_timer);
                obj.result.obstacles = obj.iter.obstacles;
                % reset iter obstacles to scenario default/static obstacles
                obj.iter.obstacles = obj.scenario.obstacles;

                obj.result.runtime_subcontroller_max(obj.k) = obj.info.runtime_subcontroller_max;
                obj.result.runtime_graph_search_max(obj.k) = obj.info.runtime_graph_search_max;
                obj.result.directed_coupling{obj.k} = obj.iter.directed_coupling;
                if obj.scenario.options.isPB && strcmp(obj.scenario.options.scenario_name,'Commonroad')
                    obj.result.determine_couplings_time(obj.k) = obj.iter.timer.determine_couplings;
                    obj.result.group_vehs_time(obj.k) = obj.iter.timer.group_vehs;
                    obj.result.assign_priority_time(obj.k) = obj.iter.timer.assign_priority;
                    obj.result.num_couplings(obj.k) = nnz(obj.iter.directed_coupling);
                    obj.result.num_couplings_ignored(obj.k) = nnz(obj.iter.directed_coupling) - nnz(obj.iter.directed_coupling_reduced);
                    obj.result.num_couplings_between_grps(obj.k) = obj.iter.num_couplings_between_grps;
                    obj.result.num_couplings_between_grps_ignored(obj.k) = obj.iter.num_couplings_between_grps_ignored;
                    obj.result.belonging_vector(:,obj.k) = obj.iter.belonging_vector;
                    obj.result.coupling_weights_reduced{obj.k} = obj.iter.coupling_weights_reduced;
                    obj.result.coupling_info{obj.k} = obj.iter.coupling_info;
                    obj.result.coupling_weights_optimal{obj.k} = obj.iter.coupling_weights_optimal;
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
                vehs_stop = any(ismember(obj.info.trim_indices,obj.scenario.mpa.trims_stop),2); % vehicles stop at the current time step
                obj.vehs_stop_duration( vehs_stop) = obj.vehs_stop_duration(vehs_stop) + 1;
                obj.vehs_stop_duration(~vehs_stop) = 0; % reset others

                threshold_stop_steps = 3*obj.scenario.options.Hp;
                vehs_deadlocked = (obj.vehs_stop_duration>threshold_stop_steps);
                n_deadlocked_vehicles = nnz(vehs_deadlocked);
                % TODO n_coupled_deadlocked_vehicles
                if n_deadlocked_vehicles > 0
                    veh_idcs_deadlocked = find(vehs_deadlocked);
                    veh_str = sprintf("%4d",veh_idcs_deadlocked);
                    t_str = sprintf("%4d",obj.vehs_stop_duration(vehs_deadlocked));
                    fprintf("Deadlock. Vehicle:%s\n",veh_str);
                    fprintf("    For timesteps:%s\n",t_str)
                    obj.result.is_deadlock(obj.k) = 1;
                end

                % Apply control action
                % -------------------------------------------------------------------------
                obj.hlc_adapter.apply(obj.info, obj.result, obj.k, obj.scenario);

                % Check for stop signal
                % -------------------------------------------------------------------------
                obj.got_stop = obj.hlc_adapter.is_stop() || obj.got_stop;

            end
            obj.success = true;
        end

        function save_results(obj)
            %% save results at end of experiment
            obj.result.total_fallback_times = obj.total_fallback_times;
            disp(['Total times of fallback: ' num2str(obj.total_fallback_times) '.'])

            obj.result.t_total = obj.k*obj.scenario.options.dt;
            obj.result.nSteps = obj.k;

            disp(['Total runtime: ' num2str(round(obj.result.t_total,2)) ' seconds.'])

            if obj.scenario.options.isSaveResult
                empty_cells = cell(1,obj.scenario.options.amount);
                % delete ros nodes, because they can't be written to a
                % file.
                [obj.result.scenario.vehicles.communicate] = empty_cells{:};
                obj.result.mpa = obj.scenario.mpa;

                % Delete unimportant data
                if obj.scenario.options.isSaveResultReduced
                    for iIter = 1:length(obj.result.iteration_structs)
                        obj.result.iteration_structs{iIter}.predicted_lanelet_boundary = [];
                        obj.result.iteration_structs{iIter}.predicted_lanelets = [];
                        obj.result.iteration_structs{iIter}.reachable_sets = [];
                        obj.result.iteration_structs{iIter}.occupied_areas = [];
                        obj.result.iteration_structs{iIter}.emergency_maneuvers = [];
                    end
                    obj.result.scenario.mpa = [];
                    obj.result.scenario.speed_profile_mpas = [];
                end

                % check if file with the same name exists
                % while isfile(result.output_path)
                %     warning('File with the same name exists, timestamp will be added to the file name.')
                %     result.output_path = [result.output_path(1:end-4), '_', datestr(now,'yyyymmddTHHMMSS'), '.mat']; % move '.mat' to end
                % end

                result = obj.result;
                save(obj.result.output_path,'result');
                disp(['Simulation results were saved under ' obj.result.output_path])
            else
                disp('As required, simulation/Experiment Results were not saved.')
                % exportVideo( result );
            end
            % hacky way to destroy all ros nodes to avoid duplicates
            empty_cells = cell(1,obj.scenario.options.amount);
            [obj.result.scenario.vehicles.communicate] = empty_cells{:};
            [obj.scenario.vehicles.communicate] = empty_cells{:};
            obj.hlc_adapter.end_run()

        end
    end
end
