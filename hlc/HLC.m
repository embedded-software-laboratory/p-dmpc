classdef HLC < handle
    properties (Access=public)
        % For which vehicle IDs to start the HLC
        % Must be an array of vehicles. In distributed HLC case the array contains
        % exactly 1 integer
        vehicle_ids

        % scenario variable
        scenario

        % Wether to use the DistributedHlc (control 1 vehicle) or the
        % CentralHlc (controls all vehicles)
        % We can still run the DistributedHlc on a single machine!
        is_distributed logical

        % Adapter for the lab
        % or one for a local simulation
        hlc_adapter

        % MultiVehiclePlotter we can give to the main thread, so it can plot
        % in response to the workers
        plotter

        % Ros Subscribers for Inter HLC Communication (distributed HLCs) or
        % to simualate distributed communication in pb-sequential controller
        ros_subscribers

        % Wether to send data to plotter
        visualization

        % which controller should be used (e.g. centralized,
        % pb for a single vehicle, pb-sequential for all vehicles)
        controller

        sub_controller = @graph_search
        controller_name
        result

    end
    properties (Access=public)
        k
        initialized_reference_path
        got_stop;
        speedProfileMPAsInitialized;
        cooldown_after_lane_change;
        cooldown_second_manual_vehicle_after_lane_change;
        controller_init;
        iter;
        info;
    end

    methods
        % Set default settings
        function obj = HLC()
            % Some default values are invalid and thus they're easily spotted when they haven't been explicitly set
            % We can then either throw an exception or use an arbitrary option when we find a default value
            % Or should we make valid and useful default values?
            obj.vehicle_ids = [];
            obj.is_distributed = false;
            obj.ros_subscribers = {};
            obj.k = 0;
            obj.controller_name = 'RHGS';
            obj.initialized_reference_path = false;
            obj.got_stop = false;
            obj.speedProfileMPAsInitialized = false;
            obj.cooldown_after_lane_change = 0;
            obj.cooldown_second_manual_vehicle_after_lane_change = 0;
            obj.controller_init = false;

            if obj.visualization
                viz_data_queue = obj.plotter.getDataQueue();
            else
                viz_data_queue = false;
            end
        end

        % Optional argument wether to do a dry run of the first timestep beforehand
        % dry_run can massively decrease the time needed for the first
        % timestep during the experiment.
        function hlc = getHlc( obj, dry_run )

            if isempty(obj.scenario)
                throw(MException('HlcFactory:InvalidState', 'HlcScenario not set'));
            end

            obj.hlcSetup();

            % If the user doesn't specify otherwise, we do a dry run beforehand
            if nargin < 2
                dry_run = true;
            end

            if dry_run
                obj.dryRunHlc();
            end

        end

        function result = run(obj)
            obj.hlcMainLoop();
            result = obj.result;
        end

        function withDistributedPlanning( obj )
            obj.is_distributed = true;
        end

        function withCentralPlanning( obj )
            obj.is_distributed = false;
        end

        % A vehicle ID implies distributed, but we do not use this
        function setVehicleIds( obj, vehicle_ids )
            obj.vehicle_ids = vehicle_ids;
        end

        function setScenario( obj, scenario )
            obj.scenario = scenario;
        end

        function plotter = getPlotter( obj )
            obj.plotter = MultiVehiclePlotter( obj.scenario );
            plotter = obj.plotter;
        end
    end

    methods (Access=private)

        function setHlcAdapter( obj )
            if obj.scenario.options.is_sim_lab == false
                obj.hlc_adapter = CPMLab(obj.scenario, obj.vehicle_ids);
            else
                obj.hlc_adapter = SimLab(obj.scenario);
            end
            obj.hlc_adapter.setup();
        end

        function hlcSetup( obj )

            if obj.scenario.options.scenario_name == Scenario_Type.Commonroad
                if obj.scenario.options.isPB
                    if obj.scenario.options.isParl
                        % if parallel computation is used
                        obj.controller_name = strcat('par. PB-', obj.controller_name, ' ', char(obj.scenario.options.priority));
                        obj.controller = @pb_controller_parl;
                    else
                        obj.controller_name = strcat('seq. PB-', obj.controller_name, ' ', char(obj.scenario.options.priority));
                        obj.controller = @pb_controller_seq;
                    end
                else
                    obj.controller_name = strcat('centralized-', obj.controller_name, ' ', char(obj.scenario.options.priority));
                    obj.controller = @centralized_controller;
                end
            else
                if obj.scenario.options.isPB
                    obj.scenario.controller_name = strcat(obj.controller_name, '-PB');
                    obj.scenario.controller = @pb_controller;
                else
                    obj.scenario.controller_name = strcat(obj.controller_name, '-centralized');
                    obj.scenario.controller = @centralized_controller;
                end
            end

            obj.setHlcAdapter();

            obj.setVehicleIds(obj.scenario.options.veh_ids);

            % init result struct
            obj.result = get_result_struct(obj.scenario);

            %TODO Shouldn't this value be already 0?
            obj.scenario.k = obj.k;

            % turn off warning if intersections are detected and fixed, collinear points or
            % overlapping points are removed when using MATLAB function `polyshape`
            warning('off','MATLAB:polyshape:repairedBySimplify')

            if obj.scenario.options.isPB
                % In priority-based computation, vehicles communicate via ROS 2
                % Initialize the communication network of ROS 2
                [obj.scenario, obj.ros_subscribers] = communication_init(obj.scenario, obj.hlc_adapter);
            end

        end


        function hlcMainLoop(obj)

            vehs_fallback_times = zeros(1,obj.scenario.options.amount); % record the number of successive fallback times of each vehicle
            info_old = []; % old information for fallback
            total_fallback_times = 0; % total times of fallbacks

            vehs_stop_duration = zeros(obj.scenario.options.amount,1); % record the number of time steps that vehicles continually stop

            if obj.scenario.options.mixed_traffic_config.first_manual_vehicle_mode == Control_Mode.Expert_mode || obj.scenario.options.mixed_traffic_config.second_manual_vehicle_mode == Control_Mode.Expert_mode
                %r = rosrate(100000);
            end

            scenario_static = obj.scenario;

            %% Main control loop
            while (~obj.got_stop)
                % TODO separate static scenario from dynamic
                % entries such as adjacency
                obj.result.step_timer = tic;

                % increment interation counter
                obj.k = obj.k+1;

                % Measurement
                % -------------------------------------------------------------------------
                [x0_measured, trims_measured] = obj.hlc_adapter.measure(obj.controller_init);% trims_measured： which trim
                obj.controller_init = true;

                obj.scenario.k = obj.k;

                if mod(obj.k,10)==0
                    % only display 0, 10, 20, ...
                    disp(['>>> Time step ' num2str(obj.scenario.k)])
                end


                % Control
                % ----------------------------------------------------------------------

                % Update the iteration data and sample reference trajectory
                [obj.iter,obj.scenario] = rhc_init(obj.scenario,x0_measured,trims_measured, obj.initialized_reference_path, obj.ros_subscribers);
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


                % visualize reachabel set of vehicle in Expert-Mode
                for iVeh = 1:obj.scenario.options.amount
                    if obj.scenario.options.visualize_reachable_set && ((obj.scenario.vehicle_ids(iVeh) == obj.scenario.manual_vehicle_id && obj.scenario.options.firstManualVehicleMode == 2) ...
                            || (obj.scenario.vehicle_ids(iVeh) == obj.scenario.second_manual_vehicle_id && obj.scenario.options.secondManualVehicleMode == 2))
                        [visualization_command] = lab_visualize_polygon(obj.scenario, obj.iter.reachable_sets{iVeh, end}.Vertices, iVeh);
                        obj.hlc_adapter.visualize(visualization_command);
                    end
                end

                if obj.scenario.options.is_mixed_traffic
                    if obj.scenario.manual_vehicle_id ~= 0
                        if (obj.scenario.options.firstManualVehicleMode == 1)
                            wheelData = obj.hlc_adapter.getWheelData();
                            % function that translates current steering angle into lane change and velocity profile inputs into velocity changes
                            modeHandler = GuidedMode(obj.scenario,x0_measured,obj.scenario.manual_vehicle_id,obj.vehicle_ids,obj.cooldown_after_lane_change,obj.speedProfileMPAsInitialized,wheelData,true);
                            obj.scenario = modeHandler.scenario;
                            obj.scenario.updated_manual_vehicle_path = modeHandler.updatedPath;
                            obj.speedProfileMPAsInitialized = true;
                        end
                    end

                    if obj.scenario.second_manual_vehicle_id ~= 0
                        % function that updates the gamepad data
                        gamepadData = obj.hlc_adapter.getGamepadData();

                        if (obj.scenario.options.secondManualVehicleMode == 1)
                            % function that translates current steering angle into lane change and velocity profile inputs into velocity changes
                            modeHandler = GuidedMode(obj.scenario,x0_measured,obj.scenario.second_manual_vehicle_id,obj.vehicle_ids,obj.cooldown_second_manual_vehicle_after_lane_change,obj.speedProfileMPAsInitialized,gamepadData,false);
                            obj.scenario = modeHandler.scenario;
                            obj.scenario.updated_second_manual_vehicle_path = modeHandler.updatedPath;
                            obj.speedProfileMPAsInitialized = true;
                        end
                    end

                    if obj.scenario.updated_manual_vehicle_path
                        obj.cooldown_after_lane_change = 0;
                    else
                        obj.cooldown_after_lane_change = obj.cooldown_after_lane_change + 1;
                    end

                    if obj.scenario.updated_second_manual_vehicle_path
                        obj.cooldown_second_manual_vehicle_after_lane_change = 0;
                    else
                        obj.cooldown_second_manual_vehicle_after_lane_change = obj.cooldown_second_manual_vehicle_after_lane_change + 1;
                    end
                end

                % For parallel computation, information from previous time step is need, for example,
                % the previous fail-safe trajectory is used again if a new fail-safe trajectory cannot be found.

                if ~isempty(obj.scenario.lanelets)

                    if ( obj.scenario.options.amount > 1 )
                        % update the coupling information
                        obj.scenario = coupling_based_on_reachable_sets(obj.scenario, obj.iter);
                    end

                    % update the lanelet boundary for each vehicle
                    for iVeh = 1:obj.scenario.options.amount
                        obj.scenario.vehicles(iVeh).lanelet_boundary = obj.iter.predicted_lanelet_boundary(iVeh,1:2);
                    end
                else
                    % for other scenarios, no lanelet boundary
                    for iVeh = 1:obj.scenario.options.amount
                        obj.scenario.vehicles(iVeh).lanelet_boundary = {};
                    end

                end

                % calculate the distance
                distance = zeros(obj.scenario.options.amount,obj.scenario.options.amount);
                adjacency = obj.scenario.adjacency(:,:,end);

                for jVeh = 1:obj.scenario.options.amount-1
                    adjacent_vehicle = find(adjacency(jVeh,:));
                    adjacent_vehicle = adjacent_vehicle(adjacent_vehicle > jVeh);
                    for vehn = adjacent_vehicle
                        distance(jVeh,vehn) = check_distance(obj.iter,jVeh,vehn);
                    end
                end
                obj.result.distance(:,:,obj.k) = distance;

                % dynamic scenario
                obj.scenario = get_next_dynamic_obstacles_scenario(obj.scenario, scenario_static, obj.k);
                obj.result.iter_runtime(obj.k) = toc(obj.result.step_timer);

                % The controller computes plans
                controller_timer = tic;

                %% controller %%
                obj = obj.controller(obj);

                %% fallback
                if strcmp(obj.scenario.options.fallback_type,'noFallback')
                    % disable fallback
                    if any(obj.info.is_exhausted)
                        disp('Fallback is disabled. Simulation ends.')
                        obj.result.distance(:,:,obj.k) = [];
                        obj.result.iter_runtime(obj.k) = [];
                        break
                    end
                else
                    vehs_fallback_times(obj.info.vehs_fallback) = vehs_fallback_times(obj.info.vehs_fallback) + 1;
                    vehs_not_fallback = setdiff(1:obj.scenario.options.amount, obj.info.vehs_fallback);
                    vehs_fallback_times(vehs_not_fallback) = 0; % reset

                    % check whether at least one vehicle has fallen back Hp times successively

                    if ~isempty(obj.info.vehs_fallback)
                        i_triggering_vehicles = find(obj.info.is_exhausted);

                        str_veh = sprintf('%d ', i_triggering_vehicles);
                        str_fb_type = sprintf('triggering %s', obj.scenario.options.fallback_type);
                        disp_tmp = sprintf(' %d,',obj.info.vehs_fallback); disp_tmp(end) = [];
                        disp(['Vehicle ', str_veh, str_fb_type, ', affecting vehicle' disp_tmp '.'])
                        obj.info = pb_controller_fallback(obj.info, info_old, obj.scenario);
                        total_fallback_times = total_fallback_times + 1;
                    end
                end

                info_old = obj.info; % save variable in case of fallback
                %% save result of current time step
                obj.result.controller_runtime(obj.k) = toc(controller_timer);

                % save controller outputs in result struct
                obj.result.scenario = obj.scenario;
                obj.result.is_deadlock(obj.k) = 0;
                obj.result.iteration_structs{obj.k} = obj.iter;
                obj.result.trajectory_predictions(:,obj.k) = obj.info.y_predicted;
                obj.result.controller_outputs{obj.k} = obj.info.u;
                obj.result.subcontroller_runtime_each_veh(:,obj.k) = obj.info.runtime_subcontroller_each_veh;
                obj.result.vehicle_path_fullres(:,obj.k) = obj.info.vehicle_fullres_path(:);
                obj.result.n_expanded(obj.k) = obj.info.n_expanded;
                obj.result.priority(:,obj.k) = obj.scenario.priority_list;
                obj.result.computation_levels(obj.k) = obj.info.computation_levels;
                obj.result.step_time(obj.k) = toc(obj.result.step_timer);

                obj.result.runtime_subcontroller_max(obj.k) = obj.info.runtime_subcontroller_max;
                obj.result.runtime_graph_search_max(obj.k) = obj.info.runtime_graph_search_max;
                obj.result.directed_coupling{obj.k} = obj.scenario.directed_coupling;
                if obj.scenario.options.isParl && strcmp(obj.scenario.options.scenario_name,'Commonroad')
                    obj.result.determine_couplings_time(obj.k) = obj.scenario.timer.determine_couplings;
                    obj.result.group_vehs_time(obj.k) = obj.scenario.timer.group_vehs;
                    obj.result.assign_priority_time(obj.k) = obj.scenario.timer.assign_priority;
                    obj.result.num_couplings(obj.k) = nnz(obj.scenario.directed_coupling);
                    obj.result.num_couplings_ignored(obj.k) = nnz(obj.scenario.directed_coupling) - nnz(obj.scenario.directed_coupling_reduced);
                    obj.result.num_couplings_between_grps(obj.k) = obj.scenario.num_couplings_between_grps;
                    obj.result.num_couplings_between_grps_ignored(obj.k) = obj.scenario.num_couplings_between_grps_ignored;
                    obj.result.belonging_vector(:,obj.k) = obj.scenario.belonging_vector;
                    obj.result.coupling_weights_reduced{obj.k} = obj.scenario.coupling_weights_reduced;
                    obj.result.coupling_info{obj.k} = obj.scenario.coupling_info;
                    obj.result.coupling_weights_optimal{obj.k} = obj.scenario.coupling_weights_optimal;
                    obj.result.parl_groups_info{obj.k} = obj.scenario.parl_groups_info;
                    obj.result.lanelet_crossing_areas{obj.k} = obj.scenario.lanelet_crossing_areas;
                    obj.scenario.lanelet_crossing_areas = {};
                end
                obj.result.vehs_fallback{obj.k} = obj.info.vehs_fallback;

                % check if deadlock occurs
                % if a vehicle stops for more than a defined time, assume deadlock
                % TODO check if deadlocked vehicles are coupled. Sometimes single
                % vehicles stop because trajectory planner fails to work as intended
                vehs_stop = any(ismember(obj.info.trim_indices,obj.scenario.mpa.trims_stop),2); % vehicles stop at the current time step
                vehs_stop_duration( vehs_stop) = vehs_stop_duration(vehs_stop) + 1;
                vehs_stop_duration(~vehs_stop) = 0; % reset others

                threshold_stop_steps = 3*obj.scenario.options.Hp;
                vehs_deadlocked = (vehs_stop_duration>threshold_stop_steps);
                n_deadlocked_vehicles = nnz(vehs_deadlocked);
                % TODO n_coupled_deadlocked_vehicles
                if n_deadlocked_vehicles > 0
                    veh_idcs_deadlocked = find(vehs_deadlocked);
                    veh_str = sprintf("%4d",veh_idcs_deadlocked);
                    t_str = sprintf("%4d",vehs_stop_duration(vehs_deadlocked));
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

            %% save results
            obj.result.total_fallback_times = total_fallback_times;
            disp(['Total times of fallback: ' num2str(total_fallback_times) '.'])

            obj.result.t_total = obj.k*obj.scenario.options.dt;
            obj.result.nSteps = obj.k;

            disp(['Total runtime: ' num2str(round(obj.result.t_total,2)) ' seconds.'])

            if obj.scenario.options.isSaveResult
                % Delete varibales used for ROS 2 since some of them cannot be saved
                % Create comma-separated list
                empty_cells = cell(1,obj.scenario.options.amount);

                obj.result.scenario.ros_subscribers = [];
                [obj.result.scenario.vehicles.communicate] = empty_cells{:};
                % for i_iter = 1:length(result.iteration_structs)
                %     result.iteration_structs{i_iter}.scenario = [];
                % end

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

                save(obj.result.output_path,'result');
                disp(['Simulation results were saved under ' obj.result.output_path])
            else
                disp('As required, simulation/Experiment Results were not saved.')
                % exportVideo( result );
            end
            % hacky way to destroy all ros nodes to avoid duplicates
            empty_cells = cell(1,obj.scenario.options.amount);
            obj.result.scenario.ros_subscribers = {};
            [obj.result.scenario.vehicles.communicate] = empty_cells{:};
            obj.scenario.ros_subscribers = {};
            [obj.scenario.vehicles.communicate] = empty_cells{:};
            obj.hlc_adapter.end_run()
        end

        % This function runs the HLC once without outputting anything and
        % resets it afterwards.
        % Usually MATLAB takes some time to run code for the first time because
        % it has to compile it while running. If we run if before the
        % experiment starts, we save a few hundred milliseconds on the first
        % actual timestep of the experiment.
        % This so far only works when the HLC doesn't require input arguments
        % It also assumes that if the HLC is distributed, all other HLCs start
        % at approximately the same time, because otherwise onEachTimestep
        % won't run through.
        %
        % Important note: This might take some time depending on how hard to
        % solve the first timestep of this scenario is.
        function dryRunHlc(obj)
            disp("Starting dry run of HLC - TODO Implement");

            % Reset visualization + result + iter etc. to its initial value
            % TODO implement
        end
    end

end
