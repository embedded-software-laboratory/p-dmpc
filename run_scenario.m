function [result,scenario] = run_scenario(scenario)
    %% Setup
    % Initialize
    got_stop = false;
    k = 0;
    initialized_reference_path = false;
    speedProfileMPAsInitialized = false;
    cooldown_after_lane_change = 0;
    cooldown_second_manual_vehicle_after_lane_change = 0;
    controller_init = false;

% init result struct
result = get_result_struct(scenario);

    

% turn off warning if intersections are detected and fixed, collinear points or
% overlapping points are removed when using MATLAB function `polyshape`
warning('off','MATLAB:polyshape:repairedBySimplify')

iter = IterationData(scenario,k);
if scenario.options.isPB
    % In priority-based computation, vehicles communicate via ROS 2
    % Initialize the communication network of ROS 2
    scenario = communication_init(scenario, iter, exp);
end

vehs_fallback_times = zeros(1,scenario.options.amount); % record the number of successive fallback times of each vehicle 
info_old = []; % old information for fallback
total_fallback_times = 0; % total times of fallbacks

vehs_stop_duration = zeros(scenario.options.amount,1); % record the number of time steps that vehicles continually stop

if scenario.options.is_mixed_traffic && ...
   (scenario.options.mixed_traffic_config.first_manual_vehicle_mode == Control_Mode.Expert_mode || scenario.options.mixed_traffic_config.second_manual_vehicle_mode == Control_Mode.Expert_mode)
    %r = rosrate(100000);
end

%% Main control loop
while (~got_stop)
    % TODO separate static scenario from dynamic
    % entries such as adjacency
    result.step_timer = tic;
    
    % increment interation counter
    k = k+1;
    
    % Measurement
    % -------------------------------------------------------------------------
    [x0_measured, trims_measured] = exp.measure(controller_init);% trims_measured： which trim  
    controller_init = true;

    if mod(k,10)==0
        % only display 0, 10, 20, ...
        disp(['>>> Time step ' num2str(k)])
    end


    % Control
    % ----------------------------------------------------------------------
     
    % Update the iteration data and sample reference trajectory
    [ iter ] = rhc_init(iter, scenario, x0_measured, trims_measured, initialized_reference_path, k);
    initialized_reference_path = true;

    % collision checking 
    is_collision_occur = false;
    for iiVeh = 1:scenario.options.amount-1
        for jjVeh = iiVeh+1:scenario.options.amount
            if InterX(iter.occupied_areas{iiVeh}.without_offset,iter.occupied_areas{jjVeh}.without_offset)
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
    for iVeh = 1:scenario.options.amount 
        if scenario.options.visualize_reachable_set && ((scenario.options.veh_ids(iVeh) == scenario.manual_vehicle_id && scenario.options.firstManualVehicleMode == 2) ...
            || (scenario.options.veh_ids(iVeh) == scenario.second_manual_vehicle_id && scenario.options.secondManualVehicleMode == 2))
            [visualization_command] = lab_visualize_polygon(scenario, iter.reachable_sets{iVeh, end}.Vertices, iVeh);
            exp.visualize(visualization_command);
        end
    end

    if scenario.options.is_mixed_traffic
        if scenario.manual_vehicle_id ~= 0
            if (scenario.options.firstManualVehicleMode == 1)
                wheelData = exp.getWheelData();
                % function that translates current steering angle into lane change and velocity profile inputs into velocity changes
                modeHandler = GuidedMode(scenario,x0_measured,scenario.manual_vehicle_id,vehicle_ids,cooldown_after_lane_change,speedProfileMPAsInitialized,wheelData,true);
                scenario = modeHandler.scenario; % TODO_DATA: Scenario changes here
                scenario.updated_manual_vehicle_path = modeHandler.updatedPath;
                speedProfileMPAsInitialized = true;
            end
        end

        if scenario.second_manual_vehicle_id ~= 0
            % function that updates the gamepad data
            gamepadData = exp.getGamepadData();

            if (scenario.options.secondManualVehicleMode == 1)
                % function that translates current steering angle into lane change and velocity profile inputs into velocity changes
                modeHandler = GuidedMode(scenario,x0_measured,scenario.second_manual_vehicle_id,vehicle_ids,cooldown_second_manual_vehicle_after_lane_change,speedProfileMPAsInitialized,gamepadData,false);
                scenario = modeHandler.scenario; % TODO_DATA: Scenario changes here
                scenario.updated_second_manual_vehicle_path = modeHandler.updatedPath;
                speedProfileMPAsInitialized = true;
            end
        end

        if scenario.updated_manual_vehicle_path
            cooldown_after_lane_change = 0;
        else
            cooldown_after_lane_change = cooldown_after_lane_change + 1;
        end
    
        if scenario.updated_second_manual_vehicle_path
            cooldown_second_manual_vehicle_after_lane_change = 0;
        else
            cooldown_second_manual_vehicle_after_lane_change = cooldown_second_manual_vehicle_after_lane_change + 1;
        end
    end

    if ( scenario.options.amount > 1 )
        % update the coupling information
        iter = coupling_based_on_reachable_sets(scenario, iter); % TODO_DATA: Scenario changes here
    end
    
    % calculate the distance
    distance = zeros(scenario.options.amount,scenario.options.amount);
    adjacency = iter.adjacency;

    for jVeh = 1:scenario.options.amount-1
        adjacent_vehicle = find(adjacency(jVeh,:));
        adjacent_vehicle = adjacent_vehicle(adjacent_vehicle > jVeh);
        for vehn = adjacent_vehicle
            distance(jVeh,vehn) = check_distance(iter,jVeh,vehn);
        end
    end
    result.distance(:,:,k) = distance;
    
    % dynamic scenario
    result.iter_runtime(k) = toc(result.step_timer);

    % The controller computes plans
    controller_timer = tic; 
    %% controller %%
    [info, scenario, iter] = scenario.controller(scenario, iter); % TODO_DATA: does scenario change here?

    %% fallback
    if strcmp(scenario.options.fallback_type,'noFallback')
        % disable fallback
        if any(info.is_exhausted)
            disp('Fallback is disabled. Simulation ends.')
            result.distance(:,:,k) = [];
            result.iter_runtime(k) = [];
            break
        end
    else
        vehs_fallback_times(info.vehs_fallback) = vehs_fallback_times(info.vehs_fallback) + 1;
        vehs_not_fallback = setdiff(1:scenario.options.amount, info.vehs_fallback);
        vehs_fallback_times(vehs_not_fallback) = 0; % reset
        
        % check whether at least one vehicle has fallen back Hp times successively
        
        % The controller computes plans
        controller_timer = tic;

        %% controller %%
        [info, scenario] = scenario.controller(scenario, iter);

            str_veh = sprintf('%d ', i_triggering_vehicles);
            str_fb_type = sprintf('triggering %s', scenario.options.fallback_type);
            disp_tmp = sprintf(' %d,',info.vehs_fallback); disp_tmp(end) = [];
            disp(['Vehicle ', str_veh, str_fb_type, ', affecting vehicle' disp_tmp '.'])
            info = pb_controller_fallback(iter, info, info_old, scenario);
            total_fallback_times = total_fallback_times + 1;
        end
    end

        info_old = info; % save variable in case of fallback
        %% save result of current time step
        result.controller_runtime(k) = toc(controller_timer);
        
        % save controller outputs in result struct
        result.scenario = scenario;
        result.is_deadlock(k) = 0;
        result.iteration_structs{k} = iter;
        result.trajectory_predictions(:,k) = info.y_predicted;
        result.controller_outputs{k} = info.u;
        result.subcontroller_runtime_each_veh(:,k) = info.runtime_subcontroller_each_veh;
        result.vehicle_path_fullres(:,k) = info.vehicle_fullres_path(:);
        result.n_expanded(k) = info.n_expanded;
        result.priority(:,k) = scenario.priority_list;
        result.computation_levels(k) = info.computation_levels;
        result.step_time(k) = toc(result.step_timer);

    result.runtime_subcontroller_max(k) = info.runtime_subcontroller_max;
    result.runtime_graph_search_max(k) = info.runtime_graph_search_max;
    result.directed_coupling{k} = iter.directed_coupling;
    if scenario.options.isParl && strcmp(scenario.options.scenario_name,'Commonroad')
        result.determine_couplings_time(k) = iter.timer.determine_couplings;
        result.group_vehs_time(k) = iter.timer.group_vehs;
        result.assign_priority_time(k) = iter.timer.assign_priority;
        result.num_couplings(k) = nnz(iter.directed_coupling);
        result.num_couplings_ignored(k) = nnz(iter.directed_coupling) - nnz(iter.directed_coupling_reduced);
        result.num_couplings_between_grps(k) = iter.num_couplings_between_grps;
        result.num_couplings_between_grps_ignored(k) = iter.num_couplings_between_grps_ignored;
        result.belonging_vector(:,k) = iter.belonging_vector;
        result.coupling_weights_reduced{k} = iter.coupling_weights_reduced;
        result.coupling_info{k} = iter.coupling_info;
        result.coupling_weights_optimal{k} = iter.coupling_weights_optimal;
        result.parl_groups_info{k} = iter.parl_groups_info;
        result.lanelet_crossing_areas{k} = iter.lanelet_crossing_areas;
        iter.lanelet_crossing_areas = {};
    end
    result.vehs_fallback{k} = info.vehs_fallback;

    % check if deadlock occurs
    % if a vehicle stops for more than a defined time, assume deadlock
    % TODO check if deadlocked vehicles are coupled. Sometimes single
    % vehicles stop because trajectory planner fails to work as intended
    vehs_stop = any(ismember(info.trim_indices,scenario.mpa.trims_stop),2); % vehicles stop at the current time step
    vehs_stop_duration( vehs_stop) = vehs_stop_duration(vehs_stop) + 1;
    vehs_stop_duration(~vehs_stop) = 0; % reset others
    
    threshold_stop_steps = 3*scenario.options.Hp;
    vehs_deadlocked = (vehs_stop_duration>threshold_stop_steps);
    n_deadlocked_vehicles = nnz(vehs_deadlocked);
    % TODO n_coupled_deadlocked_vehicles
    if n_deadlocked_vehicles > 0
        veh_idcs_deadlocked = find(vehs_deadlocked);
        veh_str = sprintf("%4d",veh_idcs_deadlocked);            
        t_str = sprintf("%4d",vehs_stop_duration(vehs_deadlocked)); 
        fprintf("Deadlock. Vehicle:%s\n",veh_str);
        fprintf("    For timesteps:%s\n",t_str)
        result.is_deadlock(k) = 1;
    end

    %% save results
    result.total_fallback_times = total_fallback_times;
    disp(['Total times of fallback: ' num2str(total_fallback_times) '.'])

result.t_total = k*scenario.options.dt;
result.nSteps = k;

disp(['Total runtime: ' num2str(round(result.t_total,2)) ' seconds.'])

    if scenario.options.isSaveResult
        % Delete varibales used for ROS 2 since some of them cannot be saved
        % Create comma-separated list
        empty_cells = cell(1,scenario.options.amount);
        
        result.scenario.ros_subscribers = [];
        [result.scenario.vehicles.communicate] = empty_cells{:};
        % for i_iter = 1:length(result.iteration_structs)
        %     result.iteration_structs{i_iter}.scenario = [];
        % end
        
        result.mpa = scenario.mpa;

    % Delete unimportant data
    if scenario.options.isSaveResultReduced
        for iIter = 1:length(result.iteration_structs)
            result.iteration_structs{iIter}.predicted_lanelet_boundary = [];
            result.iteration_structs{iIter}.predicted_lanelets = [];
            result.iteration_structs{iIter}.reachable_sets = [];
            result.iteration_structs{iIter}.occupied_areas = [];
            result.iteration_structs{iIter}.emergency_maneuvers = [];
        end
        result.scenario.mpa = [];
        result.scenario.speed_profile_mpas = [];
    end

    % check if file with the same name exists
    % while isfile(result.output_path)
    %     warning('File with the same name exists, timestamp will be added to the file name.')
    %     result.output_path = [result.output_path(1:end-4), '_', datestr(now,'yyyymmddTHHMMSS'), '.mat']; % move '.mat' to end
    % end

    save(result.output_path,'result');
    disp(['Simulation results were saved under ' result.output_path])
else
    disp('As required, simulation/Experiment Results were not saved.')
% exportVideo( result );
end
% hacky way to destroy all ros nodes to avoid duplicates
empty_cells = cell(1,scenario.options.amount);
result.scenario.ros_subscribers = {};
[result.scenario.vehicles.communicate] = empty_cells{:};
scenario.ros_subscribers = {};
[scenario.vehicles.communicate] = empty_cells{:};
exp.end_run()

end