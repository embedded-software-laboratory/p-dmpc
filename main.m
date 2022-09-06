function [result,scenario,options] = main(varargin)
% MAIN  main function for graph-based receeding horizon control

if verLessThan('matlab','9.10')
    warning("Code is developed in MATLAB 2021a, prepare for backward incompatibilities.")
end

random_seed = RandStream('mt19937ar'); % for reproducibility

% check if options are given as input
find_options = cellfun(@(c) isa(c,'OptionsMain'), varargin);
    
if any(find_options)
    options = varargin{find_options};
else
    options = startOptions();
end
% options.fallback_type = 'globalFallback';
% options.fallback_type = 'localFallback';
% 
%[options, vehicle_ids] = eval_guided_mode(1);
%[options, vehicle_ids] = eval_expert_mode(1);
options.is_eval = false;
options.visualize_reachable_set = false;
is_sim_lab = options.is_sim_lab;

%% Determine options
% if matlab simulation should be started with certain parameters
% first argument has to be 'sim'
%is_sim_lab = (nargin == 0 || (nargin > 0 && strcmp(varargin{1},'sim')));

if is_sim_lab
    if ~any(find_options)
        switch nargin
            case 6
                options = selection(varargin{2},varargin{3},varargin{4},varargin{5},varargin{6});
            case 5
                options = selection(varargin{2},varargin{3},varargin{4},varargin{5},1);
            case 4
                options = selection(varargin{2},varargin{3},varargin{4},1,1);            
            case 3
                options = selection(varargin{2},varargin{3},1,1,1); 
            case 2 
                options = selection(varargin{2},2,1,1,1);
            otherwise
                % former UI for Sim lab
                %options = selection();
        end
    end

    if isempty(options.veh_ids)
        switch options.amount
            % specify vehicles IDs
            case 2
                vehicle_ids = [16,18];
            case 4
                vehicle_ids = [14,16,18,20];
    %         case 6
    %             vehicle_ids = [10,14,16,17,18,20]; 
            otherwise
%                 vehicle_ids = 1:options.amount; % default IDs
                if options.max_num_CLs == 1
                    % if allowed computation is only 1, the first 8
                    % vehicles will not be used to avoid infeasibility at
                    % the first time step as there may be vehicles being
                    % very colse to others
                    vehicle_ids = sort(randsample(random_seed,9:40,options.amount),'ascend');
                else
                    vehicle_ids = sort(randsample(random_seed,1:40,options.amount),'ascend');
                end
                options.veh_ids = vehicle_ids;
                
        end
    else
        vehicle_ids = options.veh_ids;
    end

    manualVehicle_id = 0;
    manualVehicle_id2 = 0;
    options.firstManualVehicleMode = 0;
    options.secondManualVehicleMode = 0;
    options.collisionAvoidanceMode = 0;
    options.is_mixed_traffic = 0;
    options.force_feedback_enabled = 0;

else
    disp('cpmlab')
    if ~options.is_eval
        vehicle_ids = [varargin{:}];
    end
    options.amount = numel(vehicle_ids);
    options.isPB = true;
    manualVehicle_id = 0;
    manualVehicle_id2 = 0;

    if options.is_mixed_traffic

        % former UI for CPM Lab
        %mixedTrafficOptions = mixedTrafficSelection();
        options.isParl = false;
        manualVehicle_id = options.manualVehicle_id;

        if ~strcmp(manualVehicle_id, 'No MV')
            manualVehicle_id = str2num(options.manualVehicle_id);
            options.firstManualVehicleMode = str2num(options.firstManualVehicleMode);

            if ~strcmp(options.manualVehicle_id2, 'No second MV')
                manualVehicle_id2 = str2num(options.manualVehicle_id2);
                options.secondManualVehicleMode = str2num(options.secondManualVehicleMode);
            end
        else
            manualVehicle_id = 0;
        end

        if options.collisionAvoidanceMode == 1
            options.isParl = false;
            options.priority = 'right_of_way_priority';
        elseif options.collisionAvoidanceMode == 2 
            options.isParl = true;
            options.priority = 'right_of_way_priority';
        else
            options.isParl = true;
            options.priority = 'mixed_traffic_priority';
            options.visualize_reachable_set = true;
        end
    else
        options.firstManualVehicleMode = 0;
        options.secondManualVehicleMode = 0;
        options.collisionAvoidanceMode = 0;
    end
end

% scenario = circle_scenario(options.amount,options.isPB);
% scenario = lanelet_scenario4(options.isPB,options.isParl,isROS);
 
switch options.scenario_name
    case 'Circle_scenario'
        scenario = circle_scenario(options);
    case 'Commonroad'
        scenario = commonroad(options, vehicle_ids, manualVehicle_id, manualVehicle_id2, is_sim_lab);  
end
scenario.random_seed = random_seed;
scenario.name = options.scenario_name;
scenario.manual_vehicle_id = manualVehicle_id;
scenario.second_manual_vehicle_id = manualVehicle_id2;
scenario.vehicle_ids = vehicle_ids;
scenario.mixedTrafficCollisionAvoidanceMode = options.collisionAvoidanceMode;
% scenario.options = options;

for iVeh = 1:scenario.options.amount
    % initialize vehicle ids of all vehicles
    scenario.vehicles(iVeh).ID = scenario.vehicle_ids(iVeh);
end

if is_sim_lab
    exp = SimLab(scenario);
else
    exp = CPMLab(scenario, vehicle_ids);
end

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

exp.setup();

scenario.k = k;

% turn off warning if intersections are detected and fixed, collinear points or
% overlapping points are removed when using MATLAB function `polyshape`
warning('off','MATLAB:polyshape:repairedBySimplify')

% check if scenario iss given as input
find_scenario = cellfun(@(c) isa(c,'Scenario'), varargin);
if any(find_scenario)
    if length(varargin{find_scenario}.ros_subscribers) >= scenario.options.amount
        % if ROS 2 subscribers and publishers are given as input, store them
        for iiVeh = 1:scenario.options.amount
            scenario.vehicles(iiVeh).communicate = varargin{find_scenario}.vehicles(iiVeh).communicate;
        end
        scenario.ros_subscribers = varargin{find_scenario}.ros_subscribers(1:scenario.options.amount);
    end
end

if options.isParl && strcmp(scenario.name, 'Commonroad')
    % In parallel computation, vehicles communicate via ROS 2
    % Initialize the communication network of ROS 2
    scenario = communication_init(scenario, exp);
end

vehs_fallback_times = zeros(1,scenario.options.amount); % record the number of successive fallback times of each vehicle 
info_old = []; % old information for fallback
total_fallback_times = 0; % total times of fallbacks

threshold_stop_steps = 20; % if a vehicle steps more than this number of time steps, a deadlock is considered to have occur
vehs_stop_time_steps = inf(options.amount,1); % record the number of time steps that vehicles continually stop
is_deadlock = false;

if scenario.options.firstManualVehicleMode == 2 || scenario.options.secondManualVehicleMode == 2
    %r = rosrate(100000);
end

%% Main control loop
while (~got_stop)

    result.step_timer = tic;
    
    % increment interation counter
    k = k+1;
    
    % Measurement
    % -------------------------------------------------------------------------
    [x0_measured, trims_measured] = exp.measure(controller_init);% trims_measured： which trim  
    controller_init = true;

    scenario.k = k;

    if mod(k,10)==0
        % only display 0, 10, 20, ...
        disp(['>>> Time step ' num2str(scenario.k)])
    end


    % Control
    % ----------------------------------------------------------------------
     
    % Update the iteration data and sample reference trajectory
    [iter,scenario] = rhc_init(scenario,x0_measured,trims_measured, initialized_reference_path, is_sim_lab);
    initialized_reference_path = true;

    % collision checking 
    is_collision_occur = false;
    for iiVeh = 1:options.amount-1
        for jjVeh = iiVeh+1:options.amount
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
        if options.visualize_reachable_set && ((scenario.vehicle_ids(iVeh) == scenario.manual_vehicle_id && scenario.options.firstManualVehicleMode == 2) ...
            || (scenario.vehicle_ids(iVeh) == scenario.second_manual_vehicle_id && scenario.options.secondManualVehicleMode == 2))
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
                scenario = modeHandler.scenario;
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
                scenario = modeHandler.scenario;
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
    
    % For parallel computation, information from previous time step is need, for example, 
    % the previous fail-safe trajectory is used again if a new fail-safe trajectory cannot be found.
    
    % update the coupling information
    if strcmp(scenario.name, 'Commonroad')
        if ~options.isParl || scenario.mixedTrafficCollisionAvoidanceMode == 2
            % update the coupling information
            scenario = coupling_adjacency(scenario, iter);
        end

        % update the lanelet boundary for each vehicle
        for iVeh = 1:options.amount
            scenario.vehicles(iVeh).lanelet_boundary = iter.predicted_lanelet_boundary(iVeh,1:2);
        end
    else
        % for other scenarios, no lanelet boundary 
        for iVeh = 1:options.amount
            scenario.vehicles(iVeh).lanelet_boundary = {};
        end

    end
    
    % calculate the distance
    distance = zeros(options.amount,options.amount);
    adjacency = scenario.adjacency(:,:,end);

    for jVeh = 1:options.amount-1
        adjacent_vehicle = find(adjacency(jVeh,:));
        adjacent_vehicle = adjacent_vehicle(adjacent_vehicle > jVeh);
        for vehn = adjacent_vehicle
            distance(jVeh,vehn) = check_distance(iter,jVeh,vehn);
        end
    end
    result.distance(:,:,k) = distance;
    
    % dynamic scenario
    scenario_tmp = get_next_dynamic_obstacles_scenario(scenario, k);
    result.iter_runtime(k) = toc(result.step_timer);
    
    % The controller computes plans
    controller_timer = tic; 
    %% controller %%
    [info, scenario] = scenario.controller(scenario_tmp, iter);

    %% fallback
    if strcmp(options.fallback_type,'noFallback')
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
        
            if ~isempty(info.vehs_fallback)
                real_vehicles = zeros(1,length(info.vehs_fallback));
    
                for i=1:length(info.vehs_fallback)
                    real_vehicles(i) = scenario.vehicle_ids(info.vehs_fallback(i));
                end
    
                disp_tmp = sprintf('%d,',real_vehicles); disp_tmp(end) = [];
                % disp(['*** Vehicles ' disp_tmp ' take fallback.']) % use * to highlight this message
                info = pb_controller_fallback(info, info_old, scenario);
                total_fallback_times = total_fallback_times + 1;
            end
    
        if max(vehs_fallback_times) >= scenario.options.Hp
            disp('Already fall back successively Hp times, terminate the simulation')
%             break % break the while loop
        end
    end

    info_old = info; % save variable in case of fallback
    %% save result
    result.controller_runtime(k) = toc(controller_timer);
    
    % save controller outputs in result struct
    result.scenario = scenario;
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
    result.directed_coupling{k} = scenario.directed_coupling;
    if options.isParl && strcmp(scenario.options.scenario_name,'Commonroad')
        result.determine_couplings_time(k) = scenario.timer.determine_couplings;
        result.group_vehs_time(k) = scenario.timer.group_vehs;
        result.assign_priority_time(k) = scenario.timer.assign_priority;
        result.num_couplings(k) = nnz(scenario.directed_coupling);
        result.num_couplings_ignored(k) = nnz(scenario.directed_coupling) - nnz(scenario.directed_coupling_reduced);
        result.num_couplings_between_grps(k) = scenario.num_couplings_between_grps;
        result.num_couplings_between_grps_ignored(k) = scenario.num_couplings_between_grps_ignored;
        result.belonging_vector(:,k) = scenario.belonging_vector;
        result.coupling_weights_reduced{k} = scenario.coupling_weights_reduced;
        result.coupling_info{k} = scenario.coupling_info;
        result.coupling_weights_optimal{k} = scenario.coupling_weights_optimal;
        result.parl_groups_info{k} = scenario.parl_groups_info;
        result.lanelet_crossing_areas{k} = scenario.lanelet_crossing_areas;
        scenario.lanelet_crossing_areas = {};
    end
    result.vehs_fallback{k} = info.vehs_fallback;

    % check if deadlock occurs
    vehs_stop = any(ismember(info.trim_indices,scenario.mpa.trims_stop),2); % vehicles stop at the current time step
    vehs_stop_time_steps(~vehs_stop) = inf; % reset others

    vehs_stop_successively = vehs_stop_time_steps<k & vehs_stop; % vehicles stop both at the current and previous time step
    vehs_stop_newly = ~vehs_stop_successively & vehs_stop;
    vehs_stop_time_steps(vehs_stop_newly) = k;
    
    % if more than one third of vehicles stop for more than a defined time, deadlock is considered to have occur
    [min_stop_step,veh_deadlock] = mink(vehs_stop_time_steps,ceil(options.amount/3)); 
    max_stop_steps = k - min_stop_step(end);
    if max_stop_steps > threshold_stop_steps
        % a deadlock is considered to have occur if a vehicle stops continually more than a certain number time steps
        warning(['Deadlock occurs since vehicle ' num2str(veh_deadlock(1)) ' stops for a long time.'])
        result.t_total = min_stop_step(1)*scenario.options.dt;
        result.nSteps = result.t_total;
        is_deadlock = true;

        % delete all data collected after deadlock
        fieldnames_r = fieldnames(result);
        nColumns = size(result.iteration_structs,2);
        for iField = 1:length(fieldnames_r)
            filedName = fieldnames_r{iField};
            if size(result.(filedName),2) == nColumns
                % delete
                result.(filedName)(:,min_stop_step(1):end) = [];
            end
        end

        break
    end
    
    % Apply control action
    % -------------------------------------------------------------------------
    exp.apply(info, result, k, scenario); 
    
    % Check for stop signal
    % -------------------------------------------------------------------------
    got_stop = exp.is_stop() || got_stop;
    
end
result.is_deadlock = is_deadlock;
result.total_fallback_times = total_fallback_times;
disp(['Total times of fallback: ' num2str(total_fallback_times) '.'])
if ~is_deadlock
    result.t_total = k*scenario.options.dt;
    result.nSteps = k;
end
disp(['Total runtime: ' num2str(round(result.t_total,2)) ' seconds.'])

%% save results
if options.isSaveResult
    % Delete varibales used for ROS 2 since some of them cannot be saved
    % Create comma-separated list
    empty_cells = cell(1,options.amount);
    
    result.scenario.ros_subscribers = [];
    [result.scenario.vehicles.communicate] = empty_cells{:};
    % for i_iter = 1:length(result.iteration_structs)
    %     result.iteration_structs{i_iter}.scenario = [];
    % end
    
    result.mpa = scenario.mpa;

    % Delete unimportant data
    if options.isSaveResultReduced
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
exp.end_run()

