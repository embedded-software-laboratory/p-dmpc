function result = main(varargin)
% MAIN  main function for graph-based receeding horizon control

if verLessThan('matlab','9.10')
    warning("Code is developed in MATLAB 2021a, prepare for backward incompatibilities.")
end

options = startOptions();
is_sim_lab = options.is_sim_lab;

%% Determine options
% if matlab simulation should be started with certain parameters
% first argument has to be 'sim'
%is_sim_lab = (nargin == 0 || (nargin > 0 && strcmp(varargin{1},'sim')));

if is_sim_lab
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

    switch options.amount
        % specify vehicles IDs
        case 2
            vehicle_ids = [14,16];
        case 4
            vehicle_ids = [14,16,18,20];
        case 6
            vehicle_ids = [10,14,16,17,18,20]; 
        otherwise
            vehicle_ids = 1:options.amount; % default IDs
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
    vehicle_ids = [varargin{:}];
    options.amount = numel(vehicle_ids);
    options.isPB = true;

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
        end

        if options.collisionAvoidanceMode == 1
            options.isParl = false;
            options.priority = 'mixed_traffic_priority';
            %options.priority = 'topo_priority';
        elseif options.collisionAvoidanceMode == 2 
            options.isParl = true;
            %options.priority = 'mixed_traffic_priority';
            options.priority = 'right_of_way_priority';
        else
            % Not implemented yet
            options.isParl = true;
            options.priority = 'mixed_traffic_priority';
        end
    else
        manualVehicle_id = 0;
        manualVehicle_id2 = 0;
        options.firstManualVehicleMode = 0;
        options.secondManualVehicleMode = 0;
        options.collisionAvoidanceMode = 0;
    end
end
    
% scenario = circle_scenario(options.amount,options.isPB);
% scenario = lanelet_scenario4(options.isPB,options.isParl,isROS);
 
switch options.scenario
    case 'Circle_scenario'
        scenario = circle_scenario(options);
    case 'Commonroad'
        scenario = commonroad(options, vehicle_ids, manualVehicle_id, manualVehicle_id2, is_sim_lab);  
end

scenario.name = options.scenario;
scenario.priority_option = options.priority;
scenario.manual_vehicle_id = manualVehicle_id;
scenario.second_manual_vehicle_id = manualVehicle_id2;
scenario.vehicle_ids = vehicle_ids;
scenario.mixedTrafficCollisionAvoidanceMode = options.collisionAvoidanceMode;
scenario.options = options;

for iVeh = 1:scenario.options.amount
    % initialize vehicle ids of all vehicles
    scenario.vehicles(iVeh).ID = scenario.vehicle_ids(iVeh);
end

 
if is_sim_lab
    exp = SimLab(scenario, options);
else
    exp = CPMLab(scenario, vehicle_ids);
end
%scenario.pool = exp.parallelPool;

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

if options.isParl && strcmp(scenario.name, 'Commonroad')
    % In parallel computation, vehicles communicate via ROS 2
    % Initialize the communication network of ROS 2
    scenario = communication_init(scenario, exp);
end

vehs_fallback_times = zeros(1,scenario.options.amount); % record the number of successive fallback times of each vehicle 
info_old = []; % old information for fallback
total_fallback_times = 0; % total times of fallbacks

if scenario.options.firstManualVehicleMode == 2 || scenario.options.secondManualVehicleMode == 2
    r = rosrate(100000);
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

    %disp(['>>> Time step ' num2str(scenario.k) ''])


    % Control
    % ----------------------------------------------------------------------
     
    % Update the iteration data and sample reference trajectory
    [iter,scenario] = rhc_init(scenario,x0_measured,trims_measured, initialized_reference_path, is_sim_lab);
    
    if (~initialized_reference_path || scenario.updated_manual_vehicle_path || scenario.updated_second_manual_vehicle_path)
        %exp.update();
    end
    initialized_reference_path = true;

    if scenario.options.is_mixed_traffic
        if scenario.manual_vehicle_id ~= 0
            % function that updates the steering wheel data
            %wheelData = exp.getWheelData();
            %wheelData = exp.get_stored_wheel_msgs();

            if (scenario.options.firstManualVehicleMode == 1)
                wheelData = exp.getWheelData();
                % function that translates current steering angle into lane change and velocity profile inputs into velocity changes
                modeHandler = GuidedMode(scenario,x0_measured,scenario.manual_vehicle_id,vehicle_ids,cooldown_after_lane_change,speedProfileMPAsInitialized,wheelData,true);
                scenario = modeHandler.scenario;
                scenario.updated_manual_vehicle_path = modeHandler.updatedPath;
                speedProfileMPAsInitialized = true;
                
            elseif scenario.options.firstManualVehicleMode == 2
                % classify steering angle into intervals and send according steering command
                %D = parallel.pool.DataQueue;
                %afterEach(D, @(steeringWheelCallback) ExpertMode(exp, scenario, true, scenario.manual_vehicle_id));
                %spmd(2)
                    %wheelData = exp.getWheelData();
                    %input = [exp, scenario, true, scenario.manual_vehicle_id, wheelData];
                    %handle = @ExpertMode;
                    input = struct;
                    input.exp = exp;
                    input.scenario = scenario;
                    input.steeringWheel = true;
                    input.vehicle_id = scenario.manual_vehicle_id;

                    waitfor(r);
    
                    %parfeval(@ExpertMode, 0, input);
                    %parfeval(scenario.pool, handle(input), 0);
                %end
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
                
            elseif scenario.options.secondManualVehicleMode == 2
                % classify steering angle into intervals and send according steering command
                modeHandler = ExpertMode(exp, scenario, gamepadData, false, scenario.second_manual_vehicle_id);
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
    vehs_fallback_times(info.vehs_fallback) = vehs_fallback_times(info.vehs_fallback) + 1;
    vehs_not_fallback = setdiff(1:scenario.options.amount, info.vehs_fallback);
    vehs_fallback_times(vehs_not_fallback) = 0; % reset
    
    % check whether at least one vehicle has fallen back Hp times successively
    if max(vehs_fallback_times) < scenario.Hp
        if ~isempty(info.vehs_fallback)
            real_vehicles = zeros(1,length(info.vehs_fallback));

            for i=1:length(info.vehs_fallback)
                real_vehicles(i) = scenario.vehicle_ids(info.vehs_fallback(i));
            end

            disp_tmp = sprintf('%d,',real_vehicles); disp_tmp(end) = [];
            disp(['*** Vehicles ' disp_tmp ' take fallback.']) % use * to highlight this message
            info = pb_controller_fallback(info, info_old, scenario);
            total_fallback_times = total_fallback_times + 1;
        end
    else
        disp('Already fall back successively Hp times, terminate the simulation')
        disp(['Total times of fallback: ' num2str(total_fallback_times) '.'])
        break % break the while loop
    end

    info_old = info; % save variable in case of fallback
    %% save result
    result.controller_runtime(k) = toc(controller_timer);
    
    % save controller outputs in result struct
    result.scenario = scenario;
    result.iteration_structs{k} = iter;
    result.trajectory_predictions(:,k) = info.y_predicted;
    result.controller_outputs{k} = info.u;
    result.subcontroller_runtime(:,k) = info.subcontroller_runtime;
    result.vehicle_path_fullres(:,k) = info.vehicle_fullres_path(:);
    result.n_expanded(k) = info.n_expanded;
    result.priority(:,k) = scenario.priority_list;
    result.computation_levels(k) = info.computation_levels;
    result.step_time(k) = toc(result.step_timer);
    result.subcontroller_run_time_total(k) = info.subcontroller_run_time_total;
    if options.isParl
        result.subcontroller_runtime_all_grps{k} = info.subcontroller_runtime_all_grps; % subcontroller runtime of each parallel group 
    end
   
    % Apply control action
    % -------------------------------------------------------------------------
    exp.apply(info, result, k, scenario); 
    
    % Check for stop signal
    % -------------------------------------------------------------------------
    got_stop = exp.is_stop() || got_stop;
    
end

%delete(gcp('nocreate'));
%% save results

% Delete varibales used for ROS 2 since some of them cannot be saved
% Create comma-separated list
empty_cells = cell(1,options.amount);

result.scenario.ros_subscribers = [];
[result.scenario.vehicles.communicate] = empty_cells{:};
% for i_iter = 1:length(result.iteration_structs)
%     result.iteration_structs{i_iter}.scenario = [];
% end

result.mpa = scenario.mpa;
% tic_start = tic;
save(result.output_path,'result');
disp(['Simulation results were saved under ' result.output_path])
% disp(['Result was saved in ' num2str(toc(tic_start)) ' seconds.'])
% exportVideo( result );
exp.end_run()

end

