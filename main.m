function result = main(varargin)
% MAIN  main function for graph-based receeding horizon control

if verLessThan('matlab','9.10')
    warning("Code is developed in MATLAB 2021a, prepare for backward incompatibilities.")
end

%% Determine options
% if matlab simulation should be started with certain parameters
% first argument has to be 'sim'
is_sim_lab = (nargin == 0 || (nargin > 0 && strcmp(varargin{1},'sim')));

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
            options = selection();
    end
    vehicle_ids = 1:options.amount; % default IDs
%     vehicle_ids = [10,14,16,17,18,20]; % specify vehicles IDs
    
else
    disp('cpmlab')
    options = struct;
    vehicle_ids = [varargin{:}];
    options.amount = numel(vehicle_ids);
    options.isPB = true;
    options.scenario = 'Commonroad';
    options.priority = 'topo_priority';
end
    
% scenario = circle_scenario(options.amount,options.isPB);
% scenario = lanelet_scenario4(options.isPB,options.isParl,isROS);

% plot_reachable_sets_offline(scenario.mpa)
 
switch options.scenario
    case 'Circle_scenario'
        scenario = circle_scenario(options);
    case 'Commonroad'
        scenario = commonroad(vehicle_ids,options);
end

 
if is_sim_lab
    exp = SimLab(scenario, options);
else
    exp = CPMLab(scenario, vehicle_ids);
end


%% Setup
% Initialize
got_stop = false;
k = 0;

% init result struct
result = get_result_struct(scenario);

exp.setup();

% group_and_prioritize_vehicles(scenario.communication)
fallback=0;

scenario.k = k;

% turn off warning if intersections are detected and fixed, collinear points or
% overlapping points are removed when using MATLAB function `polyshape`
warning('off','MATLAB:polyshape:repairedBySimplify')

if options.isParl && strcmp(scenario.name, 'Commonroad')
    % In parallel computation, vehicles communicate via ROS 2
    % Initialize the communication network of ROS 2
    scenario = communication_init(scenario, exp);
end


%% Main control loop
while (~got_stop)

    result.step_timer = tic;
    
    % increment interation counter
    k = k+1;
    
    % Measurement
    % -------------------------------------------------------------------------
    [x0_measured, trims_measured] = exp.measure();% trim_indices： which trim  
    scenario.k = k;

    disp(['Time step ' num2str(scenario.k) '.'])

    try
        % Control
        % ----------------------------------------------------------------------
        
        % Update the iteration data
        iter = rhc_init(scenario, x0_measured, trims_measured, options); 
        
        % For parallel computation, information from previous time step is need, for example, 
        % the previous fail-safe trajectory is used again if a new fail-safe trajectory cannot be found.
        
        % update the coupling information
        if strcmp(scenario.name, 'Commonroad')
            % update the lanelet boundary for each vehicle
            for iVeh = 1:options.amount
                scenario.vehicles(iVeh).lanelet_boundary = iter.predicted_lanelet_boundary(iVeh,:);
            end

            if options.isParl
                % update the coupling information
                scenario = get_coupling_infos(scenario, iter);
            else
                % update the coupling information
                scenario = coupling_adjacency(scenario, iter);
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
            [u, y_pred, info] = scenario.controller(scenario_tmp, iter);
        scenario.last_veh_at_intersection = info.veh_at_intersection;
        result.controller_runtime(k) = toc(controller_timer);
        result.iteration_structs{k} = iter;
        
        % save controller outputs in result struct
        result.scenario = scenario;
        result.iteration_structs{k} = iter;
        result.trajectory_predictions(:,k) = y_pred;
        result.controller_outputs{k} = u;
        result.subcontroller_runtime(:,k) = info.subcontroller_runtime;
        result.vehicle_path_fullres(:,k) = info.vehicle_fullres_path(:);
        result.n_expanded(k) = info.n_expanded;
        result.priority(:,k) = info.priority_list;
        result.computation_levels(k) = info.computation_levels;
        result.edges_to_break{k} = info.edge_to_break;
        result.step_time(k) = toc(result.step_timer);
        if options.isParl
            result.subcontroller_runtime_all_grps{k} = info.subcontroller_runtime_all_grps; % subcontroller run time of each parallel group 
            result.subcontroller_runtime_max(k) = max(result.subcontroller_runtime_all_grps{k}); % maximum subcontroller run time among all parallel groups
            result.belonging_vector{k} = info.belonging_vector;
        end
       
        % Apply control action
        % -------------------------------------------------------------------------
        exp.apply(u, y_pred, info, result, k);
        
        fallback_update = 0;
        
    % catch case where graph search could not find a new node
     catch ME
        switch ME.identifier
        case 'MATLAB:graph_search:tree_exhausted'
            warning([ME.message, ', ME, fallback to last priority.............']);
             
            fallback = fallback + 1;
            fallback_update = fallback_update + 1;
            disp(['fallback: ', num2str(fallback)])

            % fallback to last plan
            controller_timer = tic;
            [u, y_pred, info] = pb_controller_fallback(scenario, u, y_pred, info);
            result.controller_runtime(k) = toc(controller_timer);
            
            % save controller outputs in result struct
            result.scenario = scenario;
            result.iteration_structs{k} = iter;
            result.trajectory_predictions(:,k) = y_pred;
            result.controller_outputs{k} = u;
            result.subcontroller_runtime(:,k) = info.subcontroller_runtime;
            result.vehicle_path_fullres(:,k) = info.vehicle_fullres_path(:);
            result.n_expanded(k) = info.n_expanded;
            result.priority(:,k) = info.priority_list;
            result.computation_levels(k) = info.computation_levels;
            result.edges_to_break{k} = info.edge_to_break;
            result.step_time(k) = toc(result.step_timer);
            result.fallback = fallback;

            % Apply control action
            % -------------------------------------------------------------------------
            exp.apply(u, y_pred, info, result, k); 
            
            % if fallback to last plan Hp times continuously, the vehicle
            % will stop at the current position, terminate the simulation
            if fallback_update == scenario.Hp
                got_stop = true;
                disp('Already fallback Hp times, terminate the simulation')
            end
            
        otherwise
            rethrow(ME)
        end
    end
    
    % Check for stop signal
    % -------------------------------------------------------------------------
    got_stop = exp.is_stop() || got_stop;
    
end
%% save results
% result.mpa = scenario.mpa;
% save(fullfile(result.output_path,'data.mat'),'result');
% exportVideo( result );
exp.end_run()
end
