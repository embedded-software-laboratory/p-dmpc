function result = main_update(varargin)
% MAIN  main function for graph-based receeding horizon control with
% right-of-way priority assignment. Vehicles at intersection keep
% higher priority and do not change their relative priority until they leave intersection

if verLessThan('matlab','9.10')
    warning("Code is developed in MATLAB 2021a, prepare for backward incompatibilities.")
end

%% Determine options
% if matlab simulation should be started with certain parameters
% first argument has to be 'sim'
is_sim_lab = (nargin == 0 || (nargin > 0 && strcmp(varargin{1},'sim')));

if is_sim_lab
    switch nargin
        case 4
            options = selection(varargin{2},varargin{3},varargin{4});
        case 3
            options = selection(varargin{2},varargin{3},1);
        case 2
            options = selection(varargin{2},2,1);
        otherwise
            options = selection();
    end
    vehicle_ids = 1:20;
    
else
    disp('cpmlab')
    options = struct;
    vehicle_ids = [varargin{:}];
    options.amount = numel(vehicle_ids);
    options.isPB = true;
end
  

scenario = commonroad(options.amount,vehicle_ids,options.isPB);

% scenario = lanelet_scenario4(options.isPB);

% scenario = circle_scenario(options.amount,options.isPB);

if is_sim_lab
    exp = SimLab(scenario, options);
else
    exp = CPMLab(scenario, vehicle_ids);
end


%% Setup
% Initialize
got_stop = false;
k = 1;

% init result struct
result = get_result_struct(scenario);

exp.setup();
fallback=0;

%% Main control loop
while (~got_stop)
    
    result.step_timer = tic;
    % Measurement
    % -------------------------------------------------------------------------
    [ x0, trim_indices ] = exp.measure();% trim_indices： which trim  
    scenario.k = k;

    try
        % Control 
        % ----------------------------------------------------------------------
        
        % Sample reference trajectory
        iter = rhc_init(scenario,x0,trim_indices);
        
        % update the boundary information of each vehicle
        if strcmp(scenario.name, 'Commonroad')
            lanelet_boundary = lanelets_boundary(scenario, iter);
            for iveh = 1:options.amount
                scenario.vehicles(1,iveh).lanelet_boundary = lanelet_boundary{1,iveh};
                %disp(sprintf('left, veh: %d', iveh));
                %disp(lanelet_boundary{1,iveh}{1,1});
                %disp(sprintf('right, veh: %d', iveh));
                %disp(lanelet_boundary{1,iveh}{1,2});
            end
            % update the coupling information
            scenario = coupling_adjacency(scenario,iter);
        end
        
        % calculate the distance 
        distance = zeros(options.amount,options.amount);
        adjacency = scenario.adjacency(:,:,end);

        for vehi = 1:options.amount-1
            adjacent_vehicle = find(adjacency(vehi,:));
            adjacent_vehicle = adjacent_vehicle(adjacent_vehicle > vehi);
            for vehn = adjacent_vehicle
                distance(vehi,vehn) = check_distance(iter,vehi,vehn);
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
    
    % increment interation counter
    k = k+1;
end
%% save results
save(fullfile(result.output_path,'data.mat'),'result');
% exportVideo( result );
exp.end_run()
end
