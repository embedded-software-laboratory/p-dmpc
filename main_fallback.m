function result = main_fallback(varargin)
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
        case 4
            options = selection(varargin{2},varargin{3},varargin{4});
        case 3
            options = selection(varargin{2},varargin{3},1);
        case 2
            options = selection(varargin{2},2,1);
        otherwise
            options = selection();
    end
%     vehicle_ids = [1];
%     vehicle_ids = [1,3,5,8,10,13,15,17,18,20];
%     vehicle_ids = [1,2,3,4,5,7,8,12,14,17,19]; % ok
    vehicle_ids = [1,2,3,4,5,7,8,9,11,12,14,16,17,18,19]; % ok
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
%% Main control loop
while (~got_stop)
    
    result.step_timer = tic;
    % Measurement
    % -------------------------------------------------------------------------
    [ x0, trim_indices ] = exp.measure();% trim_indices： which trim  

%     disp('x0 is:')
%     disp(x0)
    try
        % Control 
        % ----------------------------------------------------------------------
        % Sample reference trajectory
        iter = rhc_init(scenario,x0,trim_indices);
        
        if ~isempty(scenario.lanelets)
            lanelet_boundary = lanelets_boundary(scenario, iter);% update the boundary information of each vehicle
            for iveh = 1:options.amount
                scenario.vehicles(1,iveh).lanelet_boundary = lanelet_boundary{1,iveh};
            end
            scenario.adjacency(:,:,k) = coupling_adjacency(scenario,iter);
        end
        
        
%         disp('adjacency_matrix is:')
%         disp(scenario.adjacency)
        scenario_tmp = get_next_dynamic_obstacles_scenario(scenario, k);
        result.iter_runtime(k) = toc(result.step_timer);
        result.scenario = scenario;
        
        controller_timer = tic;
            [u, y_pred, info, priority_list] = scenario.controller(scenario_tmp, iter);

        result.controller_runtime(k) = toc(controller_timer);
        result.iteration_structs{k} = iter;
        % save controller outputs in result struct
        result.trajectory_predictions(:,k) = y_pred;
        result.controller_outputs{k} = u;
        result.subcontroller_runtime(:,k) = info.subcontroller_runtime;
        % store vehicles path in higher resolution
        result.vehicle_path_fullres(:,k) = info.vehicle_fullres_path(:);
        result.n_expanded(k) = info.n_expanded;
        result.step_time(k) = toc(result.step_timer);
        result.priority(:,k) = priority_list;
        
        % Apply control action f/e veh
        % -------------------------------------------------------------------------
        apply_timer = tic;
        exp.apply(u, y_pred, info, result, k);
        result.apply_runtime(k) = toc(apply_timer);
        result.total_runtime(k) = toc(result.step_timer);
        
    % catch case where graph search could not find a new node
    catch ME
        switch ME.identifier
        case 'MATLAB:graph_search:tree_exhausted'
%             warning([ME.message, ', ending search...']);

            disp('ME, fallback to last priority...............................')  
      
        last_priority =  result.priority(:,end);    
        controller_timer = tic;
            [u, y_pred, info, priority_list] = re_pb_controller_fallback(scenario, iter, last_priority);

        result.controller_runtime(k) = toc(controller_timer);
        result.iteration_structs{k} = iter;
        % save controller outputs in result struct
        result.trajectory_predictions(:,k) = y_pred;
        result.controller_outputs{k} = u;
        result.subcontroller_runtime(:,k) = info.subcontroller_runtime;
        % store vehicles path in higher resolution
        result.vehicle_path_fullres(:,k) = info.vehicle_fullres_path(:);
        result.n_expanded(k) = info.n_expanded;
        result.step_time(k) = toc(result.step_timer);
        result.priority(:,k) = priority_list;
        
        % Apply control action f/e veh
        % -------------------------------------------------------------------------
        apply_timer = tic;
        exp.apply(u, y_pred, info, result, k);
        result.apply_runtime(k) = toc(apply_timer);
        result.total_runtime(k) = toc(result.step_timer);            
            
        disp('plotting finished ........................................')     
            

%             got_stop = true;
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
exportVideo( result );
exp.end_run()
% a;
end
