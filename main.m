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
        case 4
            options = selection(varargin{2},varargin{3},varargin{4});
        case 3
            options = selection(varargin{2},varargin{3},1);
        case 2
            options = selection(varargin{2},2,1);
        otherwise
            options = selection();
    end
else
    disp('cpmlab')
    options = struct;
    vehicle_ids = [varargin{:}];
    options.amount = numel(vehicle_ids);
    options.isPB = false;
end
    

% scenario = circle_scenario(options.amount,options.isPB);
scenario = lanelet_scenario4_parl(options.isPB);

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
    [ x0, trim_indices ] = exp.measure();
    
    try
        % Control 
        % ----------------------------------------------------------------------
        % Sample reference trajectory
        iter = rhc_init(scenario,x0,trim_indices);
        scenario_tmp = get_next_dynamic_obstacles_scenario(scenario, k);
        
        controller_timer = tic;
            [u, y_pred, info] = scenario.controller(scenario_tmp, iter);
            
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
        
        % Apply control action f/e veh
        % -------------------------------------------------------------------------
        exp.apply(u, y_pred, info, result, k);

    % catch case where graph search could not find a new node
    catch ME
        switch ME.identifier
        case 'MATLAB:graph_search:tree_exhausted'
            warning([ME.message, ', ending search...']);
            got_stop = true;
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

exp.end_run()


end
