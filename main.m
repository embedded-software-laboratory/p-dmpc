function result = main(varargin)
% MAIN  main function for graph-based receeding horizon control

close all
clc

%% Determine options
is_cpm_lab = (nargin > 0 && strcmp(varargin{1},'cpmlab'));

if ~is_cpm_lab
    switch nargin
        case 3
            options = selection(varargin{1},varargin{2},varargin{3});
        case 2
            options = selection(varargin{1},varargin{2},1);
        case 1
            options = selection(varargin{1},2,1);
        otherwise
            options = selection();
    end
else
    disp('cpmlab')
    options = struct;
    vehicle_ids = [varargin{2:end}];
    options.amount = numel(vehicle_ids);
    options.isPB = false;
end
    

scenario = circle_scenario(options.amount, options.isPB);

if is_cpm_lab
    env = CPMLab(scenario, vehicle_ids);
else
    env = SimLab(scenario, options);
end

%% Setup
% Initialize
got_stop = false;
k = 1;

% init result struct
result = get_result_struct(scenario);

env.setup();

%% Main control loop
while (~got_stop)
    result.step_timer = tic;
    % Measurement
    % -------------------------------------------------------------------------
    [ x0, trim_indices ] = env.measure();
    
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
        env.apply(u, y_pred, info, result, k);

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
    got_stop = env.is_stop() || got_stop;
    
    % increment interation counter
    k = k+1;
end

%% save results
save(fullfile(result.output_path,'data.mat'),'result');

env.end_run()


end