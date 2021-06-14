function result = run_simulation(scenario, doOnlinePlot, doPlotExploration)
%% Setup
% Setup controller
info = struct;
info.trim_indices = [scenario.vehicles(:).trim_config];
% Initialize
k = 0;
cur_node = node(k, info.trim_indices, [scenario.vehicles(:).x_start]', [scenario.vehicles(:).y_start]', [scenario.vehicles(:).yaw_start]', zeros(scenario.nVeh,1), zeros(scenario.nVeh,1));
tree = Tree(cur_node);
k = k + 1;

controller = @(scenario, iter)...
    graph_search(scenario, iter);

% init result struct
result = get_result_struct(scenario);

% Visualize
% Plot controls: SPACE to pause, ESC to abort.
paused = false;
abort = false;
function keyPressCallback(~,eventdata)
    if strcmp(eventdata.Key, 'escape')
        abort = true;
    elseif strcmp(eventdata.Key, 'space')
        paused = ~paused;
    end
end
if doOnlinePlot
    resolution = [1920 1080];
    
    fig = figure(...
        'Visible','On'...
        ,'Color',[1 1 1]...
        ,'units','pixel'...
        ,'OuterPosition',[100 100 resolution(1) resolution(2)]...
    );
    set(gcf,'WindowKeyPressFcn',@keyPressCallback);
    hold on
end


%% Execute

% Main control loop
finished = false;

while ~finished && k <= scenario.k_end
    result.step_timer = tic;
    % Measurement
    % --------------------------------------------------------------------------
    speeds = zeros(scenario.nVeh, 1);
    for iVeh=1:scenario.nVeh
        speeds(iVeh) = scenario.mpa.trims(cur_node(iVeh,NodeInfo.trim)).speed;
    end
    
    x0 = [cur_node(:,NodeInfo.x), cur_node(:,NodeInfo.y), cur_node(:,NodeInfo.yaw), speeds];
    scenario_tmp = get_next_dynamic_obstacles_scenario(scenario, k);
    
    
    try
        % Control 
        % ----------------------------------------------------------------------
        % Sample reference trajectory
        iter = rhc_init(scenario,x0,info.trim_indices);
        result.iteration_structs{k} = iter;
        controller_timer = tic;
            [u, y_pred, info] = controller(scenario_tmp, iter);
        result.controller_runtime(k) = toc(controller_timer);
        % save controller outputs in result struct
        result.trajectory_predictions(:,k) = y_pred;
        result.controller_outputs{k} = u;

        % init struct for exploration plot
        if doPlotExploration
            exploration_struct.doExploration = true;
            exploration_struct.info = info;
        else
            exploration_struct = [];
        end

        % Determine next node
        assert(numel(info.tree_path)>1);
        cur_node = info.tree.node{info.tree_path(2)};

        % Add node to Tree
        tree = tree.add_nodes(k, {cur_node});

        % store vehicles path in higher resolution
        result.vehicle_path_fullres(:,k) = path_between(tree.node{end-1},tree.node{end},tree,scenario.mpa);

        result.n_expanded(k) = numel(info.tree.node);

        % Simulation
        % ----------------------------------------------------------------------

        result.step_time(k) = toc(result.step_timer);

        % Visualization
        % ----------------------------------------------------------------------
        if doOnlinePlot
            % wait to simulate realtime plotting
            pause(scenario.dt-result.step_time(k))

            % visualize time step
            plotOnline(result,k,1,exploration_struct);
        end
    catch ME
        switch ME.identifier
        case 'MATLAB:graph_search:tree_exhausted'
            warning([ME.message, ', ending search...']);
            finished = true;
        otherwise
            rethrow(ME)
        end
    end
    
    % idle while paused, and check if we should stop early
    while paused
        pause(0.1);
        if abort
            disp('Aborted.');
            finished = true;
            break;
        end
    end
    if abort
        disp('Aborted.');
        finished = true;
    end

    k = k+1;
end


%% save results
save(fullfile(result.output_path,'data.mat'),'result');

if doOnlinePlot
    close(fig)
end

end