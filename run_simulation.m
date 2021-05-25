function result = run_simulation(scenario, doOnlinePlot, doPlotExploration)
%% Setup
% Setup controller
info = struct;
info.trim_indices = [scenario.vehicles(:).trim_config];
% Initialize
cur_depth = 0;
cur_node = node(cur_depth, info.trim_indices, [scenario.vehicles(:).x_start]', [scenario.vehicles(:).y_start]', [scenario.vehicles(:).yaw_start]', zeros(scenario.nVeh,1), zeros(scenario.nVeh,1));
idx = tree.nodeCols();
cur_depth = cur_depth + 1;

trim_pred_mat = zeros(scenario.Hp+1,(scenario.Hp+1)*8);
for k = 1:scenario.Hp+1
    trim_pred_mat(k,(k-1)*8+idx.trim) = 1;
end

controller = @(scenario, iter)...
    scenario.controller(scenario, iter);

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
    framerate = 30;
    frame_per_step = framerate*scenario.dt;
    ticks = round(linspace(2,scenario.tick_per_step+1,frame_per_step));
    
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

while cur_depth <= 15
    result.step_timer = tic;
    % Measurement
    % -------------------------------------------------------------------------
    % TODO no real measurement in trajectory following.
    % Coud use vehicles' predicted mpc traj.
    speeds = zeros(scenario.nVeh, 1);
    for iVeh=1:scenario.nVeh
        speeds(iVeh) = scenario.mpa.trims(cur_node(iVeh,idx.trim)).speed;
    end
    x0 = [cur_node(:,idx.x), cur_node(:,idx.y), cur_node(:,idx.yaw), speeds];
    
    % Control 
    % -------------------------------------------------------------------------
    try
        % Sample reference trajectory
        iter = rhc_init(scenario,x0,cur_node(:,idx.trim));
        result.iteration_structs{cur_depth} = iter;
        controller_timer = tic;
            [u, y_pred, info] = controller(scenario, iter);
        result.controller_runtime(cur_depth) = toc(controller_timer);
        % save controller outputs in result struct
        result.trajectory_predictions(:,cur_depth) = y_pred;
        result.controller_outputs{cur_depth} = u;
        result.subcontroller_runtime(:,cur_depth) = get_subcontroller_runtime(info,scenario);

        % Trims
        % TODO: Wofür wird das gebraucht? Funktioniert nicht mit pbnc
%         trim_pred = trim_pred_mat*[info.tree.Node{info.tree_path}]';

        % init struct for exploration plot
        if doPlotExploration
            exploration_struct.doExploration = true;
            exploration_struct.info = info;
        else
            exploration_struct = [];
        end

        % Determine next node
        % TODO Substitute with measure / simulate
        cur_node = get_cur_node(info,scenario);

        % store vehicles path in higher resolution
        result.vehicle_path_fullres(:,cur_depth) = get_fullres_path(info,scenario);
        
        cur_depth = cur_depth+1;

        %result.n_expanded(:) = result.n_expanded(:) + numel(info.tree{:}.Node);
        % Visualization
        % -------------------------------------------------------------------------
        % tune resolution
        if doOnlinePlot && cur_depth == 2
            plotOnline(result,1,1,[]);
            drawnow;
        end

        if doOnlinePlot
            % visualize time step
            for tick = ticks
                plotOnline(result,cur_depth-1,tick,exploration_struct);
            end
            drawnow;
        end
    catch ME
        switch ME.identifier
        case 'graph_search:tree_exhausted'
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

    % Simulation
    % -------------------------------------------------------------------------

    result.step_time(cur_depth) = toc(result.step_timer);
end


%% save results
save(fullfile(result.output_path,'data.mat'),'result');

if doOnlinePlot
    close(fig)
end

end