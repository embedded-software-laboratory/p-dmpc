function result = main(varargin)
%% Setup
% Add modules to path
addpath(genpath(pwd));
% warning('off','MATLAB:polyshape:repairedBySimplify')

close all
clear -regex ^(?!varargin$).*$
clc

% Determine options
if nargin==2
    options = selection(varargin{1},varargin{2});
else
    if nargin==1
        options = selection(varargin{1},2);
    else
        options = selection();
    end
end

% Setup scenario
scenario = Scenario(options);

% Setup controller
info = struct;
info.trim_indices = [scenario.vehicles(:).trim_config];
% Initialize
n_vertices = 0;
cur_depth = 0;
cur_node = node(cur_depth, info.trim_indices, [scenario.vehicles(:).x_start]', [scenario.vehicles(:).y_start]', [scenario.vehicles(:).yaw_start]', zeros(scenario.nVeh,1), zeros(scenario.nVeh,1));
search_tree = tree(cur_node);
idx = tree.nodeCols();
cur_depth = cur_depth + 1;

controller = @(scenario, iter)...
    graph_search(scenario, iter);

% init result struct
result = get_result_struct(scenario);

% Visualize
doExploration = false;
doOnlinePlot = options.visu(1);
if doOnlinePlot
    resolution = [1920 1080];
    framerate = 30;
    frame_per_step = framerate*scenario.dt;
    ticks = round(linspace(2,scenario.tick_per_step+1,frame_per_step));
    figure('Visible','On','Color',[1 1 1],'units','pixel','OuterPosition',[100 100 resolution(1)/2 resolution(2)/2]);
    hold on
    doExploration = options.visu(2);
end

% Create log folder
% st = dbstack;
% namestr = st(1).name;
% sub_folder = './logs/' + string(namestr) + '_' + string(scenario.trim_set) + '_circle_' + string(options.amount) + '_' + string(h_u) + '_' + string(h_p);
% 
% if ~exist(sub_folder, 'dir')
%     mkdir(sub_folder)
% end

%% Execute

% Main control loop
finished = false;

while ~finished && cur_depth < 50
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
    % Sample reference trajectory
    iter = rhc_init(scenario,x0,info.trim_indices);
    result.iteration_structs{cur_depth} = iter;
    controller_timer = tic;
        [u, y_pred, info] = controller(scenario, iter);
    result.controller_runtime(cur_depth) = toc(controller_timer);
    % save controller outputs in result struct
    result.trajectory_predictions(:,cur_depth) = y_pred;
    result.controller_outputs{cur_depth} = u;

    % init struct for exploration plot
    if doExploration
        exploration_struct.doExploration = true;
        exploration_struct.info = info;
    else
        exploration_struct = [];
    end
    
    % Determine next node
    % TODO Substitute with measure / simulate
    assert(numel(info.tree_path)>1);
    cur_node = info.tree.Node{info.tree_path(2)};

    % Add node to tree
    [search_tree, cur_depth] = search_tree.addnode(cur_depth, cur_node);

    % Check if we already reached our destination
    is_goals = is_goal(cur_node, scenario);
    if(sum(is_goals) == scenario.nVeh)
        finished = true;
    end
    
    % store vehicles path in higher resolution
    result.vehicle_path_fullres(:,cur_depth-1) = path_between(search_tree.Node{end-1},search_tree.Node{end},search_tree,scenario.mpa);
    
    n_vertices = n_vertices + length(info.tree.Node);
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
    
    % Simulation
    % -------------------------------------------------------------------------

    result.step_time(cur_depth) = toc(result.step_timer);
end

%% Log and visualize
% % Log workspace to subfolder 
% file_name = fullfile(sub_folder,'data');
% fig_name = fullfile(sub_folder,'fig');
% 
% if ~exist(sub_folder, 'dir')
%     mkdir(sub_folder)
% end
% 
% % Get a list of all variables
% allvars = whos;
% 
% % Identify the variables that ARE NOT graphics handles. This uses a regular
% % expression on the class of each variable to check if it's a graphics object
% tosave = cellfun(@isempty, regexp({allvars.class}, '^matlab\.(ui|graphics)\.'));
% 
% % Pass these variable names to save
% save(file_name, allvars(tosave).name)
% savefig(fig_name);
end