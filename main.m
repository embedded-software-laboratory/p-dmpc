function result = main(varargin)
%% Setup
% Add modules to path
addpath(genpath(pwd));
% warning('off','MATLAB:polyshape:repairedBySimplify')

close all
clear -regex ^(?!varargin$).*$
clc

% Determine options
if nargin==1
    options = selection(varargin{1});
else
    options = selection();
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
result = get_result_struct(scenario,50);

% Visualize
f = figure;
hold on
f.WindowState = 'maximized';
set(f,'color','w');
set(gca, 'FontSize', 35);
set(gca,'TickLabelInterpreter','latex');
set(gca,'Box','on')
xlabel('$x$ [m]', 'Interpreter','Latex')
ylabel('$y$ [m]', 'Interpreter','Latex')
axis([-1, 1, -1, 1]*2.5);
%set(gca, 'ytick', [-9 9]);
pbaspect([1 1 1]);
title("Iteration: 0, Time: 0");
draw_destination(scenario);
plot(scenario);
for i = 1:numel(scenario.obstacles)
    obstacles = polyshape(scenario.obstacles{i}(1,:),scenario.obstacles{i}(2,:));
    plot(obstacles, 'EdgeColor', 'blue', 'LineWidth', 2);
end
% init graphic objects
vis = struct;
vis.p = gobjects(1, scenario.nVeh);
vis.g = gobjects(1, scenario.nVeh);
for iVeh = 1:scenario.nVeh
    cur_color = vehColor(iVeh);
    vis.p(iVeh) = plot(scenario.vehicles(iVeh).x_start, scenario.vehicles(iVeh).y_start, '-','Color', cur_color, 'LineWidth', 2);
    vis.p(iVeh).Color(4) = 0.5;
    vis.g(iVeh) = plot(scenario.vehicles(iVeh).x_start, scenario.vehicles(iVeh).y_start, 'o','Color', cur_color, 'MarkerSize',3,'MarkerFaceColor', cur_color);
    vis.g(iVeh).Color(4) = 0.5;
%     vis.search(iVeh) = scatter3(0, 0, 0);
    vis.open = line(NaN,NaN,NaN, 'Marker', 'o', 'MarkerSize',6,'LineStyle','none');
    vis.open_leaves = line(NaN,NaN,NaN, 'Marker', '+', 'MarkerSize',6,'LineStyle','none');
end

% Create log folder
% st = dbstack;
% namestr = st(1).name;
% sub_folder = './logs/' + string(namestr) + '_' + string(scenario.trim_set) + '_circle_' + string(options.amount) + '_' + string(h_u) + '_' + string(h_p);
% 
% if ~exist(sub_folder, 'dir')
%     mkdir(sub_folder)
% end

% Initialize video
% i = 1;
% video_name = fullfile(sub_folder,"video" + "(" + string(i) + ")");
% while isfile(video_name+ ".avi")
%     i = i + 1;
%     video_name = fullfile(sub_folder,"video" + "(" + string(i) + ")");
% end
% video_name = fullfile(sub_folder,"video");
% video = VideoWriter(video_name);
% video.FrameRate = 1;
% open(video)




%% Execute

% Main control loop
finished = false;

while ~finished || cur_depth > 50
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
    % save controller outputs in resultstruct
    result.trajectory_predictions(:,cur_depth) = y_pred;
    result.controller_outputs{cur_depth} = u;
            
    n_vertices = n_vertices + length(info.tree.Node);
    
    % Visualization
    % -------------------------------------------------------------------------
    for iVeh = 1:scenario.nVeh
        % Visualize horizon
        path = y_pred{iVeh};
        if ~isempty(path)
            vis.p(iVeh).XData = path(:,1);
            vis.p(iVeh).YData = path(:,2);
        end
    end
    % Visualize exploration
    vis = visualize_exploration(scenario, info, vis);


    drawnow;
    
    
    % Visualize path before addition to properly display horizon
    visualize_step(scenario, search_tree, cur_depth, scenario.mpa);
%     frame = getframe(gcf);
%     writeVideo(video, frame);

    % Determine next node
    % TODO Substitute with measure / simulate
    assert(numel(info.tree_path)>1);
    cur_node = info.tree.Node{info.tree_path(2)};

    % Add node to tree
    [search_tree, cur_depth] = search_tree.addnode(cur_depth, cur_node);

    % Check if we already reached our destination
    is_goals = is_goal(cur_node, scenario);
    if(sum(is_goals) == scenario.nVeh)
        visualize_step(scenario, search_tree, cur_depth, scenario.mpa);
%         frame = getframe(gcf);
%         writeVideo(video, frame);
        finished = true;
    end
    
    % Simulation
    % -------------------------------------------------------------------------

    result.step_time(cur_depth) = toc(result.step_timer);
end
% close(video);

% store vehicles path in higher resolution
for nn=1:numel(search_tree.Node)-1
    result.vehicle_path_fullres(:,nn) = path_between(search_tree.Node{nn},search_tree.Node{nn+1},search_tree,scenario.mpa);
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