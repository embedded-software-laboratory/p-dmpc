%% Setup
% Add modules to path
addpath(genpath(pwd));
% warning('off','MATLAB:polyshape:repairedBySimplify')

close all
clear
clc

options = selection();

scenario = Scenario(options);



% Load costs for A*-Star
load('situation_costs', 'situation_costs');

% Set figure
f = figure;
f.WindowState = 'maximized';
set(f,'color','w');
set(gca, 'FontSize', 35);
set(gca,'TickLabelInterpreter','latex');
set(gca,'Box','on')
xlabel('$x$ [m]', 'Interpreter','Latex')
ylabel('$y$ [m]', 'Interpreter','Latex')
box on;
axis([-18, 18, -18, 18]);
%set(gca, 'ytick', [-9 9]);
pbaspect([1 1 1]);
title("Iteration: 0, Time: 0");
draw_destination(scenario);
draw_cars(scenario);
if numel(scenario.obstacles) ~= 0
    for i = 1:numel(scenario.obstacles)
        obstacles = polyshape(scenario.obstacles{i}(1,:),scenario.obstacles{i}(2,:));
        plot(obstacles, 'EdgeColor', 'blue', 'LineWidth', 2);
    end
end

% Create log folder
st = dbstack;
namestr = st(1).name;
sub_folder = './logs/' + string(namestr) + '_' + string(scenario.trim_set) + '_circle_' + string(options.amount) + '_' + string(h_u) + '_' + string(h_p);

if ~exist(sub_folder, 'dir')
    mkdir(sub_folder)
end

% Initialize video
% i = 1;
% video_name = fullfile(sub_folder,"video" + "(" + string(i) + ")");
% while isfile(video_name+ ".avi")
%     i = i + 1;
%     video_name = fullfile(sub_folder,"video" + "(" + string(i) + ")");
% end
video_name = fullfile(sub_folder,"video");
video = VideoWriter(video_name);
video.FrameRate = 1;
open(video)



trim_indices = [scenario.vehicles(:).trim_config];
% Initialize
n_vertices = 0;
horizon = cell(1, 3);
p = gobjects(1, scenario.nVeh);
g = gobjects(1, scenario.nVeh);
for i = 1:scenario.nVeh
    cur_color = vehColor(i);
    p(i) = plot(scenario.vehicles(i).x_start, scenario.vehicles(i).y_start, '-','Color', cur_color, 'LineWidth', 2);
    p(i).Color(4) = 0.5;
    g(i) = plot(scenario.vehicles(i).x_start, scenario.vehicles(i).y_start, 'o','Color', cur_color, 'MarkerSize',3,'MarkerFaceColor', cur_color);
    g(i).Color(4) = 0.5;
end
cur_depth = 0;
cur_node = node(cur_depth, trim_indices, [scenario.vehicles(:).x_start]', [scenario.vehicles(:).y_start]', [scenario.vehicles(:).yaw_start]', zeros(scenario.nVeh,1), zeros(scenario.nVeh,1));
search_tree = tree(cur_node);
cur_depth = cur_depth + 1;


controller = @(scenario, iter, prev_info)...
    graph_search(scenario, iter, prev_info);
%% Execute

% Main control loop
finished = false;
prev_info = struct;
prev_info.trim_indices = trim_indices;



% for iVeh = 1:scenario.nVeh
%     c = vehColor(iVeh);
%     prev_info.plot.openNodes(iVeh) = scatter(0, 0 ...
%         ,'LineWidth', 3 ...
%         ,'MarkerEdgeColor', c ...
%         ,'MarkerFaceColor', c ...
%         ,'MarkerEdgeAlpha', 0.4 ...
%         ,'MarkerFaceAlpha', 0.4 ...
%     );
% end
while ~finished || cur_depth > 50
    % Measurement
    % -------------------------------------------------------------------------
    % TODO no real measurement in trajectory following.
    % Coud use vehicles' predicted mpc traj.
    speeds = zeros(scenario.nVeh, 1);
    for iVeh=1:scenario.nVeh
        speeds(iVeh) = scenario.combined_graph.motionGraphList(iVeh).trims(cur_node.trims(iVeh)).velocity;
    end
    x0 = [cur_node.xs, cur_node.ys, cur_node.yaws, speeds];
    % Sample reference trajectory
    iter = rhc_init(scenario,x0);
    
    
    % Control 
    % -------------------------------------------------------------------------
    [u, y_pred, info] = controller(scenario, iter, prev_info);
            
    n_vertices = n_vertices + length(info.tree.Node);
    
    % Visualization
    % -------------------------------------------------------------------------
    % Visualize horizon
    for i = 1:scenario.nVeh
        path = y_pred{i};
        if ~isempty(path)
            p(i).XData = path(:,1);
            p(i).YData = path(:,2);
        end
    end
    drawnow;
    
    
    % Visualize path before addition to properly display horizon
    visualize_step(search_tree, cur_depth, scenario.combined_graph);
    frame = getframe(gcf);
    writeVideo(video, frame);

    % Determine next node
    % TODO Substitute with measure / simulate
    assert(numel(info.tree_path)>1);
    cur_node = info.tree.Node{info.tree_path(2)};

    % Add node to tree
    [search_tree, cur_depth] = search_tree.addnode(cur_depth, cur_node);

    % Check if we already reached our destination
    is_goals = is_goal(cur_node, scenario);
    if(sum(is_goals) == scenario.nVeh)
        visualize_step(search_tree, cur_depth, scenario.combined_graph);
        frame = getframe(gcf);
        writeVideo(video, frame);
        finished = true;
    end
    
    % Simulation
    % -------------------------------------------------------------------------



    prev_info = info;
end
close(video);



%% Log and visualize
% Log workspace to subfolder 
file_name = fullfile(sub_folder,'data');
fig_name = fullfile(sub_folder,'fig');

if ~exist(sub_folder, 'dir')
    mkdir(sub_folder)
end

% Get a list of all variables
allvars = whos;

% Identify the variables that ARE NOT graphics handles. This uses a regular
% expression on the class of each variable to check if it's a graphics object
tosave = cellfun(@isempty, regexp({allvars.class}, '^matlab\.(ui|graphics)\.'));

% Pass these variable names to save
save(file_name, allvars(tosave).name)
savefig(fig_name);