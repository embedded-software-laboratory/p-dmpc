%% Add modules to path
% Import tree class
matlab_tree = './matlab-tree/';
assert(logical(exist(matlab_tree, 'dir')));
addpath(matlab_tree);

addpath(genpath(pwd));

%% Define motion graph
% Choose Model
model = BicycleModel(2.2,2.2);


%% Define scenario
% Obstacles
% Vehicles
n_veh = 2;
scenario = Scenario(2*pi/n_veh*(1:n_veh));

% Initial position
init_poses = [];
trim_indices = 5 * ones(1, n_veh);

for i = 1:n_veh
    init_pose.x = scenario.vehicles(i).x_start;
    init_pose.y = scenario.vehicles(i).y_start;
    init_pose.yaw = scenario.vehicles(i).heading;
    init_poses = [init_poses, init_pose];
end
    


% Goal
target_poses = [];
step = floor(n_veh / 2);

for i = 1:n_veh
    
    k = mod(i - 1 + step, n_veh) + 1;
    
    target_pose.x = scenario.vehicles(k).x_start;
    target_pose.y = scenario.vehicles(k).y_start;
    target_pose.yaw = scenario.vehicles(k).heading;
    target_poses = [target_poses, target_pose];
end

% trims left to right
load('trim_inputs_6');
load('trim_adjacency_6_1');
n_trims = length(u_trims);

primitive_dt = 1;
depth = 10;

motionGraph1 = MotionGraph(model, u_trims, trim_adjacency, primitive_dt);

% make motionGraph Tupel
motionGraphList = [motionGraph1, motionGraph1];

% Combine graphs
combinedGraph = CombinedGraph(motionGraphList);
%% Graph search
% Choose search algorithm

% Search
search_tree = generate_tree(init_poses, target_poses, trim_indices, combinedGraph, depth, @is_collision, @get_next_node_weighted_astar);

%% Visualize
fig = figure('Name','Search Tree');
h = plot(search_tree);

%% Log workspace to subfolder 
st = dbstack;
namestr = st.name;
sub_folder = './logs/' + string(st.name);
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