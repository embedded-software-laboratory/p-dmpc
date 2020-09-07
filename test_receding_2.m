% Import tree class
matlab_tree = './matlab-tree/';
assert(logical(exist(matlab_tree, 'dir')));
addpath(matlab_tree);

addpath(genpath(pwd));

warning('off','MATLAB:polyshape:repairedBySimplify')

% Choose Model
model = BicycleModel(2.2,2.2);

init_poses = [];

init_pose.x = 0;
init_pose.y = 0;
init_pose.yaw = 0.25*pi;

init_poses = [init_poses, init_pose];

init_pose.x = 15;
init_pose.y = 15;
init_pose.yaw = 1.25*pi;

init_poses = [init_poses, init_pose];

target_poses = [];

target_pose.x = 15;
target_pose.y = 15;
target_pose.yaw = 0;

target_poses = [target_poses, target_pose];

target_pose.x = 0;
target_pose.y = 0;
target_pose.yaw = 0;

target_poses = [target_poses, target_pose];

trim_indices = [5,5];

primitive_dt = 1;
depth = 10;

load('trim_set_6_1');
n_trims = length(u_trims);

motionGraph1 = MotionGraph(model, u_trims, trim_adjacency, primitive_dt);

% make motionGraph Tupel
motionGraphList = [motionGraph1, motionGraph1];

% Combine graphs
combined_graph = CombinedGraph(motionGraphList);
axis_size = [-10 20 -10 20];
[search_tree, parents] = combine_trees(init_poses, target_poses, trim_indices, combined_graph, depth, @is_collision, @get_next_node_weighted_astar);

vis_trajectory(search_tree, parents, combined_graph, target_poses, axis_size);


%% Log workspace to subfolder 
st = dbstack;
namestr = st(1).name;
sub_folder = './logs/' + string(namestr);
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