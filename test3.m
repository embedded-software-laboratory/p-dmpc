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

init_pose.x = 15;
init_pose.y = 0;
init_pose.yaw = 0.75*pi;

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

target_pose.x = 0;
target_pose.y = 15;
target_pose.yaw = 0;

target_poses = [target_poses, target_pose];

trim_indices = [5,5,5];

primitive_dt = 1;
depth = 20;

load('trim_set_6_1');
n_trims = length(u_trims);

motionGraph1 = MotionGraph(model, u_trims, trim_adjacency, primitive_dt);

% make motionGraph Tupel
motionGraphList = [motionGraph1, motionGraph1, motionGraph1];

% Combine graphs
combinedGraph = CombinedGraph(motionGraphList);
tic
search_tree = generate_tree(init_poses, target_poses, trim_indices, combinedGraph, depth, @is_collision, @get_next_node_weighted_astar);
toc
search_paths = return_path(search_tree);