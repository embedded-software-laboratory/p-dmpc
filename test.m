addpath(genpath(pwd));

% Choose Model
model = BicycleModel(2.2,2.2);

init_poses = [];

init_pose.x = 0;
init_pose.y = 0;
init_pose.yaw = 0;

init_poses = [init_poses, init_pose];

init_pose.x = 10;
init_pose.y = 10;
init_pose.yaw = pi;

init_poses = [init_poses, init_pose];

target_poses = [];

target_pose.x = 10;
target_pose.y = 10;
target_pose.yaw = 0;

target_poses = [target_poses, target_pose];

target_pose.x = 0;
target_pose.y = 0;
target_pose.yaw = pi;

target_poses = [target_poses, target_pose];

trim_indices = [1,1];

primitive_dt = 1;
depth = 5;

load('trim_inputs');

n_trims = length(u_trims);

% Transitions (maneuver)
trim_adjacency = eye(n_trims);
for i = 1:n_trims-1
    trim_adjacency(i,i+1) = 1;
end

% Mirror to make symmetric
trim_adjacency = trim_adjacency'+triu(trim_adjacency,1);

motionGraph1 = MotionGraph(model, u_trims, trim_adjacency, primitive_dt);
motionGraph2 = MotionGraph(model, u_trims, trim_adjacency, primitive_dt);

% make motionGraph Tupel
motionGraphList = [motionGraph1 , motionGraph2];

% Combine graphs
combinedGraph = CombinedGraph(motionGraphList);

search_graph = generate_tree(init_poses, target_poses, trim_indices, combinedGraph, depth);