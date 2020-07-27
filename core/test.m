% Choose Model
model = BicycleModel(2.2,2.2);

init_pose.x = 0;
init_pose.y = 0;
init_pose.yaw = 0;

target_pose.x = 10;
target_pose.y = 10;
target_pose.yaw = 0;

trim_index = 1;

primitive_dt = 1;
depth = 30;

load('trim_inputs');

n_trims = length(u_trims);

% Transitions (maneuver)
trim_adjacency = eye(n_trims);
for i = 1:n_trims-1
    trim_adjacency(i,i+1) = 1;
end

% Mirror to make symmetric
trim_adjacency = trim_adjacency'+triu(trim_adjacency,1);

motionGraph = MotionGraph(model, u_trims, trim_adjacency, primitive_dt);

search_graph = generate_tree(init_pose, target_pose, trim_index, motionGraph.maneuvers, depth);