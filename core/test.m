% Add modules to path
addpath(genpath(pwd));

% Choose Model
model = BicycleModel(2.2,2.2);

init_state.x = 0;
init_state.y = 0;
init_state.yaw = 0;

target.x = 10;
target.y = 10;
target.yaw = 0;

init_trim = 1;

primitive_dt = 1;
depth = 2;

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

search_tree = generate_tree(init_state, target, init_trim, motionGraph.maneuvers, depth, primitive_dt)