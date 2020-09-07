%% settings
matlab_tree = './matlab-tree/';
assert(logical(exist(matlab_tree, 'dir')));
addpath(matlab_tree);

addpath(genpath(pwd));

warning('off','MATLAB:polyshape:repairedBySimplify')

%% choose model

model = BicycleModel(2.2,2.2);

%% load scenario
% init_poses, target_poses, trim_indices, primitive_dt
load('2_veh_15_coll');

n_veh = length(init_poses);

%% load trim_set

load('trim_set_10_1');

n_trims = length(u_trims);

if n_trims == 6
    trim_indices = ones(1,n_veh)*5;
end
if n_trims == 10
    trim_indices = ones(1,n_veh)*8;
end

%% compute motion graph

tmp_motion_graph = MotionGraph(model, u_trims, trim_adjacency, primitive_dt);

motion_graph_list = [];

for i = 1 : n_veh

    motion_graph_list = [motion_graph_list, tmp_motion_graph];

end

motion_graph = CombinedGraph(motion_graph_list);

%% find path
search_tree = generate_tree(init_poses, target_poses, trim_indices, motion_graph, depth, @is_collision, @get_next_node_weighted_astar);

search_paths = return_path(search_tree, motion_graph);
