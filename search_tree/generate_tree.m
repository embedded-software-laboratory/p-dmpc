% Generates search tree of all feasible trajectories
% init_pose refers to the struct representing the initial pose of our vehicle
% target_pose refers to the struct representing the target pose of our vehicle
% trim refers to the index of the initial trim
% maneuvers refers to the matrix of maneuvers of a motion graph
% search_depth specifies the depth of the created search tree
function [search_tree, leaf_nodes] = generate_tree(init_poses, target_poses, trim_indices, combined_graph, search_depth, is_collisionF, graph_searchF)

    
    n_veh = length(combined_graph.motionGraphList);
    
    trim_length = [];
    
    for i = 1 : n_veh
    
        tmp_length = length(combined_graph.motionGraphList(i).trims);
        
        trim_length = [trim_length, tmp_length];
    
    end

    % Initialize node table values
    id = 1;
    
    g_values = zeros(n_veh,1).';
    
    goal = zeros(n_veh,1);
    
    % high but should be unnecessary
    hvalues = Inf(n_veh,1).';
    
    trims = trim_indices;
    
    xs = [init_poses(1:n_veh).x];
    ys = [init_poses(1:n_veh).y];
    yaws = [init_poses(1:n_veh).yaw];
    
    cur_poses = init_poses;
    
    next_node_id = 0;
    depth = 0;
    
    cur_poses = [];
        
    for i = 1 : n_veh

        cur_pose.x = xs(i);
        cur_pose.y = ys(i);

        cur_poses = [cur_poses, cur_pose];

    end
    
    offset = ones(1, n_veh);
    
    is_goals = is_goal(cur_poses, target_poses, offset);
    
    % Create digraph with root node
    node1 = node(depth, trims, xs, ys, yaws, g_values, hvalues);
    search_tree = tree(node1);
    
    % Array storing ids of nodes that may be expanded
    leaf_nodes = [id];
    
    % Array storing ids of nodes that were visited
    visited_nodes = [];
    
    % loop condition
    tf1 = (search_tree.Node{id}.depth < search_depth);
    tf2 = (sum(is_goals) == n_veh);
    tf3 = isempty(leaf_nodes);
    
    loop = tf1 && ~tf2 && ~tf3;
    
    % Expand leaves of tree until depth or target is reached or until there 
    % are no leaves
    while loop
        
        % get next node for expansion
        next_node_id = graph_searchF(search_tree, leaf_nodes);
        
        % Delete chosen entry from list of expandable nodes
        leaf_nodes(leaf_nodes == next_node_id) = [];
        
        [leaf_nodes, search_tree, id, is_goals] = expand_tree(leaf_nodes, search_tree, next_node_id, combined_graph, trim_length, target_poses, visited_nodes, id, is_goals, is_collisionF);
        
        visited_nodes = [visited_nodes, next_node_id];
        
        % loop condition
        tf1 = (search_tree.Node{id}.depth < search_depth);
        tf2 = (sum(is_goals) == n_veh);
        tf3 = isempty(leaf_nodes);

        loop = tf1 && ~tf2 && ~tf3;
        
    end 

end

