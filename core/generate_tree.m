% Generates search tree of all feasible trajectories
% init_pose refers to the struct representing the initial pose of our vehicle
% target_pose refers to the struct representing the target pose of our vehicle
% trim refers to the index of the initial trim
% maneuvers refers to the matrix of maneuvers of a motion graph
% search_depth specifies the depth of the created search tree
function search_graph = generate_tree(init_poses, target_poses, trim_indices, combined_graph, search_depth)
    
    nVeh = length(combined_graph.motionGraphList);

    % Initialize node table values
    ids =  ones(nVeh,1);
    id = 1;
    
    driven = zeros(nVeh,1);
    
    goal = zeros(nVeh,1);
    
    % high but should be unnecessary
    values = 10000 * ones(nVeh,1)
    
    trims = trim_indices.';
    
    xs = [init_poses(1:nVeh).x].';
    ys = [init_poses(1:nVeh).y].';
    yaws = [init_poses(1:nVeh).yaw].';
    
    cur_poses = init_poses;
    
    parent = 0;
    
    % Create digraph with root node
    search_graph = digraph;
    
    % --- TODO: vector in tables result in two nodes ---
    node = table(ids, values, trims, xs, ys, yaws, driven);
    search_graph = addnode(search_graph, node);
    
    % Array storing ids of nodes that may be expanded
    leaf_nodes = [node.ids(1)];
    
    % Array storing ids of nodes that were visited
    visited_nodes = [node.ids(1)];
    
    % Expand leaves of tree until depth or target is reached or until there 
    % are no leaves
    while (length(shortestpath(search_graph, 1, id)) < search_depth) ...
            && ~is_goal(cur_poses,target_poses,2) ...
            && ~isempty(leaf_nodes)
        
        % get next node for expansion
        parent = get_next_node_astar(search_graph, leaf_nodes);
        
        % Delete chosen entry from list of expandable nodes
        leaf_nodes(leaf_nodes == parent) = [];
        
        [leaf_nodes, search_graph] = expand_tree(leaf_nodes, search_graph, parent, combined_graph, target_poses, visited_nodes);
        
        visited = [visited, parent];
        
        % Reset parent 
        parent = NaN;
        
        id = length(leaf_nodes);
        
        cur_poses = [];
        
        for i = 1 : nVeh
        
            cur_pose.x = search_graph.Nodes{id, 4}(i);
            cur_pose.y = search_graph.Nodes{id, 5}(i);
            
            cur_poses = [cur_poses, cur_pose];
        
        end

    end
    
    h = plot(search_graph);
end

