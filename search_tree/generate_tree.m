function search_tree = generate_tree(init_pose, target_pose, trim, combined_graph)

    n_veh = length(combined_graph.motionGraphList);
    trim_length = length(combined_graph.motionGraphList.trims);

    % Initialize tree
    search_tree = tree(node(0, trim, init_pose.x, init_pose.y, init_pose.yaw, 0, Inf));
    id = 1;
    cur_node = search_tree.Node{id};
    
    % Array storing ids of nodes that may be expanded
    leaf_nodes = [id];
    next_node_id = id;
    
    % Array storing ids of nodes that were visited
    visited_nodes = [];
    
    % Initialize
    finished = is_goal(init_poses, target_poses);
    
    % Expand leaves of tree until depth or target is reached or until there 
    % are no leaves
    while (((search_tree.Node{id}.depth < h_p) && ~finished && ~isempty(leaf_nodes)) || cheaper(search_tree.Node{next_node_id}, search_tree.Node{id}))
               
        n_veh = length(motion_graph.motionGraphList);
        cur_node = search_tree.Node{id};
        next_pose = node2poses(cur_node);

        for i = 1 : trim_length

            next_node = cur_node;
            next_node.trims = motion_graph.trimTuple(i);
            next_trim_id = tuple2index(next_node.trims,trim_length);

            existsManeuver = (motion_graph.transitionMatrix(cur_trim_id,next_trim_id) == 1);
            if ~existsManeuver
                continue;
            end

            maneuver = motion_graph.motionGraphList.maneuvers{cur_node.trims, next_node.trims};

            next_node.yaws = cur_node.yaws + maneuver.dyaw;
            [next_node.xs, next_node.ys] = translate_global(cur_node.yaws, cur_node.xs, cur_node.ys, maneuver.dx, maneuver.dy);

            next_pose.x = next_node.xs;
            next_pose.y = next_node.ys;

            [next_node.g_values, next_node.h_values] = calculate_next_values_time(cur_node.g_values, next_pose, target_pose);

            next_node.depth = cur_node.depth + 1;
            [search_tree, id] = search_tree.addnode(next_id, next_node);
            leaf_nodes = [leaf_nodes, id];
            finished = is_goal(next_pose, target_poses);
        end
    end 
end