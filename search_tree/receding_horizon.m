function [search_tree, parents] = receding_horizon(init_poses, target_poses, trim_indices, combined_graph, search_depth, is_collisionF, graph_searchF)
%RECEDING_HORIZON Explore path to target using a receding horizon 

% Initial search
[search_window, leaf_nodes] = generate_tree(init_poses, target_poses, trim_indices, combined_graph, search_depth, is_collisionF, graph_searchF);
node_id = graph_searchF(search_window, leaf_nodes);

% Construct resulting tree
search_tree = tree(search_window.Node{1});
cur_depth = 1;

% Determine next node
search_path = findpath(search_window, 1, node_id);
next_node_id = search_path(2);
poses = [];
n_veh = length(combined_graph.motionGraphList);
for i = 1:n_veh
    pose.x = search_window.Node{next_node_id}.xs(i);
    pose.y = search_window.Node{next_node_id}.ys(i);
    pose.yaw = search_window.Node{next_node_id}.yaws(i);
    poses = [poses, pose];
end
trims = search_window.Node{next_node_id}.trims;

% Add node to tree
search_tree = search_tree.addnode(cur_depth, search_window.Node{next_node_id});
cur_depth = cur_depth + 1;

% Check if the vehicle reached the destination
offset = 2 * ones(1, n_veh);
is_goals = is_goal(poses, target_poses, offset);
while(sum(is_goals) ~= n_veh)
    
    % Continue receding horizon search
    [search_window, leaf_nodes] = generate_tree(poses, target_poses, trims, combined_graph, search_depth, is_collisionF, graph_searchF);
    node_id = get_next_node_weighted_astar(search_window, leaf_nodes);
    
    % Check if we already reached our destination
    final_node = search_window.Node{end};
    for i = 1:n_veh
        poses(i).x = final_node.xs(i);
        poses(i).y = final_node.ys(i);
        poses(i).yaw = final_node.yaws(i);
    end
    
    is_goals = is_goal(poses, target_poses, offset); 
    if(sum(is_goals) == n_veh)
        length_tree = length(search_tree.Node);
        search_tree = search_tree.graft(length_tree, search_window);
        break;
    end
    
    % Determine next node
    search_path = findpath(search_window, 1, node_id);
    next_node_id = search_path(2);
    next_node = search_window.Node{next_node_id};
    for i = 1:n_veh
        poses(i).x = next_node.xs(i);
        poses(i).y = next_node.ys(i);
        poses(i).yaw = next_node.yaws(i);
    end
    trims = search_window.Node{next_node_id}.trims;
    
    % Add node to tree
    search_tree = search_tree.addnode(cur_depth, search_window.Node{next_node_id});
    cur_depth = cur_depth + 1;
    
    % Update our loop condition
    is_goals = is_goal(poses, target_poses, offset);
end

parents = search_tree.Parent(2:end);

end

