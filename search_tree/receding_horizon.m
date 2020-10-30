function [search_tree] = receding_horizon(init_poses, target_poses, trim_indices, combined_graph, search_depth, is_collisionF, graph_searchF)
%RECEDING_HORIZON Explore path to target using a receding horizon 

    % Initialize
    n_veh = length(combined_graph.motionGraphList);
    cur_depth = 1;
    time = 0;
    search_tree = tree(node(0, trim_indices, [init_poses(1:end).x], [init_poses(1:end).y], [init_poses(1:end).yaw], zeros(1,n_veh), Inf(1,n_veh)));
    poses = init_poses;
    trims = trim_indices;

    % Check if the vehicle reached the destination
    offset = ones(1, n_veh);
    is_goals = is_goal(init_poses, target_poses, offset);
    while(sum(is_goals) ~= n_veh)

        % Continue receding horizon search
        [search_window, leaf_nodes] = generate_tree(poses, target_poses, trims, combined_graph, search_depth, is_collisionF, graph_searchF);
        if(~isempty(leaf_nodes))
            node_id = get_next_node_weighted_astar(search_window, leaf_nodes);
        else
            return
        end

        % Check if we already reached our destination
        final_node = search_window.Node{end};
        for i = 1:n_veh
            poses(i).x = final_node.xs(i);
            poses(i).y = final_node.ys(i);
            poses(i).yaw = final_node.yaws(i);
        end

        is_goals = is_goal(poses, target_poses, offset); 
        if(sum(is_goals) == n_veh)
            search_path = findpath(search_window, 1, length(search_window.Node));
            length_path = length(search_path);
            for i = 2:length_path
                [search_tree, cur_depth] = search_tree.addnode(cur_depth, search_window.Node{search_path(i)});

                % Visualize
                visualize_step(search_tree, cur_depth, combined_graph);
            end
            return
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
        [search_tree, cur_depth] = search_tree.addnode(cur_depth, search_window.Node{next_node_id});
        
        % Visualize
        visualize_step(search_tree, cur_depth, combined_graph);

        % Update our loop condition
        is_goals = is_goal(poses, target_poses, offset);
        
    end
end

