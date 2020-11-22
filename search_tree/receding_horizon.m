function video = receding_horizon(init_poses, target_poses, trim_indices, combined_graph, video)
%RECEDING_HORIZON Explore path to target using a receding horizon 

    % Initialize
    n_veh = length(combined_graph.motionGraphList);
    p = gobjects(1, n_veh);
    g = gobjects(1, n_veh);
    for i = 1:n_veh
        cur_color = vehColor(i);
        p(i) = plot(init_poses(i).x, init_poses(i).y, '-','Color', cur_color);
        p(i).Color(4) = 0.5;
        g(i) = plot(init_poses(i).x, init_poses(i).y, 'o','Color', cur_color, 'MarkerSize',3,'MarkerFaceColor', cur_color);
        g(i).Color(4) = 0.5;
    end
    cur_depth = 1;
    cur_node = node(0, trim_indices, [init_poses(1:end).x], [init_poses(1:end).y], [init_poses(1:end).yaw], zeros(1,n_veh), zeros(1,n_veh));
    search_tree = tree(cur_node);
    cur_poses = init_poses;
    trims = trim_indices;

    % Check if the vehicle reached the destination
    is_goals = is_goal(init_poses, target_poses);
    while(sum(is_goals) ~= n_veh)
               
        % Continue receding horizon search
        [search_window, leaf_nodes] = generate_horizon(init_poses, target_poses, cur_node, trims, combined_graph, video);
               
        if(~isempty(leaf_nodes))
            node_id = get_next_node(search_window, leaf_nodes);
        else
            return
        end
        
        % Visualize Horizon
        path_cell = return_path_to(node_id, search_window, combined_graph);
        for i = 1:n_veh
            path = path_cell{i};
            p(i).XData = path(:,1);
            p(i).YData = path(:,2);
        end
        drawnow;
        
        % Visualize Path before addition to properly display horizon
        visualize_step(search_tree, cur_depth, combined_graph);
        frame = getframe(gcf);
        writeVideo(video, frame);

        % Check if we already reached our destination
        final_node = search_window.Node{end};
        for i = 1:n_veh
            cur_poses(i).x = final_node.xs(i);
            cur_poses(i).y = final_node.ys(i);
            cur_poses(i).yaw = final_node.yaws(i);
        end

        is_goals = is_goal(cur_poses, target_poses); 
        if(sum(is_goals) == n_veh)
            search_path = findpath(search_window, 1, length(search_window.Node));
            length_path = length(search_path);
            for i = 1:length_path
                [search_tree, cur_depth] = search_tree.addnode(cur_depth, search_window.Node{search_path(i)});

                % Visualize
                path_cell = return_path(search_window, combined_graph);
                for j = 1:n_veh
                    path = path_cell{j};
                    p(j).XData = path(:,1);
                    p(j).YData = path(:,2);
                end
                drawnow;
                visualize_step(search_tree, cur_depth, combined_graph);
                frame = getframe(gcf);
                writeVideo(video, frame);
            end
            return
        end

        % Determine next node
        search_path = findpath(search_window, 1, node_id);
        next_node_id = search_path(2);
        cur_node = search_window.Node{next_node_id};
        for i = 1:n_veh
            cur_poses(i).x = cur_node.xs(i);
            cur_poses(i).y = cur_node.ys(i);
            cur_poses(i).yaw = cur_node.yaws(i);
        end
        trims = search_window.Node{next_node_id}.trims;

        % Add node to tree
        [search_tree, cur_depth] = search_tree.addnode(cur_depth, cur_node);

        % Update our loop condition
        is_goals = is_goal(cur_poses, target_poses);
    end
end