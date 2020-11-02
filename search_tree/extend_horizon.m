function [search_tree, node_list] = extend_horizon(node_list, search_tree, motion_graph, target_poses)
    n_veh = length(motion_graph.motionGraphList);
    for i = 1:length(node_list)
        cur_node = search_tree.Node{node_list(i)};
        while cur_node.depth < h_p
            depth = cur_node.depth + 1;
            xs = zeros(1, n_veh);
            ys = zeros(1, n_veh);
            yaws = zeros(1, n_veh);
            g_values = zeros(1, n_veh);
            h_values = zeros(1, n_veh);
            for j = 1:n_veh
                pose.x = cur_node.xs(j);
                pose.y = cur_node.ys(j);
                pose.yaw = cur_node.yaws(j);
                % Update only if prediction horizon is not at goal
                if ~is_goal(pose, target_poses(j)) 
                    maneuver = motion_graph.motionGraphList(j).maneuvers{cur_node.trims(j), cur_node.trims(j)};
                    [pose.x, pose.y] = translate_global(pose.yaw, pose.x, pose.y, maneuver.dx, maneuver.dy);
                    pose.yaw = pose.yaw + maneuver.dyaw;
                end
                xs(j) = pose.x;
                ys(j) = pose.y;
                yaws(j) = pose.yaw;
                [g_values(j), h_values(j)] = calculate_next_values_time(cur_node.g_values(j), pose, target_poses(j));
            end
            next_node = node(depth, cur_node.trims, xs, ys, yaws, g_values, h_values);
            [search_tree, node_list(i)] = search_tree.addnode(node_list(i), next_node);
            cur_node = next_node;
        end
    end
end

