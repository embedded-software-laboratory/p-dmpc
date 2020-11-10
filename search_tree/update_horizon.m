function [leaf_nodes, search_tree, max_id, is_goals] = update_horizon(cur_node, next_node, leaf_nodes, search_tree, next_id, motion_graph, init_poses, target_poses, visited_nodes, max_id, is_goals)
        
    n_veh = length(motion_graph.motionGraphList);
    next_poses = node2poses(cur_node);
    visited = true;
    
    for j = 1 : n_veh
            
        if is_goals(j)
            [shape_x, shape_y] = translate_global(cur_node.yaws(j), cur_node.xs(j), cur_node.ys(j), [-2.2 2.2 -2.2 2.2],[-0.9 -0.9 0.9 0.9]);
            shape = polyshape(shape_x,shape_y,'Simplify',false);
            node_shapes{j} = shape;
            continue
        end

        maneuver = motion_graph.motionGraphList(j).maneuvers{cur_node.trims(j), next_node.trims(j)};

        next_node.yaws(j) = cur_node.yaws(j) + maneuver.dyaw;
        [next_node.xs(j), next_node.ys(j)] = translate_global(cur_node.yaws(j), cur_node.xs(j), cur_node.ys(j), maneuver.dx, maneuver.dy);

        next_poses(j).x = next_node.xs(j);
        next_poses(j).y = next_node.ys(j);

        [next_node.g_values(j), next_node.h_values(j)] = calculate_next_values_reference(cur_node.g_values(j), init_poses(j), target_poses(j), next_poses(j));

        [shape_x, shape_y] = translate_global(next_node.yaws(j), next_node.xs(j), next_node.ys(j), maneuver.area(1,:), maneuver.area(2,:));
        shape = polyshape(shape_x,shape_y,'Simplify',false);
        node_shapes{j} = shape;
        
        % Abort update if collision is detected
        if collision_with(j, node_shapes)
                    return
        end
        
        % Check if similar state was visited until difference occurs
        if visited
            visited = has_visited(j, next_node, search_tree, visited_nodes, 0.1);
        end
    end
    
    % Similar state was already explored
    if visited
        return
    end

    next_node.depth = cur_node.depth + 1;
    [search_tree, max_id] = search_tree.addnode(next_id, next_node);
    leaf_nodes = [leaf_nodes, max_id];

    is_goals = is_goal(next_poses, target_poses);
end


