function [leaf_nodes, search_tree, max_id, is_goals] = expand_horizon(leaf_nodes, search_tree, next_id, motion_graph, trim_length, target_poses, visited, max_id, is_goals)
    
    n_veh = length(motion_graph.motionGraphList);
    cur_node = search_tree.Node{next_id};
    next_node = cur_node;
    next_poses = node2poses(cur_node);
    trim_tuple = motion_graph.trimTuple;
    trim_tuple_size = length(trim_tuple);
    cur_trim_id = tuple2index(cur_node.trims,trim_length);
    node_shapes = {};
    
    for i = 1 : trim_tuple_size

        next_node.trims = trim_tuple(i,:);
        next_trim_id = tuple2index(next_node.trims,trim_length);
        
        existsManeuver = (motion_graph.transitionMatrix(cur_trim_id,next_trim_id) == 1);
        if ~existsManeuver
            continue;
        end
        
        for j = 1 : n_veh
            
            if is_goals(j)
                [shape_x, shape_y] = translate_global(cur_node.yaws(j), cur_node.xs(j), cur_node.ys(j), [-2.2 2.2 -2.2 2.2],[-0.9 -0.9 0.9 0.9]);
                shape = polyshape(shape_x,shape_y,'Simplify',false);
                node_shapes{j} = shape;
                continue;
            end
            
            maneuver = motion_graph.motionGraphList(j).maneuvers{cur_node.trims(j), next_node.trims(j)};
                        
            next_node.yaws(j) = cur_node.yaws(j) + maneuver.dyaw;
            [next_node.xs(j), next_node.ys(j)] = translate_global(cur_node.yaws(j), cur_node.xs(j), cur_node.ys(j), maneuver.dx, maneuver.dy);
            
            next_poses(j).x = next_node.xs(j);
            next_poses(j).y = next_node.ys(j);
            
            [next_node.g_values(j), next_node.h_values(j)] = calculate_next_values_time(cur_node.g_values(j), next_poses(j), target_poses(j));
            
            [shape_x, shape_y] = translate_global(next_node.yaws(j), next_node.xs(j), next_node.ys(j), maneuver.area(1,:), maneuver.area(2,:));
            shape = polyshape(shape_x,shape_y,'Simplify',false);
            node_shapes{j} = shape;
        end
        
        if is_visited(next_node, search_tree, visited, 0.1)
            continue;
        end
        
        if is_collision(node_shapes)
           continue;
        end
        
        next_node.depth = cur_node.depth + 1;
        [search_tree, max_id] = search_tree.addnode(next_id, next_node);
        leaf_nodes = [leaf_nodes, max_id];
              
        is_goals = is_goal(next_poses, target_poses);
        if sum(is_goals) == n_veh
            return;
        end
    end
end


