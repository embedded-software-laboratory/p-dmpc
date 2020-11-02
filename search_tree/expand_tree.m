function [node_list, search_tree, cur_id, is_goals] = expand_tree(node_list, search_tree, next_node_id, motion_graph, trim_length, target_poses, visited, counter, is_goals, is_collisionF)
    
    cur_id = counter;

    n_veh = length(motion_graph.motionGraphList);
    
    parent_node = search_tree.Node{next_node_id};
    
    parent_trims = parent_node.trims;
    parent_xs = parent_node.xs;
    parent_ys = parent_node.ys;
    parent_yaws = parent_node.yaws;
    parent_g_values = parent_node.g_values;
    parent_h_values = parent_node.h_values;
    
    trim_tuple = motion_graph.trimTuple;
    trim_tuple_size = length(trim_tuple);
    
    parent_trim_id = tuple2index(parent_trims,trim_length);
    
    for i = 1 : trim_tuple_size
        
        next_trims = parent_trims;
        next_xs = parent_xs;
        next_ys = parent_ys;
        next_yaws = parent_yaws;
        next_g_values = parent_g_values;
        next_h_values = parent_h_values;
        
        node_shapes = {};
        
        next_trims = trim_tuple(i,:);
        
        cur_trim_id = tuple2index(next_trims,trim_length);
        
        % check if maneuver connecting trim tuples exists
        existsManeuver = (motion_graph.transitionMatrix(parent_trim_id,cur_trim_id) == 1);
        
        % if no maneuver exists -> skip
        if ~existsManeuver
        
            continue;
        
        end
        
        for j = 1 : n_veh
            
            if is_goals(j)
                
                % --- TODO: which shape? ---
                [shape_x, shape_y] = translate_global(parent_yaws(j), parent_xs(j), parent_ys(j), [-2.2 2.2 -2.2 2.2],[-0.9 -0.9 0.9 0.9]);
                
                shape = polyshape(shape_x,shape_y,'Simplify',false);
            
                node_shapes{j} = shape;
                continue;
                
            end
            
            maneuver = motion_graph.motionGraphList(j).maneuvers{parent_trims(j), next_trims(j)};
            
            x = parent_xs(j);
            y = parent_ys(j);
            yaw = parent_yaws(j);
            
            new_yaw = yaw + maneuver.dyaw;
            [new_x, new_y] = translate_global(yaw, x, y, maneuver.dx, maneuver.dy);

            next_xs(j) = new_x;
            next_ys(j) = new_y;
            next_yaws(j) = new_yaw;
            
            next_pose.x = new_x;
            next_pose.y = new_y;
            
            last_pose.x = x;
            last_pose.y = y;
            
            [gvalue, hvalue] = calculate_next_values_time(parent_g_values(j), next_pose, target_poses(j));
            
            next_g_values(j) = gvalue;
            next_h_values(j) = hvalue;
            
            [shape_x, shape_y] = translate_global(yaw, x, y, maneuver.area(1,:), maneuver.area(2,:));
            
            shape = polyshape(shape_x,shape_y,'Simplify',false);
            
            node_shapes{j} = shape;
            
        end
        
        next_state.xs = next_xs;
        next_state.ys = next_ys;
        next_state.yaws = next_yaws;
        next_state.trims = next_trims;
        
        % if node was already visited -> skip
        if is_visited(next_state, search_tree, visited, 0.1)
        
            continue;
        
        end
        
        % if node has vehicle collision -> skip
        if is_collisionF(node_shapes)
        
           continue;
        
        end
        
        cur_id = cur_id + 1;

        
        % create new node
        % Add node to existing new graph and connect parent to it
        depth = parent_node.depth + 1;
        node1 = node(depth, next_trims, next_xs, next_ys, next_yaws, next_g_values, next_h_values);
        [search_tree new_node_id] = search_tree.addnode(next_node_id, node1);

        % Update new leaves to be expanded 
        node_list = [node_list, cur_id];
        
        
        cur_poses = [];
        
        for i = 1 : n_veh

            cur_pose.x = next_xs(i);
            cur_pose.y = next_ys(i);

            cur_poses = [cur_poses, cur_pose];

        end
        
        offset = ones(1, n_veh);

        is_goals = is_goal(cur_poses, target_poses);
        
        if sum(is_goals) == n_veh
        
            return;
        
        end
        
    end

end

