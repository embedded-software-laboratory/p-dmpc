function [node_list, search_tree, cur_id, isgoals] = expand_tree(node_list, search_tree, parent, motion_graph, target_poses, visited, counter, isgoals)
    
    cur_id = counter;

    nVeh = length(motion_graph.motionGraphList);
    
    parent_node = search_tree.get(parent);
    
    parent_trims = parent_node.trims;
    parent_values = parent_node.values;
    parent_xs = parent_node.xs;
    parent_ys = parent_node.ys;
    parent_yaws = parent_node.yaws;
    parent_driven = parent_node.driven;
    
    trim_tuple = motion_graph.trimTuple;
    trim_tuple_size = length(trim_tuple);
    
    parent_trim_id = tuple2index(parent_trims);
    
    for i = 1 : trim_tuple_size
        
        next_trims = parent_trims;
        next_values = parent_values;
        next_xs = parent_xs;
        next_ys = parent_ys;
        next_yaws = parent_yaws;
        next_driven = parent_driven;
        
        node_shapes = {};
        
        next_trims = trim_tuple(i,:);
        
        cur_trim_id = tuple2index(next_trims);
        
        % check if maneuver connecting trim tuples exists
        existsManeuver = (motion_graph.transitionMatrix(parent_trim_id,cur_trim_id) == 1);
        
        % if no maneuver exists -> skip
        if ~existsManeuver
        
            continue;
        
        end
        
        for j = 1 : nVeh
            
            if isgoals(j)
                
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
            
            newYaw = yaw + maneuver.dyaw;
            [newX, newY] = translate_global(yaw, x, y, maneuver.dx, maneuver.dy);

            next_xs(j) = newX;
            next_ys(j) = newY;
            next_yaws(j) = newYaw;
            
            next_pose.x = newX;
            next_pose.y = newY;
            
            man_driven = sqrt( (newX - x)^2 + (newY - y)^2 );
            
            next_driven(j) = parent_driven(j) + man_driven;
            
            next_values(j) = next_driven(j) + euclidean_distance(next_pose, target_poses(j));
            
            [shape_x, shape_y] = translate_global(yaw, x, y, maneuver.area(1,:), maneuver.area(2,:));
            
            shape = polyshape(shape_x,shape_y,'Simplify',false);
            
            node_shapes{j} = shape;
            
        end
        
        % if node has vehicle collision -> skip
        if is_collision(node_shapes)
        
            continue;
        
        end
        
        next_state.xs = next_xs;
        next_state.ys = next_ys;
        next_state.yaws = next_yaws;
        next_state.trims = next_trims;
        
        % if node was already visited -> skip
        if is_visited(next_state, search_tree, visited, 0.1)
        
            continue;
        
        end
        
        cur_id = cur_id + 1;

        
        % create new node
        % Add node to existing new graph and connect parent to it
        node1 = node(cur_id, parent, next_values, next_trims, next_xs, next_ys, next_yaws, next_driven);
        [search_tree new_node_id] = search_tree.addnode(parent, node1);

        % Update new leaves to be expanded 
        node_list = [node_list, node1.id];
        
        
        cur_poses = [];
        
        for i = 1 : nVeh

            cur_pose.x = next_xs(i);
            cur_pose.y = next_ys(i);

            cur_poses = [cur_poses, cur_pose];

        end
        
        offset = ones(1, nVeh).*3;

        isgoals = is_goal(cur_poses, target_poses, offset);
        
        if sum(isgoals) == nVeh
        
            return;
        
        end
        
    end

end

