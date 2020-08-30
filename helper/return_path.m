function search_paths = return_path(search_tree, motion_graph)
%RETURN_PATH returns the path to the closest node
    
    n_veh = length(search_tree.Node{1, 1}.trims);
    
    end_node = search_tree.nnodes();
    
    path = findpath(search_tree, 1, end_node);
    
    path_length = length(path);
    
    for i = 1:n_veh
    
        for j = 1 : (path_length - 1)
            
            cur_node = search_tree.get(path(j));
            next_node  = search_tree.get(path(j + 1));
            
            cur_trim = cur_node.trims(i);
            next_trim = next_node.trims(i);
            
            maneuver = motion_graph.motionGraphList(i).maneuvers{cur_trim, next_trim};
            
            x = cur_node.xs(i);
            y = cur_node.ys(i);
            yaw = cur_node.yaws(i);
            
            xs = maneuver.xs;
            ys = maneuver.ys;
            yaws = maneuver.yaws + yaw;       
            
            length_maneuver = length(xs);
            trims = cur_trim * ones(length_maneuver, 1);
            
            [xs, ys] = translate_global(yaw, x, y, xs, ys);
            
            xs = xs.';
            ys = ys.';
            
            start_index = 1 + (j - 1)*length_maneuver;
            end_index = start_index - 1 + length_maneuver;
            
            search_path(start_index:end_index,:) = [xs, ys, yaws, trims];

        end
        
        search_paths(:,:,i) = search_path;
        
    end

end


