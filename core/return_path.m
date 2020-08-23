function search_paths = return_path(search_tree)
%RETURN_PATH returns the path to the closest node
    
    nVeh = length(search_tree.Node{1, 1}.trims);
    
    end_node = search_tree.nnodes();
    
    path = findpath(search_tree, 1, end_node);
    
    path_length = length(path);
    
    for i = 1:nVeh
    
        for j = 1 : path_length
            
            cur_node = search_tree.get(path(j));
            
            x = cur_node.xs(i);
            y = cur_node.ys(i);
            yaw = cur_node.yaws(i);
            trim = cur_node.trims(i);

            search_path(j,:) = [x, y, yaw, trim];

        end
        
        search_paths(:,:,i) = search_path;
        
    end

end


