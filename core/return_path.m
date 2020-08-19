function search_paths = return_path(search_tree)
%RETURN_PATH returns the path to the closest node
    
    nVeh = length(search_tree.Node{1, 1}.values);
    
    min_value = Inf(1, nVeh);
    min_id = zeros(1, nVeh);
    
    % Find the node closest to the target for every vehicle
    for i = 1:nVeh
        
        for j = 1:length(search_tree.Node)
            
            cur_node = search_tree.get(j);
            
            if cur_node.values(i) < min_value(i)
                min_value(i) = cur_node.values(i);
                min_id(i) = j;
            end
            
        end
        
    end
    
    max_length_path = 0;
    for i = 1:nVeh
        path = findpath(search_tree, 1, min_id(i));
        cur_length_path = length(path); 
        max_length_path = max(max_length_path, cur_length_path);
    end
    
    search_paths = zeros(max_length_path, 4, nVeh);
    
    for i = 1:nVeh
        
    path = findpath(search_tree, 1, min_id(i));
    search_path = zeros(max_length_path, 4);  
    
        for j = 1:max_length_path
            
            cur_length_path = length(path); 
            k = min(j, cur_length_path);
            
            cur_node = search_tree.get(path(k));
            
            x = cur_node.xs(i);
            y = cur_node.ys(i);
            yaw = cur_node.yaws(i);
            trim = cur_node.trims(i);

            search_path(j,:) = [x, y, yaw, trim];

        end
        
        search_paths(:,:,i) = search_path;
        
    end

end


