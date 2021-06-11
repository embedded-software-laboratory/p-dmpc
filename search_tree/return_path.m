function search_paths = return_path(tree, mpa)
%RETURN_PATH returns the path as cell array to the closest node
    
    n_veh = length(tree.node{1, 1}.trims);
    end_node = tree.n_nodes();
    path = findpath(tree, 1, end_node);
    path_length = length(path);
    search_paths = cell(1, n_veh);
    
    for j = 1 : (path_length - 1)
        cur_node = tree.get(path(j));
        next_node  = tree.get(path(j + 1));
        search_path = path_between(cur_node, next_node, tree, mpa);
        
        for i = 1:n_veh
            search_paths(i) = {[search_paths{i}; search_path{i}]};
        end
    end  
end