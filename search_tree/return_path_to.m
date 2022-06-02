function search_paths = return_path_to(iNode, tree, scenario)
%RETURN_PATH returns the path as cell array to the closest node

    n_veh = size(tree.x,1);
    tree_path = path_to_root(tree, iNode);
    tree_path = fliplr(tree_path);
    path_length = length(tree_path);
    search_paths = cell(1, n_veh);
    
    for j = 1 : (path_length - 1)
        search_path = path_between(tree_path(j), tree_path(j+1), tree, scenario);
        
        for i = 1:n_veh
            search_paths(i) = {[search_paths{i}; search_path{i}]};
        end
    end  
end
