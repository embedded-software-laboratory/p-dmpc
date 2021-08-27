function search_paths = return_path_to(iNode, tree, mpa)
%RETURN_PATH returns the path as cell array to the closest node

    n_veh = size(tree.x,1);
    path = path_to_root(tree, iNode);
    path = fliplr(path);
    path_length = length(path);
    search_paths = cell(1, n_veh);
    
    for j = 1 : (path_length - 1)
        search_path = path_between(j, j+1, tree, mpa);
        
        for i = 1:n_veh
            search_paths(i) = {[search_paths{i}; search_path{i}]};
        end
    end  
end
