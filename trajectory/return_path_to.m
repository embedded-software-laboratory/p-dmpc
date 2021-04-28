function search_paths = return_path_to(node_id, search_tree, motion_graph)
%RETURN_PATH returns the path as cell array to the closest node
    n_veh = length(search_tree.Node{1, 1}(:,search_tree.idx.trim));
    path = pathtoroot(search_tree, node_id);
    path = fliplr(path);
    path_length = length(path);
    search_paths = cell(1, n_veh);
    
    for j = 1 : (path_length - 1)
        cur_node = search_tree.get(path(j));
        next_node  = search_tree.get(path(j + 1));
        search_path = path_between(cur_node, next_node, search_tree, motion_graph);
        
        for i = 1:n_veh
            search_paths(i) = {[search_paths{i}; search_path{i}]};
        end
    end  
end
