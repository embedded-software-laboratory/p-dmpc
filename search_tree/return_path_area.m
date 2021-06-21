function path_area = return_path_area(shapes ,search_tree, node_id)
% RETUEN_PATH_AREA  Return the area occupied by the path.

    % get all occupied areas along the predicted path
    path_area = shapes(:,fliplr(path_to_root(search_tree, node_id)));
    % remove first (empty) one (root node)
    path_area = path_area(:,2:end);
end

