function trajectory = return_path_to(iNode, tree)
    %RETURN_PATH returns the path as cell array to the closest node

    n_vehicles = tree.number_of_vehicles();
    tree_path = path_to_root(tree, iNode);
    tree_path = fliplr(tree_path);
    path_length = length(tree_path);

    % predicted trajectory, (x, y, yaw) x prediction horizon + 1 x n_vehicles
    Hp = path_length - 1;
    trajectory = nan(3, Hp, n_vehicles);

    for i_node = 2:path_length
        i_step = i_node - 1;

        for i_vehicle = 1:n_vehicles
            node_index = tree_path(i_node);
            trajectory(:, i_step, i_vehicle) = [
                                                tree.get_x(i_vehicle, node_index)
                                                tree.get_y(i_vehicle, node_index)
                                                tree.get_yaw(i_vehicle, node_index)
                                                ];
        end

    end

end
