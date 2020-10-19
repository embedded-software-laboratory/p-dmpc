function maneuver_matrix = compute_product_maneuver_matrix(motion_graph_list)

    % initialize product maneuver_matrix
    maneuver_matrix = motion_graph_list(1).transition_matrix;

    n_veh = length(motion_graph_list);
    
    % compute tensor product iteratively
    for i = 2 : n_veh
        maneuver_matrix = kron(maneuver_matrix,motion_graph_list(i).transition_matrix);
    end
    
end

