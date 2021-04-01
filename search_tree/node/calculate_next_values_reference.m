function [g_values, h_values] = calculate_next_values_reference(iter, cur_node, situation_costs, init_poses, target_poses, next_node, motion_graph)
    g_values = cur_node.g_values;
    % iter.reference is of size (scenario.Hp,2,scenario.nVeh)
    x_ref = squeeze(iter.referenceTrajectoryPoints(next_node.depth,1,:))';
    y_ref = squeeze(iter.referenceTrajectoryPoints(next_node.depth,2,:))';
    % Distance to reference trajectory points squared
    d = sum((next_node.xs-x_ref).^2 + (next_node.ys-y_ref).^2);
    g_values = g_values + d;
    h_values = (1 + (next_node.depth == 14)) * cost_to_go(situation_costs, init_poses, target_poses, next_node, motion_graph);
    % TODO adjust ctg
    nVeh = numel(next_node.xs);
    if (next_node.depth < h_p)
        % Distance to every reference trajectory point squared
        % subtract squared distance traveled for every timestep and vehicle
        x_ref = squeeze(iter.referenceTrajectoryPoints(next_node.depth+1:h_p,1,:));
        y_ref = squeeze(iter.referenceTrajectoryPoints(next_node.depth+1:h_p,2,:));
        d_traveled_per_step = dt*v_max;
        time_steps_to_go = h_p - next_node.depth;
        total_steps = nVeh * time_steps_to_go*(time_steps_to_go+1)/2;
        h_values = sum((next_node.xs-x_ref).^2 + (next_node.ys-y_ref).^2) - (d_traveled_per_step*total_steps)^2;
    else
        h_values = 0;
    end
end