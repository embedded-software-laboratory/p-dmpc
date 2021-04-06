function [g_values, h_values] = calculate_next_values_reference(scenario, iter, cur_node, next_node)
    
    nVeh = numel(next_node.xs);
    g_values = cur_node.g_values;
    % iter.reference is of size (scenario.Hp,2,scenario.nVeh)
    x_ref_now = reshape(iter.referenceTrajectoryPoints(next_node.depth,1,:),nVeh,1);
    y_ref_now = reshape(iter.referenceTrajectoryPoints(next_node.depth,2,:),nVeh,1);
    % Distance to reference trajectory points squared
    g_values = g_values + euclidean_distance(...
        next_node.xs, next_node.ys,...
        x_ref_now, y_ref_now...
    );
    
%     h_values = (1 + (next_node.depth == 14)) * cost_to_go(situation_costs, init_poses, target_poses, next_node, motion_graph);
    % TODO adjust ctg
    if (next_node.depth < h_p)
        % Distance to every reference trajectory point squared
        % subtract squared distance traveled for every timestep and vehicle
        time_steps_to_go = h_p - next_node.depth;
        x_ref = reshape(iter.referenceTrajectoryPoints(next_node.depth+1:h_p,1,:), time_steps_to_go, nVeh);
        y_ref = reshape(iter.referenceTrajectoryPoints(next_node.depth+1:h_p,2,:), time_steps_to_go, nVeh);
        d_traveled_per_step = dt*v_max;
        h_values = zeros(nVeh,1);
        for iVeh = 1:nVeh
            for i_t = 1:time_steps_to_go
                % TODO Use euclidean distance
                h_values(iVeh) = h_values(iVeh) ...
                    + norm( [next_node.xs(iVeh)-x_ref(i_t,iVeh);...
                             next_node.ys(iVeh)-y_ref(i_t,iVeh)] ) ...
                    - i_t * d_traveled_per_step;
            end
        end
%         h_values = sum((next_node.xs-x_ref).^2 + (next_node.ys-y_ref).^2) - (d_traveled_per_step*total_steps)^2;
    else
        h_values = 0;
    end
end