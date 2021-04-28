function [g_values, h_values] = calculate_next_values_reference(scenario, iter, cur_node, next_node)
    
    % Preallocate outputs
    g_values = cur_node.g_values;
    h_values = zeros(scenario.nVeh,1);
    % Cost to come
    % iter.reference is of size (scenario.Hp,2,scenario.nVeh)
    x_ref_now = reshape(iter.referenceTrajectoryPoints(next_node.depth,1,:),scenario.nVeh,1);
    y_ref_now = reshape(iter.referenceTrajectoryPoints(next_node.depth,2,:),scenario.nVeh,1);
    % Distance to reference trajectory points squared
    for iVeh = 1:scenario.nVeh
        g_values(iVeh) = g_values(iVeh) ...
            + norm( [next_node.xs(iVeh) - x_ref_now(iVeh);...
                     next_node.ys(iVeh) - y_ref_now(iVeh)] );                     
    end
    
    % Cost to go
    if (next_node.depth < scenario.Hp)
        % Distance to every reference trajectory point squared
        % subtract squared distance traveled for every timestep and vehicle
        time_steps_to_go = scenario.Hp - next_node.depth;
        x_ref = reshape(iter.referenceTrajectoryPoints(next_node.depth+1:scenario.Hp,1,:), time_steps_to_go, scenario.nVeh);
        y_ref = reshape(iter.referenceTrajectoryPoints(next_node.depth+1:scenario.Hp,2,:), time_steps_to_go, scenario.nVeh);
        for iVeh = 1:scenario.nVeh
            d_traveled_per_step = scenario.dt*iter.vRef(iVeh);
            for i_t = 1:time_steps_to_go
                h_values(iVeh) = h_values(iVeh) ...
                    + norm( [next_node.xs(iVeh)-x_ref(i_t,iVeh);...
                             next_node.ys(iVeh)-y_ref(i_t,iVeh)] ) ...
                    - d_traveled_per_step * double(i_t);
            end
        end
    end
end