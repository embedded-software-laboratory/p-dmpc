function expanded_node = calculate_next_values_reference(scenario, iter, expanded_node, idx)
    % Cost to come
    % iter.reference is of size (scenario.Hp,2,scenario.nVeh)
    x_ref_now = reshape(iter.referenceTrajectoryPoints(expanded_node(1,idx.depth),1,:),scenario.nVeh,1);
    y_ref_now = reshape(iter.referenceTrajectoryPoints(expanded_node(1,idx.depth),2,:),scenario.nVeh,1);
    % Distance to reference trajectory points squared
    for iVeh = 1:scenario.nVeh
        expanded_node(iVeh,idx.g) = expanded_node(iVeh,idx.g) ...
            + norm( [expanded_node(iVeh,idx.x) - x_ref_now(iVeh);...
                     expanded_node(iVeh,idx.y) - y_ref_now(iVeh)] );                     
    end
    
    % Cost to go
    expanded_node(:,idx.h) = 0;
    if (expanded_node(1,idx.depth) < scenario.Hp)
        % Distance to every reference trajectory point squared
        % subtract squared distance traveled for every timestep and vehicle
        time_steps_to_go = scenario.Hp - expanded_node(1,idx.depth);
        x_ref = reshape(iter.referenceTrajectoryPoints(expanded_node(1,idx.depth)+1:scenario.Hp,1,:), time_steps_to_go, scenario.nVeh);
        y_ref = reshape(iter.referenceTrajectoryPoints(expanded_node(1,idx.depth)+1:scenario.Hp,2,:), time_steps_to_go, scenario.nVeh);
        for iVeh = 1:scenario.nVeh
            d_traveled_per_step = scenario.dt*iter.vRef(iVeh);
            for i_t = 1:time_steps_to_go
                expanded_node(iVeh,idx.h) = expanded_node(iVeh,idx.h)...
                    + norm( [expanded_node(iVeh,idx.x)-x_ref(i_t,iVeh);...
                             expanded_node(iVeh,idx.y)-y_ref(i_t,iVeh)] ) ...
                    - d_traveled_per_step * i_t;
            end
        end
    end
end