function expanded_node = calculate_next_values_reference(scenario, iter, expanded_node, idx)
    % Cost to come
    % iter.reference is of size (scenario.nVeh,scenario.Hp,2)
    % Distance to reference trajectory points squared
    for iVeh = 1:scenario.nVeh
        expanded_node(iVeh,idx.g) = expanded_node(iVeh,idx.g) ...
            + norm( [expanded_node(iVeh,idx.x) - iter.referenceTrajectoryPoints(iVeh,expanded_node(1,idx.depth),1);...
                     expanded_node(iVeh,idx.y) - iter.referenceTrajectoryPoints(iVeh,expanded_node(1,idx.depth),2)] );                     
    end
    
    % Cost to go
    expanded_node(:,idx.h) = 0;
    if (expanded_node(1,idx.depth) < scenario.Hp)
        % Distance to every reference trajectory point squared
        % subtract squared distance traveled for every timestep and vehicle
        time_steps_to_go = scenario.Hp - expanded_node(1,idx.depth);
        for iVeh = 1:scenario.nVeh
            d_traveled_per_step = scenario.dt*iter.vRef(iVeh);
            for i_t = 1:time_steps_to_go
                expanded_node(iVeh,idx.h) = expanded_node(iVeh,idx.h)...
                    + norm( [expanded_node(iVeh,idx.x)-iter.referenceTrajectoryPoints(iVeh,expanded_node(1,idx.depth)+i_t,1);...
                             expanded_node(iVeh,idx.y)-iter.referenceTrajectoryPoints(iVeh,expanded_node(1,idx.depth)+i_t,2)] ) ...
                    - d_traveled_per_step * i_t;
            end
        end
    end
end