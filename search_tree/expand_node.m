function [expanded_nodes] = expand_node(scenario, iter, cur_node, idx)
    trim_tuple = scenario.mpa.trim_tuple;
    trim_length = length(scenario.mpa.trims)*ones(1, scenario.nVeh);
    cur_trim_id = tuple2index(cur_node(:,idx.trim),trim_length);
    if cur_node(1,idx.depth) < scenario.Hp
        successor_trim_ids = find(scenario.mpa.transition_matrix(cur_trim_id, :));
    else % h_u <= cur_node.depth < h_p
        successor_trim_ids = cur_trim_id;
    end
    
    nTrims = numel(successor_trim_ids);
    expanded_nodes = cell(1, nTrims);
    % Preallocate maneuver
    for iTrim = 1:nTrims
        id = successor_trim_ids(iTrim);
        expanded_node = cur_node;
        expanded_node(:,idx.depth) = cur_node(:,idx.depth) + 1;
        expanded_node(:,idx.trim) = trim_tuple(id,:);
        for iVeh = 1 : scenario.nVeh
            itrim1 = cur_node(iVeh,idx.trim);
            itrim2 = expanded_node(iVeh,idx.trim);
            maneuver = scenario.mpa.maneuvers{itrim1, itrim2};
            c = cos(cur_node(iVeh,idx.yaw));
            s = sin(cur_node(iVeh,idx.yaw));

            expanded_node(iVeh,idx.x) = c*maneuver.dx - s*maneuver.dy + cur_node(iVeh,idx.x);
            expanded_node(iVeh,idx.y) = s*maneuver.dx + c*maneuver.dy + cur_node(iVeh,idx.y);
            expanded_node(iVeh,idx.yaw) = cur_node(iVeh,idx.yaw) + maneuver.dyaw;
        end

        % expanded_node = calculate_next_values_reference(scenario, iter, expanded_node, idx);
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
                d_traveled_per_step = scenario.dt*iter.vRef;
                for i_t = 1:time_steps_to_go
                    expanded_node(iVeh,idx.h) = expanded_node(iVeh,idx.h)...
                        + norm( [expanded_node(iVeh,idx.x)-iter.referenceTrajectoryPoints(iVeh,expanded_node(1,idx.depth)+i_t,1);...
                                expanded_node(iVeh,idx.y)-iter.referenceTrajectoryPoints(iVeh,expanded_node(1,idx.depth)+i_t,2)] ) ...
                        - d_traveled_per_step * i_t;
                end
            end
        end


        expanded_nodes{iTrim} = expanded_node;
    end
end