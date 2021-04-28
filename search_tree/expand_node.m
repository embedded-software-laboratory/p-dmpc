function [expanded_nodes] = expand_node(scenario, iter, cur_node, idx)
    trim_tuple = scenario.combined_graph.trimTuple;
    trim_length = zeros(1, scenario.nVeh);
    for i = 1 : scenario.nVeh
        trim_length(i) = length(scenario.combined_graph.motionGraphList(i).trims);
    end
    cur_trim_id = tuple2index(cur_node(:,idx.trim),trim_length);
    if cur_node(1,idx.depth) < scenario.Hp
        successor_trim_ids = find(scenario.combined_graph.transitionMatrix(cur_trim_id, :));
    else % h_u <= cur_node.depth < h_p
        successor_trim_ids = cur_trim_id;
    end
    
    nTrims = numel(successor_trim_ids);
    expanded_nodes = cell(1, nTrims);
    for iTrim = 1:nTrims
        id = successor_trim_ids(iTrim);
        expanded_node = cur_node;
        expanded_node(:,idx.depth) = cur_node(:,idx.depth) + 1;
        expanded_node(:,idx.trim) = trim_tuple(id,:);
        for iVeh = 1 : scenario.nVeh

            maneuver = scenario.combined_graph.motionGraphList(iVeh).maneuvers{cur_node(iVeh,idx.trim), expanded_node(iVeh,idx.trim)};

            expanded_node(iVeh,idx.yaw) = cur_node(iVeh,idx.yaw) + maneuver.dyaw;
            [expanded_node(iVeh,idx.x), expanded_node(iVeh,idx.y)] = translate_global(cur_node(iVeh,idx.yaw), cur_node(iVeh,idx.x), cur_node(iVeh,idx.y), maneuver.dx, maneuver.dy);
        end

        expanded_node = calculate_next_values_reference(scenario, iter, expanded_node, idx);
        expanded_nodes{iTrim} = expanded_node;
    end
end