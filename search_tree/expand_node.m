function [expanded_nodes] = expand_node(scenario, iter, cur_node)
    trim_tuple = scenario.combined_graph.trimTuple;
    trim_length = zeros(1, scenario.nVeh);
    for i = 1 : scenario.nVeh
        trim_length(i) = length(scenario.combined_graph.motionGraphList(i).trims);
    end
    cur_trim_id = tuple2index(cur_node.trims,trim_length);
    if cur_node.depth < scenario.Hp
        successor_trim_ids = find(scenario.combined_graph.transitionMatrix(cur_trim_id, :));
    else % h_u <= cur_node.depth < h_p
        successor_trim_ids = cur_trim_id;
    end
    
    nTrims = numel(successor_trim_ids);
    expanded_nodes = cell(1, nTrims);
    for iTrim = 1:nTrims
        id = successor_trim_ids(iTrim);
        expanded_node = cur_node;
        expanded_node.depth = cur_node.depth + 1;
        expanded_node.trims = trim_tuple(id,:);
        for iVeh = 1 : scenario.nVeh

            maneuver = scenario.combined_graph.motionGraphList(iVeh).maneuvers{cur_node.trims(iVeh), expanded_node.trims(iVeh)};

            expanded_node.yaws(iVeh) = cur_node.yaws(iVeh) + maneuver.dyaw;
            [expanded_node.xs(iVeh), expanded_node.ys(iVeh)] = translate_global(cur_node.yaws(iVeh), cur_node.xs(iVeh), cur_node.ys(iVeh), maneuver.dx, maneuver.dy);
        end

        [expanded_node.g_values, expanded_node.h_values, expanded_node.f_value] = calculate_next_values_reference(scenario, iter, cur_node, expanded_node);
        expanded_nodes{iTrim} = expanded_node;
    end
end
