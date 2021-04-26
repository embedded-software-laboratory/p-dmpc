function [expanded_nodes, search_tree] = expand_node(scenario, iter, search_tree, node_id, visited_nodes)
    cur_node = search_tree.Node{node_id};
    trim_tuple = scenario.combined_graph.trimTuple;
    % is_goals = is_goal(cur_node, scenario);
    expanded_nodes = [];
    parents = [];
    expand_tmp = {};
    trim_length = zeros(1, scenario.nVeh);
    for i = 1 : scenario.nVeh
        trim_length(i) = length(scenario.combined_graph.motionGraphList(i).trims);
    end
    cur_trim_id = tuple2index(cur_node.trims,trim_length);
    if cur_node.depth < h_u
        successor_trim_ids = find(scenario.combined_graph.transitionMatrix(cur_trim_id, :));
    else % h_u <= cur_node.depth < h_p
        successor_trim_ids = cur_trim_id;
    end
    expanded_node = cur_node;
    expanded_node.depth = cur_node.depth + 1;
    for id = successor_trim_ids
        expanded_node.trims = trim_tuple(id,:);
        visited = true;
        for iVeh = 1 : scenario.nVeh
            % TODO remove goal check
            % if is_goals(iVeh)
            %     continue
            % end

            maneuver = scenario.combined_graph.motionGraphList(iVeh).maneuvers{cur_node.trims(iVeh), expanded_node.trims(iVeh)};

            expanded_node.yaws(iVeh) = cur_node.yaws(iVeh) + maneuver.dyaw;
            [expanded_node.xs(iVeh), expanded_node.ys(iVeh)] = translate_global(cur_node.yaws(iVeh), cur_node.xs(iVeh), cur_node.ys(iVeh), maneuver.dx, maneuver.dy);
            
            
            % Check if similar state was visited until difference occurs
            % if visited
            %     visited = has_visited(i, next_node, search_tree, visited_nodes, 0.1);
            % end
        end
        
        % Similar state was already explored
        %if visited
        %    return
        %end

        [expanded_node.g_values, expanded_node.h_values] = calculate_next_values_reference(scenario, iter, cur_node, expanded_node);
        expand_tmp{end+1} = expanded_node;
    end
    parents = ones(1,numel(expand_tmp))*node_id;
    [search_tree, expanded_nodes] = search_tree.addnnodes(parents, expand_tmp);
end
