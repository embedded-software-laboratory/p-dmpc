function [veh_at_intersection, groups, directed_adjacency, priority_list] = priority_assignment(scenario,iter)
    veh_at_intersection = [];
    switch scenario.options.priority
        case 'topo_priority'
            [groups, directed_adjacency, priority_list] = topo_priority.priority(scenario);

        case 'right_of_way_priority'
            [veh_at_intersection, groups, directed_adjacency, priority_list] = right_of_way_priority().priority(scenario,iter);

        case 'constant_priority'
            [groups, directed_adjacency, priority_list] = constant_priority().priority(scenario);

        case 'random_priority'
            [~, groups, directed_adjacency, priority_list] = random_priority().priority(scenario.options);

        case 'FCA_priority'
            [veh_at_intersection, groups, directed_adjacency, priority_list] = FCA_priority().priority(scenario,iter);

        case 'mixed_traffic_priority'
            obj = mixed_traffic_priority(scenario);
            [groups, directed_adjacency, priority_list] = obj.priority();

        case 'coloring'
            [groups, directed_adjacency, priority_list] = coloring_priority.priority(scenario);

        otherwise
            error('No valid priority assignment strategy given')
    end

end