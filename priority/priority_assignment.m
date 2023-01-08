function [veh_at_intersection, groups, directed_adjacency, priority_list] = priority_assignment(scenario,iter)
    veh_at_intersection = [];
    switch scenario.options.priority
        case 'right_of_way_priority'
            [veh_at_intersection, groups, directed_adjacency, priority_list] = right_of_way_priority().priority(scenario,iter);

        case 'constant_priority'
            [groups, directed_adjacency, priority_list] = constant_priority().priority(scenario, iter);

        case 'random_priority'
            [groups, directed_adjacency, priority_list] = random_priority().priority(scenario);

        case 'FCA_priority'
            [veh_at_intersection, groups, directed_adjacency, priority_list] = FCA_priority().priority(scenario,iter);

        case 'mixed_traffic_priority'
            obj = mixed_traffic_priority(scenario, iter);
            [groups, directed_adjacency, priority_list] = obj.priority();

        case 'coloring_priority'
            [groups, directed_adjacency, priority_list] = coloring_priority().priority(scenario);

        otherwise
            error('No valid priority assignment strategy given')
    end
    aTable = struct2table(groups); 
 disp(aTable);
end