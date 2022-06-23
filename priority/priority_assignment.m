function [veh_at_intersection, groups, directed_adjacency, priority_list] = priority_assignment(scenario,iter)
    right_of_way = false;
    switch scenario.priority_option
        case 'topo_priority' 
            [groups, directed_adjacency, priority_list] = topo_priority().priority(scenario); 
            veh_at_intersection = [];
    
        case 'right_of_way_priority' 
            right_of_way = true;
            [veh_at_intersection, groups, directed_adjacency, priority_list] = right_of_way_priority().priority(scenario,iter);  

        case 'constant_priority' 
            [groups, directed_adjacency, priority_list] = constant_priority().priority(scenario); 
            veh_at_intersection = [];
    
        case 'random_priority'  
            [groups, directed_adjacency, priority_list] = random_priority().priority(scenario); 
            veh_at_intersection = [];
    
        case 'FCA_priority' 
            [veh_at_intersection, groups, directed_adjacency, priority_list] = FCA_priority().priority(scenario,iter);
       
        case 'mixed_traffic_priority'
            obj = mixed_traffic_priority(scenario);
            [groups, directed_adjacency, priority_list] = obj.priority(); 
            veh_at_intersection = [];
    end

end