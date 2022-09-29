function [ scenario ] = coupling_based_on_reachable_sets(scenario, iter)
    traffic_info = TrafficInfo(scenario,iter);
    adjacency = traffic_info.coupling_weights;
    adjacency = (adjacency + adjacency.') ~= 0
    k = scenario.k;
    scenario.coupling_info = traffic_info.coupling_info;
    scenario.adjacency(:,:,k) = adjacency;
    scenario.semi_adjacency(:,:,k) = adjacency;
end

