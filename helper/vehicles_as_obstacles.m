% This function converts a scenario representation based on the scenario
% and iter structs into a different scenario, in which the selected
% vehicles are represented as obstacles along their predicted path
function [scenario_v, iter_v] = vehicles_as_obstacles(scenario, iter, vehicle_filter, info)

    assert( numel(info) == scenario.nVeh)
    assert( length(vehicle_filter) == scenario.nVeh );
    assert( islogical( vehicle_filter ));

    scenario_v = filter_scenario(scenario, ~vehicle_filter);
    iter_v = filter_iter(iter, ~vehicle_filter);
    
    scenario_v.dynamic_obstacles = cell(sum(vehicle_filter),scenario.Hp);

    for iVeh = find(vehicle_filter)
        scenario_v.dynamic_obstacles(iVeh,:) = info{iVeh}.shapes;
    end

end
