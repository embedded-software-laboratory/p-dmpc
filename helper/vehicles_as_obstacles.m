% This function converts a scenario representation based on the scenario
% and iter structs into a different scenario, in which the selected
% vehicles are represented as obstacles along their predicted path
function [scenario_v, iter_v] = vehicles_as_obstacles(scenario, iter, vehicle_filter, shapes)

    assert( size(shapes,1) == scenario.nVeh-1 )
    assert( length(vehicle_filter) == scenario.nVeh );
    assert( islogical( vehicle_filter ));

    scenario_v = filter_scenario(scenario, ~vehicle_filter);
    iter_v = filter_iter(iter, ~vehicle_filter);
    
    nDynObst = size(scenario_v.dynamic_obstacle_area, 1);
    
    scenario_v.dynamic_obstacle_area = [scenario_v.dynamic_obstacle_area; cell(sum(vehicle_filter),scenario.Hp)];

    for iVeh = 1:sum( vehicle_filter )
        scenario_v.dynamic_obstacle_area(nDynObst+iVeh,:) = shapes(iVeh,:);
    end

end
