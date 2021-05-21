% This function converts a scenario representation based on the scenario
% and iter structs into a different scenario, in which the selected
% vehicles are represented as obstacles along their predicted path
function [scenario_v, iter_v] = vehicles_as_obstacles(scenario, iter, vehicle_filter, vehicle_trajectories)

    % noe prediction x,y,yaw,trim
    ny = 4;

    %assert(all( size([vehicle_trajectories{:,1}]) == [any(vehicle_filter)*scenario.Hp*scenario.tick_per_step (scenario.nVeh-1)*ny]));
    assert( length(vehicle_filter) == scenario.nVeh );
    assert( islogical( vehicle_filter ));

    scenario_v = filter_scenario(scenario, ~vehicle_filter);
    iter_v = filter_iter(iter, ~vehicle_filter);
    
    scenario_v.dynamic_obstacles = cell(sum(vehicle_filter),scenario.Hp);

    for iPred = 1:scenario.Hp
        for iVeh = find(vehicle_filter)
            scenario_v.dynamic_obstacles(iVeh,iPred) = {trajectory_obstacle(scenario,vehicle_trajectories{iVeh,1},iPred)};
        end
    end

end

function obstacle = trajectory_obstacle(scenario,pred_trajectory,iPred)

        t1 = pred_trajectory((scenario.tick_per_step*iPred-1)+1,4);
        t2 = pred_trajectory((scenario.tick_per_step*iPred)+1,4);
        maneuver = scenario.mpa.maneuvers{t1,t2};
        c = cos(pred_trajectory((scenario.tick_per_step*iPred-1)+1,4));
        s = sin(pred_trajectory((scenario.tick_per_step*iPred-1)+1,4));
        x = pred_trajectory((scenario.tick_per_step*iPred-1)+1,1);
        y = pred_trajectory((scenario.tick_per_step*iPred-1)+1,2);
        
        shape_x = c*maneuver.area(1,:) - s*maneuver.area(2,:) + x;
        shape_y = s*maneuver.area(1,:) + c*maneuver.area(2,:) + y;
        obstacle = [shape_x;shape_y];

end