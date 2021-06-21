function [ scenario ] = get_next_dynamic_obstacles_scenario( scenario,iStep )
% GET_NEXT_DYNAMIC_OBSTACLES_SCENARIO   Filter dynamic obstacles in scenario by current time step and prediction horizon length.

    if ~isempty(scenario.dynamic_obstacle_area)
        scenario.dynamic_obstacle_area = scenario.dynamic_obstacle_area(:,iStep:iStep+scenario.Hp-1);
    end
end

