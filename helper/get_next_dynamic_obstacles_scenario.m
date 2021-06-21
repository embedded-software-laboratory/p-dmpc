function [ scenario ] = get_next_dynamic_obstacles_scenario( scenario,iStep )
if ~isempty(scenario.dynamic_obstacle_area)
    scenario.dynamic_obstacle_area = scenario.dynamic_obstacle_area(:,iStep:iStep+scenario.Hp-1);
end
end

