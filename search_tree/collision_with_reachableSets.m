function collision = collision_with_reachableSets(index, shapes, scenario, iStep)
% COLLISION_WITH    Determine whether position has a collision with the reachable sets 
% of coupled vehicles with higher priorities in other groups.

    collision = false;
    
    % check if collides with the reachable sets of coupled vehicles with
    % higher priorities in other groups
    if ~isempty(scenario.dynamic_obstacle_reachableSets)
%         for i = 1:size(scenario.dynamic_obstacle_reachableSets,1)
        for i = 1:size(scenario.dynamic_obstacle_reachableSets,1)
            if intersect_sat(shapes{index},scenario.dynamic_obstacle_reachableSets{i,iStep}) 
                collision = true;
                return;
            end
        end
    end
    
end
