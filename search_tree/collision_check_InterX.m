function [collision] = collision_check_InterX(shape, scenario, iStep)
% COLLISION_CHECK_INTERX Check if the shape collides with the obstacles and
% lanelet boundaries
% 
% INPUT:
%   shape: occupied area of vehicle
% 
%   scenario: instance of the class Scenario
% 
%   iStep: time step of the prediction horizon
% 
% OUTPUT:
%   collision: logical, true if the shape collides with the obstacles and
%   lanelet boundaries
% 

    % get static occupied areas of the considered vehicles
    static_obstacles = scenario.obstacles;
    
    % get predicted occupied areas of the coupling vehicles in the current time step
    [~, n_occupiedAreas_Hp] = size(scenario.dynamic_obstacle_area);
    if iStep <= n_occupiedAreas_Hp
        predicted_occpuied_areas = scenario.dynamic_obstacle_area(:,iStep)';
    else
        predicted_occpuied_areas = {};
    end

    % get reachable sets of the coupling vehicles in the current time step
    [~, n_reachableSets_Hp] = size(scenario.dynamic_obstacle_reachableSets);
    if iStep <= n_reachableSets_Hp
        reachable_sets = scenario.dynamic_obstacle_reachableSets(:,iStep)';
    else
        reachable_sets = {};
    end

    % get lanelet boundary
    lanelet_boundary = scenario.vehicles.lanelet_boundary;
    % add column [nan;nan] to separate left and right boundaries 
    lanelet_boundary = cellfun(@(c)[c,[nan;nan]],lanelet_boundary,'UniformOutput',false); 

    obstacles_shapes = [static_obstacles(:)', predicted_occpuied_areas(:)', reachable_sets(:)'];

    % Repeat the last column to enclose the shape.
    % Add column [nan;nan] to separate different obstacles 
    obstacles_shapes = cellfun(@(c)[c,c(:,1),[nan;nan]],obstacles_shapes,'UniformOutput',false); 
    
    % all obstacles, include static obstacles, dynamic obstacles and lanelet boundaries
    obstacles_all = [obstacles_shapes{:},lanelet_boundary{:}];
    collision = InterX(shape, obstacles_all);
end