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

%     if scenario.k==12 && scenario.vehicles.ID==10 && iStep==2
%         disp('') % debug
%     end

    shape = [shape, shape(:,1)]; % enclose the shape
    collision = false;
    
    % get static occupied areas of the considered vehicles
    static_obstacles = scenario.obstacles;

    % get intersecting areas of lanelets
    lanelet_intersecting_areas = scenario.lanelet_intersecting_areas;
    
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

%     obstacles_shapes = [static_obstacles(:)', lanelet_intersecting_areas(:)', predicted_occpuied_areas(:)', reachable_sets(:)'];
    for i = 1:length(static_obstacles)
        if InterX(shape,[static_obstacles{i},static_obstacles{i}(:,1)])
            collision = true;
            return
        end
    end
    for j = 1:length(lanelet_intersecting_areas)
        if InterX(shape,[lanelet_intersecting_areas{j},lanelet_intersecting_areas{j}(:,1)])
            collision = true;
            return
        end
    end
    for k = 1:length(predicted_occpuied_areas)
        if InterX(shape,[predicted_occpuied_areas{k},predicted_occpuied_areas{k}(:,1)])
            collision = true;
            return
        end
    end
    for m = 1:length(reachable_sets)
        if InterX(shape,[reachable_sets{m},reachable_sets{m}(:,1)])
            collision = true;
            return
        end
    end
    for n = 1:length(lanelet_boundary)
        if InterX(shape,lanelet_boundary{n})
            collision = true;
            return
        end
    end


