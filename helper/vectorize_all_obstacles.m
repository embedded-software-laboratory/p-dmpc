function [vehicle_obstacles, lanelet_obstacles] = vectorize_all_obstacles(scenario)
% VECTORIZE_ALL_OBSTACLES This function vectorizes all obstacles, including
% static and dynamic obstacles as well as lanelet boundaries, to 
% a single two-row matrix. The first row is for x-coordinates and the
% second row is for y-coordinates. Shapes are separated by a
% column [nan;nan]. All polygons are assumed to be open; therefore, the
% last column of all polygons will be repeated to enclosed them.
% 
% OUTPUT:
%   vehicle_obstacles: cell array with Hp subcells. Each subcell corresponds to
%   all vehicle obstacles, such as current occupied area, predicted
%   occupied areas and reachable sets, in a certain prediction horizon. 
% 
%   lanelet_obstacles: cell array with Hp subcells. Each subcell
%   corresponds to all lanelet obstacles, such as lanelet boundaries and
%   crossing area of lanelets, in a certain prediction horizon. 
% 
    vehicle_obstacles = cell(1,scenario.Hp);
    lanelet_obstacles = cell(1,scenario.Hp);

    % get static occupied areas of the considered vehicles
    current_occupied_areas = scenario.obstacles;

    % Preprocess crossing areas of lanelets
    % 1. Repeat the last column to enclose the shape.
    % 2. Add column [nan;nan] to separate different obstacles 
    lanelet_crossing_areas = cellfun(@(c)[c,c(:,1),[nan;nan]],scenario.lanelet_crossing_areas,'UniformOutput',false); 

    % Preprocess lanelet boundary
    % 1. Add column [nan;nan] to separate left and right boundaries 
    % Note that lanelet boundaries are considered as curves and thus not
    % needed to be enclosed.
    lanelet_boundary = cellfun(@(c)[c,[nan;nan]],scenario.vehicles.lanelet_boundary,'UniformOutput',false); 

    [~, n_occupiedAreas_Hp] = size(scenario.dynamic_obstacle_area);
    [~, n_reachableSets_Hp] = size(scenario.dynamic_obstacle_reachableSets);

    for iStep = 1:scenario.Hp
        % get predicted occupied areas of the coupling vehicles in the current time step
        if iStep <= n_occupiedAreas_Hp
            predicted_occpuied_areas = scenario.dynamic_obstacle_area(:,iStep)';
        else
            predicted_occpuied_areas = {};
        end
    
        % get reachable sets of the coupling vehicles in the current time step
        if iStep <= n_reachableSets_Hp
            reachable_sets = scenario.dynamic_obstacle_reachableSets(:,iStep)';
        else
            reachable_sets = {};
        end
    
        veh_obstacles_polygons_tmp = [current_occupied_areas(:)', predicted_occpuied_areas(:)', reachable_sets(:)'];

        veh_obstacles_polygons = cellfun(@(c)[c,c(:,1),[nan;nan]],veh_obstacles_polygons_tmp,'UniformOutput',false); 
    
        % all obstacles, include static obstacles, dynamic obstacles and lanelet boundaries
        vehicle_obstacles{iStep} = [veh_obstacles_polygons{:}];
        lanelet_obstacles{iStep} = [lanelet_boundary{:}, lanelet_crossing_areas{:}];
    end
end