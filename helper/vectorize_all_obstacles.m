function [vehicle_obstacles, lanelet_boundary, lanelet_intersecting_areas] = vectorize_all_obstacles(scenario)
% VECTORIZE_ALL_OBSTACLES This function vectorizes all obstacles, including
% static and dynamic obstacles as well as lanelet boundaries, to 
% a single two-row matrix. The first row is for x-coordinates and the
% second row is for y-coordinates. Shapes are separated by a
% column [nan;nan]. All polygons are assumed to be closed; therefore, the
% last column of all polygons will be repeated to enclosed them.
% 
% OUTPUT:
%   vehicle_obstacles: cell array with Hp subcells. Each subcell corresponds to
%   all vehicle obstacles in a certain prediction horizon, such as current
%   occupied area, predicted occupied areas and reachable sets.
% 
%   lanelet_boundary: two-row matrix
% 
%   lanelet_intersecting_areas: cell array with Hp subcells. Each subcell
%   corresponds to intersecting areas of two vehicles' lanelets in a certain
%   prediction horizon.
% 
    vehicle_obstacles = cell(1,scenario.Hp);
    lanelet_intersecting_areas = cell(1,scenario.Hp);

    % get static occupied areas of the considered vehicles
    current_occupied_areas = scenario.obstacles;
    check_closeness(current_occupied_areas)

    % Preprocess intersecting areas of lanelets
    % Add column [nan;nan] to separate different obstacles
    check_closeness(scenario.lanelet_intersecting_areas)
    lanelet_intersecting_areas_tmp = cellfun(@(c)[c,[nan;nan]],scenario.lanelet_intersecting_areas,'UniformOutput',false); 

    % Preprocess lanelet boundary
    % 1. Add column [nan;nan] to separate left and right boundaries 
    % Note that lanelet boundaries are considered as open curves
    lanelet_boundary_tmp = cellfun(@(c)[c,[nan;nan]],scenario.vehicles.lanelet_boundary,'UniformOutput',false); 

    [~, n_occupiedAreas_Hp] = size(scenario.dynamic_obstacle_area);
    [~, n_reachableSets_Hp] = size(scenario.dynamic_obstacle_reachableSets);

    for iStep = 1:scenario.Hp
        % get predicted occupied areas of the coupling vehicles in the current time step
        if iStep <= n_occupiedAreas_Hp
            predicted_occpuied_areas = scenario.dynamic_obstacle_area(:,iStep)';
        else
            predicted_occpuied_areas = {};
        end
        check_closeness(predicted_occpuied_areas)

        % get reachable sets of the coupling vehicles in the current time step
        if iStep <= n_reachableSets_Hp
            reachable_sets = scenario.dynamic_obstacle_reachableSets(:,iStep)';
        else
            reachable_sets = {};
        end
        check_closeness(reachable_sets)
    
        veh_obstacles_polygons_tmp = [current_occupied_areas(:)', predicted_occpuied_areas(:)', reachable_sets(:)'];

        veh_obstacles_polygons = cellfun(@(c)[c,[nan;nan]],veh_obstacles_polygons_tmp,'UniformOutput',false); 
    
        % all obstacles, include static obstacles, dynamic obstacles and lanelet boundaries
        vehicle_obstacles{iStep} = [veh_obstacles_polygons{:}];
        lanelet_boundary = [lanelet_boundary_tmp{:}];
        lanelet_intersecting_areas{iStep} = [lanelet_intersecting_areas_tmp{:}];
    end
end


%% local function
function check_closeness(cell_array)
% This function checks closeness of the input shapes contained in cell
% array
    for i = 1:length(cell_array)
        assert(all(cell_array{i}(:,1)==cell_array{i}(:,end)))
    end
end

