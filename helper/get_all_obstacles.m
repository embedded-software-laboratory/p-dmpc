function [vehicle_obstacles, hdv_obstacles] = get_all_obstacles(iter, Hp)
    % GET_ALL_OBSTACLES This function collects all obstacles, including
    % static and dynamic obstacles as well as lanelet boundaries, to
    % an array of two-row matrices. The first row is for x-coordinates and the
    % second row is for y-coordinates. All polygons are assumed to be closed; therefore, the
    % last column of all polygons will be repeated to enclosed them.
    %
    % OUTPUT:
    %   vehicle_obstacles: cell array with Hp subcells. Each subcell corresponds to
    %   all vehicle obstacles in a certain prediction horizon, such as current
    %   occupied area, predicted occupied areas and reachable sets.

    vehicle_obstacles = cell(1, Hp);
    hdv_obstacles = cell(1, Hp);

    % get static occupied areas of the considered vehicles
    current_occupied_areas = iter.obstacles;
    check_closeness(current_occupied_areas)

    % Preprocess intersecting areas of lanelets
    check_closeness(iter.lanelet_crossing_areas)

    [~, n_occupiedAreas_Hp] = size(iter.dynamic_obstacle_area);
    [~, n_hdvSets_Hp] = size(iter.hdv_reachable_sets);
    adjacent_hdv = find(iter.hdv_adjacency);

    for iStep = 1:Hp
        % get predicted occupied areas of the coupling vehicles in the current time step
        if iStep <= n_occupiedAreas_Hp
            predicted_occpuied_areas = iter.dynamic_obstacle_area(:, iStep)';
        else
            predicted_occpuied_areas = {};
        end

        check_closeness(predicted_occpuied_areas)

        % get reachable sets of the coupling vehicles in the current time step
        if iStep <= n_hdvSets_Hp && ~isempty(adjacent_hdv)
            hdv_reachable_sets = iter.hdv_reachable_sets(adjacent_hdv, iStep)';
        else
            hdv_reachable_sets = {};
        end

        check_closeness(hdv_reachable_sets)

        veh_obstacles_polygons_tmp = [current_occupied_areas(:)', predicted_occpuied_areas(:)'];

        % all obstacles, include static obstacles, dynamic obstacles and lanelet boundaries
        vehicle_obstacles{iStep} = veh_obstacles_polygons_tmp;
        hdv_obstacles{iStep} = hdv_reachable_sets(:)';
    end

end

%% local function
function check_closeness(cell_array)
    % This function checks closeness of the input shapes contained in cell
    % array
    if all(cellfun('isempty', cell_array))
        return;
    end

    for i = 1:length(cell_array)
        assert(all(cell_array{i}(:, 1) == cell_array{i}(:, end)))
    end

end
