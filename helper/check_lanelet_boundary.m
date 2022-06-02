function isValid = check_lanelet_boundary(scenario, boundary, shapes)
% CHECK_LANELET_BOUNDARY   Determine whether vehicle is allowed to intersect boundary for lane change

    isValid = false;
    currentLane = 0;
    successorLane = 0;
    boundary_points_left = boundary{1,1};
    boundary_points_right = boundary{1,2};
    lanes = [];

    %currentLane = match_pose_to_lane(scenario.vehicles.x_position, scenario.vehicles.y_position);

    for i = 1:length(shapes)
        shapeLane = match_pose_to_lane(scenario, shapes(1,i), shapes(2,i));

        if ~ismember(shapeLane, lanes)
            lanes = [lanes; shapeLane];
        end
    end

    %{
    % find current lane based on current boundaries
    for i = 1:length(scenario.lanelet_boundary)
        if (isequal(scenario.lanelet_boundary{1,i}{1,1}, boundary_points_left)) %&& isequal(scenario.lanelet_boundary{1,i}{1,2}, boundary_points_right))
            currentLane = i;
            break
        end
    end
    %}

    %{
    % find successor of current lane in reference path
    if currentLane ~= 0
        for i = 1:(length(scenario.vehicles.lanelets_index)-1)
            if scenario.vehicles.lanelets_index(i) == currentLane
                successorLane = scenario.vehicles.lanelets_index(i+1);
            end
        end

        if successorLane == 0
            disp("error in check_lanelet_boundary: could not find successor of current lane");
            return;
        end
    else
        disp("error in check_lanelet_boundary: could not find current lane");
        return;
    end
    %}

    % if successorLane leads to lane change, allow to cross boundary
    %{
    switch currentLane

        case 3
            if successorLane == 23
                isValid = true;
            end

        case 5
            if successorLane == 10
                isValid = true;
            end

        case 6
            if successorLane == 9
                isValid = true;
            end
        

    end
    %}

    enterCrossingLanes = [23, 5, 9, 10, 11, 12, 74, 55, 67, 68, 65, 66, 81, 100, 83, 101, 87, 88, 29, 48, 41, 42, 39, 40];

    if ismember(lanes(end), enterCrossingLanes)
        isValid = true;
        return;
    end

end