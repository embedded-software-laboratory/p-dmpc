function index = match_pose_to_lane(scenario, vehicle_x, vehicle_y)
    % find corresponding lane to given pose

    commonroad_data = scenario.commonroad_data;
    min_distance = 32768;

    for i = 1:length(commonroad_data.lanelet)
        centerline_x = scenario.lanelets{ i }(:,LaneletInfo.cx);
        centerline_y = scenario.lanelets{ i }(:,LaneletInfo.cy);
        centerline = [centerline_x(1:end),centerline_y(1:end)];

        for j = 1:length(centerline)
            % coordinates have to be recalculated for CPM Lab Mode
            dx = centerline(j) - vehicle_x;
            dy = centerline(j,2) - vehicle_y;
            
            squared_distance = dx * dx + dy * dy;

            if squared_distance < min_distance
                min_distance = squared_distance;
                index = commonroad_data.lanelet(i).idAttribute;
            end
        end

    end


end