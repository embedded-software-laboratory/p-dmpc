function index = match_pose_to_lane(scenario, vehicle_x, vehicle_y, iVeh)
    % find corresponding lane to given pose

    road_data = scenario.road_raw_data;
    min_distance = 32768;
    useLaneletIndex = false;

    if exist('iVeh','var')
        lanelets = scenario.vehicles(iVeh).lanelets_index;
        useLaneletIndex = true;
    else
        lanelets = road_data.lanelet;
    end

    for i = 1:length(lanelets)
        if useLaneletIndex
            centerline_x = scenario.lanelets{ lanelets(i) }(:,LaneletInfo.cx);
            centerline_y = scenario.lanelets{ lanelets(i) }(:,LaneletInfo.cy);
        else
            centerline_x = scenario.lanelets{ i }(:,LaneletInfo.cx);
            centerline_y = scenario.lanelets{ i }(:,LaneletInfo.cy);
        end

        centerline = [centerline_x(1:end),centerline_y(1:end)];

        for j = 1:length(centerline)
            % coordinates have to be recalculated for CPM Lab Mode
            dx = centerline(j) - vehicle_x;
            dy = centerline(j,2) - vehicle_y;
            
            squared_distance = dx * dx + dy * dy;

            if squared_distance < min_distance
                min_distance = squared_distance;

                if useLaneletIndex
                    index = lanelets(i);
                else
                    index = road_data.lanelet(i).idAttribute;
                end
            end
        end

    end


end