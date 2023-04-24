function result = map_position_to_closest_lanelets(lanelets, vehicle_x, vehicle_y)

    min_distance = inf;
    offset = 0.1; % offset for mutiple closest lanelets

    for iLanelet = 1:length(lanelets)
        centerline = [
                      lanelets{iLanelet}(:, LaneletInfo.cx)'
                      lanelets{iLanelet}(:, LaneletInfo.cy)'
                      ];

        position_vehicle = [vehicle_x; vehicle_y];
        distance_squared = vecnorm(centerline - position_vehicle);
        min_distance_squared = min(distance_squared);

        if min_distance_squared < min_distance
            min_distance = min_distance_squared;
            result = iLanelet;
        elseif min_distance_squared <= min_distance+offset
            result = unique([result iLanelet]);
        end

    end

end
