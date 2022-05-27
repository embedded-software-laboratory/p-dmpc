function index = match_pose_to_lane(scenario, vehicle_x, vehicle_y)
    % find corresponding lane to given pose
    % inspired by high_level_controller/examples/cpp/central_routing/src/lane_graph_tools.cpp map_match_pose


    commonroad_data = scenario.commonroad_data;
    min_distance = 32768;

    %{
    lanelets = cell(1,0);
    for i = 1:length(commonroad_data.lanelet)
        nPoints = length(horzcat(commonroad_data.lanelet(i).leftBound.point.x));
        l = zeros(nPoints,LaneletInfo.nCols);
        l(:,LaneletInfo.rx) = horzcat(commonroad_data.lanelet(i).rightBound.point.x);
        l(:,LaneletInfo.ry) = horzcat(commonroad_data.lanelet(i).rightBound.point.y);
        l(:,LaneletInfo.lx) = horzcat(commonroad_data.lanelet(i).leftBound.point.x);
        l(:,LaneletInfo.ly) = horzcat(commonroad_data.lanelet(i).leftBound.point.y);
        l(:,LaneletInfo.cx) = 1/2*(l(:,LaneletInfo.lx)+l(:,LaneletInfo.rx));
        l(:,LaneletInfo.cy) = 1/2*(l(:,LaneletInfo.ly)+l(:,LaneletInfo.ry));
        lanelets{end+1} = l;
    end
    %}

    %disp(vehicle_x);
    %disp(vehicle_y);

    for i = 1:length(commonroad_data.lanelet)
        centerline_x = scenario.lanelets{ i }(:,LaneletInfo.cx);
        centerline_y = scenario.lanelets{ i }(:,LaneletInfo.cy);
        centerline = [centerline_x(1:end),centerline_y(1:end)];
        %disp(centerline);

        for j = 1:length(centerline)
            % coordinates have to be recalculated for CPM Lab Mode
            dx = centerline(j) - vehicle_x;
            dy = centerline(j,2) - vehicle_y;
            %disp(sprintf("center x: %f", centerline(j)));
            %disp(sprintf("center y: %f", centerline(j,2)));

            squared_distance = dx * dx + dy * dy;
            %disp(sprintf("index: %d, distance: %f", commonroad_data.lanelet(i).idAttribute, squared_distance));

            if squared_distance < min_distance
                min_distance = squared_distance;
                index = commonroad_data.lanelet(i).idAttribute;
            end
        end

    end


end