function [manual_path, scenario] = generate_manual_path(scenario, vehid, n, startPosition, endOnInnerLane)
    % GENERATE_MANUAL_PATH    returns a ref_path struct
    % manual_path.lanelets_index: lanelet index of the reference path
    % manual_path.path: reference path including x and y information
    % manual_path.points_index: count the max index of reference points for each lanelets

    manual_path = struct;
    lanelets_index_vehid = startPosition;

    road_data = scenario.road_raw_data;

    manual_path.lanelets_index = [lanelets_index_vehid];

    while length(manual_path.lanelets_index) < n
        % find all edges that are the successor of the current lane or adjacent and lane change enabled
        subsequent_lanes = struct;
        subsequent_lanes.lanelets_index = [0];

        for i = 1:length(road_data.lanelet)

            % find id of current lane
            if manual_path.lanelets_index(end) == road_data.lanelet(i).idAttribute
                successor = road_data.lanelet(i).successor;
                subsequent_indices = horzcat(successor.refAttribute);

                for j = 1:length(subsequent_indices)

                    if subsequent_lanes.lanelets_index(1) == 0
                        subsequent_lanes.lanelets_index(1) = subsequent_indices(j);
                    else
                        subsequent_lanes.lanelets_index(end + 1) = subsequent_indices(j);
                    end

                end

            end

        end

        if length(subsequent_lanes.lanelets_index) > 1

            if (manual_path.lanelets_index(end) == 3 || manual_path.lanelets_index(end) == 5 || manual_path.lanelets_index(end) == 81 ...
                    || manual_path.lanelets_index(end) == 83)
                manual_path.lanelets_index(end + 1) = manual_path.lanelets_index(end) + 2;
            elseif (manual_path.lanelets_index(end) == 11 || manual_path.lanelets_index(end) == 89)
                manual_path.lanelets_index(end + 1) = manual_path.lanelets_index(end) + 15;
            elseif (manual_path.lanelets_index(end) == 12 || manual_path.lanelets_index(end) == 40 || manual_path.lanelets_index(end) == 66 ...
                    || manual_path.lanelets_index(end) == 90)
                manual_path.lanelets_index(end + 1) = manual_path.lanelets_index(end) + 5;
            elseif (manual_path.lanelets_index(end) == 22 || manual_path.lanelets_index(end) == 100)
                manual_path.lanelets_index(end + 1) = manual_path.lanelets_index(end) - 17;
            elseif (manual_path.lanelets_index(end) == 29 || manual_path.lanelets_index(end) == 31 || manual_path.lanelets_index(end) == 55 ...
                    || manual_path.lanelets_index(end) == 57)
                manual_path.lanelets_index(end + 1) = manual_path.lanelets_index(end) - 2;
            elseif (manual_path.lanelets_index(end) == 39 || manual_path.lanelets_index(end) == 65)
                manual_path.lanelets_index(end + 1) = manual_path.lanelets_index(end) + 11;
            elseif (manual_path.lanelets_index(end) == 49 || manual_path.lanelets_index(end) == 75)
                manual_path.lanelets_index(end + 1) = manual_path.lanelets_index(end) - 20;
            end

        else
            manual_path.lanelets_index(end + 1) = subsequent_lanes.lanelets_index(end);
        end

    end

    if endOnInnerLane
        index_end = manual_path.lanelets_index(end);
        laneID = find_lane_for_change(scenario, index_end, false);

        if laneID ~= 0
            manual_path.lanelets_index(end) = laneID;
        end

    end

    disp(sprintf('id: %d, lanelets: [%s]', vehid, join(string(manual_path.lanelets_index))));

    lanelets = scenario.lanelets;

    % reference path
    path = [];

    % the max points index of each lanelet
    lanelet_point_max = 0;
    points_index = zeros(1, length(manual_path.lanelets_index));

    for nlanelets = 1:length(manual_path.lanelets_index)
        % choose the center line of the lanelet as reference path
        manualPath_x = lanelets{manual_path.lanelets_index(nlanelets)}(:, LaneletInfo.cx);
        manualPath_y = lanelets{manual_path.lanelets_index(nlanelets)}(:, LaneletInfo.cy);

        manualPath_next = [manualPath_x(1:end), manualPath_y(1:end)];
        path = [path; manualPath_next];

        % count the number of points of each lanelets
        Npoints = length(manualPath_next);
        % max point index of each lanelet
        lanelet_point_max = lanelet_point_max + Npoints;
        points_index(nlanelets) = lanelet_point_max;

    end

    manual_path.path = path;
    manual_path.points_index = points_index;

end
