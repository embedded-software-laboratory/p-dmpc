function [random_path, scenario, lane_change_indices, lane_change_lanes] = generate_random_path(scenario, vehid, n, startPosition)
    % GENERATE_RANDOM_PATH    returns a ref_path struct
    % random_path.lanelets_index: lanelet index of the reference path
    % random_path.path: reference path including x and y information
    % random_path.points_index: count the max index of reference points for each lanelets

    random_path = struct;
    lane_change_indices = zeros(10,4);
    lane_change_lanes = zeros(10,2);
    lanelets_index_vehid = startPosition;  
    
    road_data = scenario.road_raw_data;

    random_path.lanelets_index = [lanelets_index_vehid];
    laneChangeAllowed = false;

    while length(random_path.lanelets_index) < n
        % find all edges that are the successor of the current lane or adjacent and lane change enabled
        subsequent_lanes = struct;
        subsequent_lanes.lanelets_index = [0];

        for i = 1:length(road_data.lanelet)
            
            % find id of current lane
            if random_path.lanelets_index(end) == road_data.lanelet(i).idAttribute
                successor = road_data.lanelet(i).successor;
                successor_indices = horzcat(successor.refAttribute);
                subsequent_indices = successor_indices;


                if laneChangeAllowed
                    %disp(subsequent_indices);
                    for k = 1:length(successor_indices)
                        successor_adjacentLeft = road_data.lanelet(successor_indices(k)).adjacentLeft;
                        if isfield(successor_adjacentLeft,'refAttribute') && strcmp(successor_adjacentLeft.drivingDirAttribute,'same')
                            subsequent_indices = horzcat(subsequent_indices, successor_adjacentLeft.refAttribute);
                            %disp(subsequent_indices);
                        end

                        successor_adjacentRight = road_data.lanelet(successor_indices(k)).adjacentRight;
                        if isfield(successor_adjacentRight,'refAttribute') && strcmp(successor_adjacentRight.drivingDirAttribute,'same')
                            subsequent_indices = horzcat(subsequent_indices, successor_adjacentRight.refAttribute);
                            %disp(subsequent_indices);
                        end

                        % include check for predecessor of successor
                    end
                end
                
                for j = 1:length(subsequent_indices)
                    if subsequent_lanes.lanelets_index(1) == 0
                        subsequent_lanes.lanelets_index(1) = subsequent_indices(j);
                    else
                        subsequent_lanes.lanelets_index(end+1) = subsequent_indices(j);
                    end
                end

                %disp(sprintf('subsequent lanelets index size: %d', length(subsequent_lanes.lanelets_index)));
            end
        end
        
        if laneChangeAllowed
            range = length(subsequent_lanes.lanelets_index);

            if scenario.options.is_eval
                rng(scenario.options.seed.Seed);
            end
            index = randi(range);
            %disp(sprintf('range: %d, index: %d', range, index));

            random_path.lanelets_index(end+1) = subsequent_lanes.lanelets_index(index);

            %disp(sprintf('random at end: %d', random_path.lanelets_index(end)));
            laneChangeAllowed = false;
        else
            if length(subsequent_lanes.lanelets_index) > 1
                if (random_path.lanelets_index(end) == 3 || random_path.lanelets_index(end) == 5 || random_path.lanelets_index(end) == 81 ... 
                    || random_path.lanelets_index(end) == 83)
                    random_path.lanelets_index(end+1) = random_path.lanelets_index(end) + 2;
                elseif (random_path.lanelets_index(end) == 11 || random_path.lanelets_index(end) == 89)
                    random_path.lanelets_index(end+1) = random_path.lanelets_index(end) + 15;
                elseif (random_path.lanelets_index(end) == 12 || random_path.lanelets_index(end) == 40 || random_path.lanelets_index(end) == 66 ...
                        || random_path.lanelets_index(end) == 90)
                    random_path.lanelets_index(end+1) = random_path.lanelets_index(end) + 5;
                elseif (random_path.lanelets_index(end) == 22 || random_path.lanelets_index(end) == 100)
                    random_path.lanelets_index(end+1) = random_path.lanelets_index(end) - 17;
                elseif (random_path.lanelets_index(end) == 29 || random_path.lanelets_index(end) == 31 || random_path.lanelets_index(end) == 55 ...
                        || random_path.lanelets_index(end) == 57)
                    random_path.lanelets_index(end+1) = random_path.lanelets_index(end) - 2;
                elseif (random_path.lanelets_index(end) == 39 || random_path.lanelets_index(end) == 65)
                    random_path.lanelets_index(end+1) = random_path.lanelets_index(end) + 11;
                elseif (random_path.lanelets_index(end) == 49 || random_path.lanelets_index(end) == 75)
                    random_path.lanelets_index(end+1) = random_path.lanelets_index(end) - 20;
                end 
            else
                random_path.lanelets_index(end+1) = subsequent_lanes.lanelets_index(end);
            end

            laneChangeAllowed = true;
        end
    end

    % paths for debugging
    %random_path.lanelets_index = [2,4,6,8,59,57,56,54,80,82,84,86,33,31,48,42,39,50,20,63,61,57,];
    %random_path.lanelets_index = [2,4,6,8,57,54,80,82,84,86,31,48,42,50,20,63,61,57];

    disp(sprintf('id: %d, lanelets: [%s]', vehid, join(string(random_path.lanelets_index))));

    %[lanelets, ~,~,~,~,scenario.lanelet_boundary] = commonroad_lanelets(scenario.options.mixedTrafficScenarioLanelets);
    % [lanelets, ~,~,~,scenario.lanelet_boundary,~,~] = get_road_data();
    lanelets = scenario.lanelets;

    % reference path
    path = [];

    % the max points index of each lanelet
    lanelet_point_max = 0;
    points_index = zeros(1,length( random_path.lanelets_index));
    laneChangeLanesIndices = [];
    beforeLaneChangeIndices = [];

    % find indices of lanes that were chosen as adjacents lanes
    for i = 1:length(random_path.lanelets_index)-1
        successor = road_data.lanelet(random_path.lanelets_index(i)).successor; 
        successor_indices = horzcat(successor.refAttribute);

        for k = 1:length(successor_indices)
            successor_adjacentLeft = road_data.lanelet(successor_indices(k)).adjacentLeft;
            if isfield(successor_adjacentLeft,'refAttribute') && strcmp(successor_adjacentLeft.drivingDirAttribute,'same')
                if ismember(random_path.lanelets_index(i+1), successor_adjacentLeft.refAttribute)
                    laneChangeLanesIndices = [laneChangeLanesIndices, i+1];
                    beforeLaneChangeIndices = [beforeLaneChangeIndices, i];
                    break
                end
            end

            successor_adjacentRight = road_data.lanelet(successor_indices(k)).adjacentRight;
            if isfield(successor_adjacentRight,'refAttribute') && strcmp(successor_adjacentRight.drivingDirAttribute,'same')
                if ismember(random_path.lanelets_index(i+1), successor_adjacentRight.refAttribute)
                    laneChangeLanesIndices = [laneChangeLanesIndices, i+1];
                    beforeLaneChangeIndices = [beforeLaneChangeIndices, i];
                    break
                end
            end
        end
    end

    delete_index = 1;

    % interpolate points for smooth lane changes
    for nlanelets = 1:length(random_path.lanelets_index)
        % choose the center line of the lanelet as reference path
        randomPath_x = lanelets{ random_path.lanelets_index(nlanelets)}(:,LaneletInfo.cx);
        randomPath_y = lanelets{ random_path.lanelets_index(nlanelets)}(:,LaneletInfo.cy);
        %delete_index = 1;
        delete_index_x = 1;
        delete_index_y = 1;
        start_index = 1;

        if nlanelets < length(random_path.lanelets_index)
            %{
            if ismember((nlanelets+1), laneChangeLanesIndices)
                % the next lane is an adjacent lane, interpolate diagonal from middle index of current lane to next lane
                middleIndex = uint8(length(randomPath_x) / 2);
                remainingIndices = length(randomPath_x) - middleIndex;
                new_lane_start_x = lanelets{ random_path.lanelets_index(nlanelets+1)}(1,LaneletInfo.cx);
                new_lane_start_y = lanelets{ random_path.lanelets_index(nlanelets+1)}(1,LaneletInfo.cy);

                distance = sqrt((randomPath_x(middleIndex) - new_lane_start_x).^2 + (randomPath_y(middleIndex) - new_lane_start_y).^2);

                
                for j = 1:remainingIndices
                    if randomPath_x(middleIndex) >= new_lane_start_x
                        if randomPath_x(middleIndex+j) >= new_lane_start_x
                            point_x = randomPath_x(middleIndex+j) - ((double(j)/double(remainingIndices) * distance) * (randomPath_x(middleIndex+j) - new_lane_start_x)/distance);
                        else
                            %point_x = randomPath_x(middleIndex+(j-1)) - ((double(j)/double(remainingIndices) * distance) * (randomPath_x(middleIndex+(j-1)) - new_lane_start_x)/distance);
                            % index speichern, punkt rausnehmen, nur für fahrzeug boundary punkt aus lane nehmen
                            delete_index = (remainingIndices - j) + 1;
                            break
                        end
                    else
                        if randomPath_x(middleIndex+j) < new_lane_start_x
                            point_x = randomPath_x(middleIndex+j) + ((double(j)/double(remainingIndices) * distance) * (new_lane_start_x - randomPath_x(middleIndex+j))/distance);
                        else
                            %point_x = randomPath_x(middleIndex+(j-1)) + ((double(j)/double(remainingIndices) * distance) * (new_lane_start_x - randomPath_x(middleIndex+(j-1)))/distance);
                            delete_index = (remainingIndices - j) + 1;
                            break
                        end
                    end

                    if randomPath_y(middleIndex) >= new_lane_start_y
                        if randomPath_y(middleIndex+j) >= new_lane_start_y
                            point_y = randomPath_y(middleIndex+j) - ((double(j)/double(remainingIndices) * distance) * (randomPath_y(middleIndex+j) - new_lane_start_y)/distance);
                        else
                            %point_y = randomPath_y(middleIndex+(j-1)) - ((double(j)/double(remainingIndices) * distance) * (randomPath_y(middleIndex+(j-1)) - new_lane_start_y)/distance);
                            delete_index = (remainingIndices - j) + 1;
                            break
                        end
                    else
                        if randomPath_y(middleIndex+j) < new_lane_start_y
                            point_y = randomPath_y(middleIndex+j) + ((double(j)/double(remainingIndices) * distance) * (new_lane_start_y - randomPath_y(middleIndex+j))/distance);
                        else
                            %point_y = randomPath_y(middleIndex+(j-1)) + ((double(j)/double(remainingIndices) * distance) * (new_lane_start_y - randomPath_y(middleIndex+(j-1)))/distance);
                            delete_index = (remainingIndices - j) + 1;
                            break
                        end
                    end

                    randomPath_x(middleIndex+j) = point_x;
                    randomPath_y(middleIndex+j) = point_y;
                end
                

                %{
                for j = 1:remainingIndices
                    if randomPath_x(middleIndex+j) > new_lane_start_x
                        point_x = randomPath_x(middleIndex+j) - ((double(j)/double(remainingIndices) * distance) * (randomPath_x(middleIndex+j) - new_lane_start_x)/distance);
                    else
                        point_x = randomPath_x(middleIndex+j) + ((double(j)/double(remainingIndices) * distance) * (new_lane_start_x - randomPath_x(middleIndex+j))/distance);
                    end

                    if randomPath_y(middleIndex+j) > new_lane_start_y
                        point_y = randomPath_y(middleIndex+j) - ((double(j)/double(remainingIndices) * distance) * (randomPath_y(middleIndex+j) - new_lane_start_y)/distance);
                    else
                        point_y = randomPath_y(middleIndex+j) + ((double(j)/double(remainingIndices) * distance) * (new_lane_start_y - randomPath_y(middleIndex+j))/distance);
                    end

                    randomPath_x(middleIndex+j) = point_x;
                    randomPath_y(middleIndex+j) = point_y;
                end
                %}
            end
            %}
            
            %{
            if ismember((nlanelets+1), laneChangeLanesIndices)
                %{
                if randomPath_x(1) >= lanelets{ random_path.lanelets_index(nlanelets+1)}(1,LaneletInfo.cx)
                    while randomPath_x(end) < lanelets{ random_path.lanelets_index(nlanelets+1)}(delete_index_x,LaneletInfo.cx)
                        delete_index_x = delete_index_x+1;
                    end 
                else
                    while randomPath_x(end) > lanelets{ random_path.lanelets_index(nlanelets+1)}(delete_index_x,LaneletInfo.cx)
                        delete_index_x = delete_index_x+1;
                    end
                end

                if randomPath_y(1) >= lanelets{ random_path.lanelets_index(nlanelets+1)}(1,LaneletInfo.cy)
                    while randomPath_y(end) < lanelets{ random_path.lanelets_index(nlanelets+1)}(delete_index_y,LaneletInfo.cy)
                        delete_index_y = delete_index_y+1;
                    end 
                else
                    while randomPath_y(end) > lanelets{ random_path.lanelets_index(nlanelets+1)}(delete_index_y,LaneletInfo.cy)
                        delete_index_y = delete_index_y+1;
                    end
                end
                delete_index = max(delete_index_x, delete_index_y);
                remainingIndices = length(randomPath_x) + delete_index;
                new_lane_start_x = lanelets{ random_path.lanelets_index(nlanelets+1)}(delete_index,LaneletInfo.cx);
                new_lane_start_y = lanelets{ random_path.lanelets_index(nlanelets+1)}(delete_index,LaneletInfo.cy);

                distance = sqrt((randomPath_x(1) - new_lane_start_x).^2 + (randomPath_y(1) - new_lane_start_y).^2);
                %}
                %{
                for j = 1:remainingIndices
                    if randomPath_x(middleIndex) >= new_lane_start_x
                        if randomPath_x(middleIndex+j) >= new_lane_start_x
                            point_x = randomPath_x(middleIndex+j) - ((double(j)/double(remainingIndices) * distance) * (randomPath_x(middleIndex+j) - new_lane_start_x)/distance);
                        else
                            %point_x = randomPath_x(middleIndex+(j-1)) - ((double(j)/double(remainingIndices) * distance) * (randomPath_x(middleIndex+(j-1)) - new_lane_start_x)/distance);
                            % index speichern, punkt rausnehmen, nur für fahrzeug boundary punkt aus lane nehmen
                            delete_index = (remainingIndices - j) + 1;
                            break
                        end
                    else
                        if randomPath_x(middleIndex+j) < new_lane_start_x
                            point_x = randomPath_x(middleIndex+j) + ((double(j)/double(remainingIndices) * distance) * (new_lane_start_x - randomPath_x(middleIndex+j))/distance);
                        else
                            %point_x = randomPath_x(middleIndex+(j-1)) + ((double(j)/double(remainingIndices) * distance) * (new_lane_start_x - randomPath_x(middleIndex+(j-1)))/distance);
                            delete_index = (remainingIndices - j) + 1;
                            break
                        end
                    end

                    if randomPath_y(middleIndex) >= new_lane_start_y
                        if randomPath_y(middleIndex+j) >= new_lane_start_y
                            point_y = randomPath_y(middleIndex+j) - ((double(j)/double(remainingIndices) * distance) * (randomPath_y(middleIndex+j) - new_lane_start_y)/distance);
                        else
                            %point_y = randomPath_y(middleIndex+(j-1)) - ((double(j)/double(remainingIndices) * distance) * (randomPath_y(middleIndex+(j-1)) - new_lane_start_y)/distance);
                            delete_index = (remainingIndices - j) + 1;
                            break
                        end
                    else
                        if randomPath_y(middleIndex+j) < new_lane_start_y
                            point_y = randomPath_y(middleIndex+j) + ((double(j)/double(remainingIndices) * distance) * (new_lane_start_y - randomPath_y(middleIndex+j))/distance);
                        else
                            %point_y = randomPath_y(middleIndex+(j-1)) + ((double(j)/double(remainingIndices) * distance) * (new_lane_start_y - randomPath_y(middleIndex+(j-1)))/distance);
                            delete_index = (remainingIndices - j) + 1;
                            break
                        end
                    end

                    randomPath_x(middleIndex+j) = point_x;
                    randomPath_y(middleIndex+j) = point_y;
                end
                %}
            elseif ismember((nlanelets), laneChangeLanesIndices)
                start_index = delete_index;
                delete_index = 1;
            end
            %}

            if ismember(nlanelets, laneChangeLanesIndices)
                for i = 1:length(lane_change_indices)
                    if lane_change_indices(i) == 0
                        lane_change_indices(i,1) = (length(path) - length(lanelets{ random_path.lanelets_index(nlanelets+1)}(:,LaneletInfo.cx)))+1;
                        lane_change_indices(i,2) = length(path);
                        lane_change_indices(i,3) = length(path) + 1;
                        lane_change_indices(i,4) = (length(path) + length(randomPath_x));
                        break
                    end
                end

                for i = 1:length(lane_change_lanes)
                    if lane_change_lanes(i,1) == 0
                        lane_change_lanes(i,1) = nlanelets-1;
                        lane_change_lanes(i,2) = nlanelets;
                        break
                    end
                end
            end
        end

        if ~ismember(nlanelets, laneChangeLanesIndices)
            randomPath_next = [randomPath_x(start_index:(end)),randomPath_y(start_index:(end))];
            path = [path; randomPath_next];
        else
            startIndex = uint8(length(randomPath_x) / 2);
            randomPath_next = [randomPath_x(startIndex:end),randomPath_y(startIndex:end)];
            path = [path; randomPath_next];
        end 

        % count the number of points of each lanelets
        Npoints = length(randomPath_next);
        % max point index of each lanelet
        lanelet_point_max = lanelet_point_max + Npoints;
        points_index(nlanelets) = lanelet_point_max;
    end 
    random_path.path = path;
    random_path.points_index = points_index;

    %{
    % reduce lanelet boundarys to get same number of points as lanelets
    for index = 1:length(scenario.lanelet_boundary)
        scenario.lanelet_boundary{1,index}{1,1} = scenario.lanelet_boundary{1,index}{1,1}(2:(end)-1,1:2);
        scenario.lanelet_boundary{1,index}{1,2} = scenario.lanelet_boundary{1,index}{1,2}(2:(end)-1,1:2);
    end
    %}

    %{
    % the max points index of each lanelet
    lanelet_point_max = 0;
    points_index = zeros(1,length( random_path.lanelets_index));
    for nlanelets = 1:length( random_path.lanelets_index)
        % count the number of points of each lanelets
        Npoints = length(lanelets{ random_path.lanelets_index(nlanelets)}(:,LaneletInfo.cx));
        % max point index of each lanelet
        lanelet_point_max = lanelet_point_max + Npoints;
        points_index(nlanelets) = lanelet_point_max;

    end 
    random_path.points_index = points_index;
    %}

end