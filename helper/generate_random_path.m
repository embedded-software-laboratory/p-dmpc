function [random_path, scenario] = generate_random_path(scenario, vehid, n, startPosition)
    % GENERATE_RANDOM_PATH    returns a ref_path struct
    % random_path.lanelets_index: lanelet index of the reference path
    % random_path.path: reference path including x and y information
    % random_path.points_index: count the max index of reference points for each lanelets

    random_path = struct;
    
    disp(sprintf('Id: %d', vehid));
   
    lanelets_index_vehid = startPosition;  
    
    load('commonroad_data.mat');
    commonroad = commonroad_data;

    random_path.lanelets_index = [lanelets_index_vehid];
    laneChangeAllowed = false;

    while length(random_path.lanelets_index) < n
        % find all edges that are the successor of the current lane or adjacent and lane change enabled
        subsequent_lanes = struct;
        subsequent_lanes.lanelets_index = [0];

        for i = 1:length(commonroad_data.lanelet)
            
            % find id of current lane
            if random_path.lanelets_index(end) == commonroad_data.lanelet(i).idAttribute
                successor = commonroad_data.lanelet(i).successor;
                successor_indices = horzcat(successor.refAttribute);
                subsequent_indices = successor_indices;


                if laneChangeAllowed
                    %disp(subsequent_indices);
                    for k = 1:length(successor_indices)
                        successor_adjacentLeft = commonroad_data.lanelet(successor_indices(k)).adjacentLeft;
                        if isfield(successor_adjacentLeft,'refAttribute') && strcmp(successor_adjacentLeft.drivingDirAttribute,'same')
                            subsequent_indices = horzcat(subsequent_indices, successor_adjacentLeft.refAttribute);
                            %disp(subsequent_indices);
                        end

                        successor_adjacentRight = commonroad_data.lanelet(successor_indices(k)).adjacentRight;
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

    %random_path.lanelets_index = [59,57,55,67,65,98,37,35,31,29,27,1,3,5,7];
    %random_path.lanelets_index = [59,57,55,67,65,98,37,35,31,29,27,1,4,6,8];
    %random_path.lanelets_index = [8,60,57,55,67,65,71,19,14,16,22,5,9,11,18,14,15,3,6,8,59];

    for i = 1:length(random_path.lanelets_index)
        disp(sprintf('random entries: i: %d, entry: %d', i,random_path.lanelets_index(i)));
    end

    %[lanelets, ~,~,~,~,scenario.lanelet_boundary] = commonroad_lanelets(scenario.options.mixedTrafficScenarioLanelets);
    
    [lanelets, ~,~,~,scenario.lanelet_boundary,~,~] = get_road_data();

    % reference path
    path = [];

    % the max points index of each lanelet
    lanelet_point_max = 0;
    points_index = zeros(1,length( random_path.lanelets_index));
    laneChangeLanesIndices = [];

    % find indices of lanes that were chosen as adjacents lanes
    for i = 1:length(random_path.lanelets_index)-1
        successor = commonroad_data.lanelet(random_path.lanelets_index(i)).successor; 
        successor_indices = horzcat(successor.refAttribute);

        for k = 1:length(successor_indices)
            successor_adjacentLeft = commonroad_data.lanelet(successor_indices(k)).adjacentLeft;
            if isfield(successor_adjacentLeft,'refAttribute') && strcmp(successor_adjacentLeft.drivingDirAttribute,'same')
                if ismember(random_path.lanelets_index(i+1), successor_adjacentLeft.refAttribute)
                    laneChangeLanesIndices = [laneChangeLanesIndices, i+1];
                    break
                end
            end

            successor_adjacentRight = commonroad_data.lanelet(successor_indices(k)).adjacentRight;
            if isfield(successor_adjacentRight,'refAttribute') && strcmp(successor_adjacentRight.drivingDirAttribute,'same')
                if ismember(random_path.lanelets_index(i+1), successor_adjacentRight.refAttribute)
                    laneChangeLanesIndices = [laneChangeLanesIndices, i+1];
                    break
                end
            end
        end
    end

    % interpolate points for smooth lane changes
    for nlanelets = 1:length(random_path.lanelets_index)
        % choose the center line of the lanelet as reference path
        randomPath_x = lanelets{ random_path.lanelets_index(nlanelets)}(:,LaneletInfo.cx);
        randomPath_y = lanelets{ random_path.lanelets_index(nlanelets)}(:,LaneletInfo.cy);

        if nlanelets < length(random_path.lanelets_index)
            if ismember((nlanelets+1), laneChangeLanesIndices)
                % the next lane is an adjacent lane, interpolate diagonal from middle index of current lane to next lane
                middleIndex = uint8(length(randomPath_x) / 2);
                remainingIndices = length(randomPath_x) - middleIndex;
                new_lane_start_x = lanelets{ random_path.lanelets_index(nlanelets+1)}(1,LaneletInfo.cx);
                new_lane_start_y = lanelets{ random_path.lanelets_index(nlanelets+1)}(1,LaneletInfo.cy);

                distance = sqrt((randomPath_x(middleIndex) - new_lane_start_x).^2 + (randomPath_y(middleIndex) - new_lane_start_y).^2);

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
            end
        end

        randomPath_next = [randomPath_x(1:end),randomPath_y(1:end)];
        path = [path; randomPath_next];

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