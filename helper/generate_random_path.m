function random_path = generate_random_path(scenario, vehid, n)
    % GENERATE_RANDOM_PATH    returns a ref_path struct
    % random_path.lanelets_index: lanelet index of the reference path
    % random_path.path: reference path including x and y information
    % random_path.points_index: count the max index of reference points for each lanelets

    random_path = struct;
    
    disp(sprintf('Id: %d', vehid));
    % maybe scenario.vehicles not yet initialized
    % lanelets_index_vehid = scenario.vehicles(1,vehid).lanelets_index; 
    lanelets_index_vehid = 31+vehid;  
    
    load('commonroad_data.mat');
    commonroad = commonroad_data;

    %{ 
    if vehicle position not directly accessable, iterate over lanelets to find position
    for i = 1:length(lanelets)
        if    
    end
    %}

    % find initial position for this
    random_path.lanelets_index = [lanelets_index_vehid];

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
        
        range = length(subsequent_lanes.lanelets_index);
        index = randi(range);
        %disp(sprintf('range: %d, index: %d', range, index));
        random_path.lanelets_index(end+1) = subsequent_lanes.lanelets_index(index);
        %disp(sprintf('random at end: %d', random_path.lanelets_index(end)));
    end

    for i = 1:length(random_path.lanelets_index)
        disp(sprintf('random entries: i: %d, entry: %d', i,random_path.lanelets_index(i)));
    end

    [lanelets, ~,~,~,~,~] = commonroad_lanelets(); 

    % reference path
    path = [];

    for nlanelets = 1:length(random_path.lanelets_index)
        % choose the center line of the lanelet as reference path
        randomPath_x = lanelets{ random_path.lanelets_index(nlanelets)}(:,LaneletInfo.cx);
        randomPath_y = lanelets{ random_path.lanelets_index(nlanelets)}(:,LaneletInfo.cy);

        if length(randomPath_x) > 3
            randomPath_x = randomPath_x([2:(end-2)]);
        end
        
        if length(randomPath_y) > 3
            randomPath_y = randomPath_y([2:(end-2)]);
        end

        randomPath_next = [randomPath_x(1:end),randomPath_y(1:end)];
        path = [path; randomPath_next];

    end 
    random_path.path = path;

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


end