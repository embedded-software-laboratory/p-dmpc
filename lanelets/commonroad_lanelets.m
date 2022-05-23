function [lanelets,adjacency,semi_adjacency,intersection_lanelets, commonroad,lanelet_boundary] = commonroad_lanelets()

% COMMONROAD_LANELETS

% Returns:
% lanelets： lanelet information of rightBound, leftBound, and central line of each lanelet
% adjacency: (nLanelets x nLanelets) matrix, entry is 1 if two lanelets are adjacent to each other
% semi_adjacency: (nLanelets x nLanelets) matrix, consecutive adjacent lanelets
% intersection_lanelets: lanelet index of the intersection
% boundary: the inner boundary and outer boundary of the Scenario
% commonroad: raw commonroad data
% lanelet_boundary: left and right boundaries of each lanelet


%    %% lanelets 
%     commonroad_data = readstruct('LabMapCommonRoad_Update.xml');
%     save('commonroad_data.mat','commonroad_data')
    

    %% path of the road data
    [file_path,name,ext] = fileparts(mfilename('fullpath'));
    folder_target = [file_path,filesep,'offline_road_data'];
    road_name = 'commonroad_lanelets.mat';
    road_full_path = [folder_target,filesep,road_name];

    % if the needed road data alread exist, simply load it, otherwise
    % they will be calculated and saved.
    if isfile(road_full_path)
        load(road_full_path,'lanelets','adjacency','semi_adjacency','intersection_lanelets', 'commonroad','lanelet_boundary');
        return
    end

    %% commonroad data
    load('commonroad_data.mat');
    commonroad = commonroad_data;
    
    %% lanelets
    lanelets = cell(1,0);
    Nlanelets = length(commonroad_data.lanelet); % number of lanelets


    for i = 1: Nlanelets

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
     
    
    %% adjacency
    
    adj = zeros(Nlanelets, Nlanelets);
 
    % check the predecessor, successor, adjacentLeft, and adjacentRight of the current lanelet, and assign the pair lanelets to be adjacent
    for i = 1:Nlanelets
        predecessor = commonroad_data.lanelet(i).predecessor;
        successor = commonroad_data.lanelet(i).successor;
        adjacent_index = [];
        
        if isfield(predecessor,'refAttribute')
            predecessor_index = horzcat(predecessor.refAttribute);
            adjacent_index = [adjacent_index,predecessor_index];
            
            % the adjacentLeft and adjacentRight of predecessor are also considered to be adjacent
            for index = predecessor_index
                predecessor_adjacentLeft = commonroad_data.lanelet(index).adjacentLeft;
                predecessor_adjacentRight = commonroad_data.lanelet(index).adjacentRight;
                predecessor_successor = commonroad_data.lanelet(index).successor;
                
                if isfield(predecessor_adjacentLeft,'refAttribute') && strcmp(predecessor_adjacentLeft.drivingDirAttribute,'same')
                   predecessor_adjacentLeft_index = horzcat(predecessor_adjacentLeft.refAttribute);
                   adjacent_index = [adjacent_index,predecessor_adjacentLeft_index];
                end
                if isfield(predecessor_adjacentRight,'refAttribute') && strcmp(predecessor_adjacentRight.drivingDirAttribute,'same')
                   predecessor_adjacentRight_index = horzcat(predecessor_adjacentRight.refAttribute);
                   adjacent_index = [adjacent_index,predecessor_adjacentRight_index];
                end
                
                % check the successor of predecessor to include forking lanelets
                if isfield(predecessor_successor,'refAttribute')
                   predecessor_successor_index = horzcat(predecessor_successor.refAttribute);
                   adjacent_index = [adjacent_index,predecessor_successor_index];
                   
                   for idx = predecessor_successor_index
                       predecessor_successor_adjacentLeft = commonroad_data.lanelet(idx).adjacentLeft;
                       predecessor_successor_adjacentRight = commonroad_data.lanelet(idx).adjacentRight;
                       if isfield(predecessor_successor_adjacentLeft,'refAttribute') && strcmp(predecessor_successor_adjacentLeft.drivingDirAttribute,'same')
                           predecessor_successor_adjacentLeft_index = horzcat(predecessor_successor_adjacentLeft.refAttribute);
                           
                           adjacent_index = [adjacent_index,predecessor_successor_adjacentLeft_index];
                       end
                       if isfield(predecessor_successor_adjacentRight,'refAttribute') && strcmp(predecessor_successor_adjacentRight.drivingDirAttribute,'same')
                           predecessor_successor_adjacentRight_index = horzcat(predecessor_successor_adjacentRight.refAttribute);
                           adjacent_index = [adjacent_index,predecessor_successor_adjacentRight_index];
                       end                      
                   end
                   
                end
            end
        end
        
        if isfield(successor,'refAttribute')
            successor_index = horzcat(successor.refAttribute);
            adjacent_index = [adjacent_index,successor_index];
            
             % the adjacentLeft and adjacentRight of successor are also considered to be adjacent
            for index = successor_index
                successor_adjacentLeft = commonroad_data.lanelet(index).adjacentLeft;
                successor_adjacentRight = commonroad_data.lanelet(index).adjacentRight;
                successor_predecessor = commonroad_data.lanelet(index).predecessor;
  
                if isfield(successor_adjacentLeft,'refAttribute') && strcmp(successor_adjacentLeft.drivingDirAttribute,'same')
                   successor_adjacentLeft_index = horzcat(successor_adjacentLeft.refAttribute);
                   adjacent_index = [adjacent_index,successor_adjacentLeft_index];
                end
                if isfield(successor_adjacentRight,'refAttribute') && strcmp(successor_adjacentRight.drivingDirAttribute,'same')
                   successor_adjacentRight_index = horzcat(successor_adjacentRight.refAttribute);
                   adjacent_index = [adjacent_index,successor_adjacentRight_index];
                end
                
                % check the predecessor of successor to include merging lanelets
                if isfield(successor_predecessor,'refAttribute')
                   successor_predecessor_index = horzcat(successor_predecessor.refAttribute);
                   adjacent_index = [adjacent_index,successor_predecessor_index];
                   
                   for idx = successor_predecessor_index
                       successor_predecessor_adjacentLeft = commonroad_data.lanelet(idx).adjacentLeft;
                       successor_predecessor_adjacentRight = commonroad_data.lanelet(idx).adjacentRight;
                       
                       if isfield(successor_predecessor_adjacentLeft,'refAttribute') && strcmp(successor_predecessor_adjacentLeft.drivingDirAttribute,'same')
                           successor_predecessor_adjacentLeft_index = horzcat(successor_predecessor_adjacentLeft.refAttribute);
                           adjacent_index = [adjacent_index,successor_predecessor_adjacentLeft_index];
                       end
                       
                       if isfield(successor_predecessor_adjacentRight,'refAttribute') && strcmp(successor_predecessor_adjacentRight.drivingDirAttribute,'same')
                           successor_predecessor_adjacentRight_index = horzcat(successor_predecessor_adjacentRight.refAttribute);
                           adjacent_index = [adjacent_index,successor_predecessor_adjacentRight_index];
                       end  
                   end
                end
            end
        end

        adj(i,adjacent_index) = 1;
     
    end
    semi_adjacency = adj + adj' + eye(Nlanelets, Nlanelets); % same lanelet is always adjacent
    semi_adjacency = (semi_adjacency > 0);
    
    %% lanelets boundary
    lanelet_boundary = cell(1,Nlanelets);
    for lanelet = 1:Nlanelets
        adjacentLeft = commonroad_data.lanelet(lanelet).adjacentLeft;
        adjacentRight = commonroad_data.lanelet(lanelet).adjacentRight;

        % if the lanelet has adjacent left lanelet, the boundary should be 
        % the leftBound of adjacentLeft and rightBound of the current lanelet
        
        left_bound_x = lanelets{ lanelet }(:,LaneletInfo.lx);
        left_bound_y = lanelets{ lanelet }(:,LaneletInfo.ly);
        left_bound = [left_bound_x(1:end),left_bound_y(1:end)];

        right_bound_x = lanelets{ lanelet }(:,LaneletInfo.rx);
        right_bound_y = lanelets{ lanelet }(:,LaneletInfo.ry);
        right_bound = [right_bound_x(1:end),right_bound_y(1:end)];
        
        if isfield(adjacentLeft,'refAttribute') && strcmp(adjacentLeft.drivingDirAttribute,'same')
            adjacentLeft_index = horzcat(adjacentLeft.refAttribute);
            left_bound_x = lanelets{ adjacentLeft_index }(:,LaneletInfo.lx);
            left_bound_y = lanelets{ adjacentLeft_index }(:,LaneletInfo.ly);
            left_bound = [left_bound_x(1:end),left_bound_y(1:end)];

        elseif isfield(adjacentRight,'refAttribute') && strcmp(adjacentRight.drivingDirAttribute,'same')
            adjacentRight_index = horzcat(adjacentRight.refAttribute);
            right_bound_x = lanelets{ adjacentRight_index }(:,LaneletInfo.rx);
            right_bound_y = lanelets{ adjacentRight_index }(:,LaneletInfo.ry);
            right_bound = [right_bound_x(1:end),right_bound_y(1:end)];
    
        end
        
        lanelet_boundary{lanelet} = {left_bound,right_bound};
    end 
  
    % forking lanelets
    forking = [9,41,67,87];
    for i = forking
        lanelet_boundary{i}{1} = lanelet_boundary{i}{1}(6:end,:);
    end

    % update the boundary for lanelets at the turning corner
    corner1 = [3,4,22];
    corner2 = [5,6,23];
    corner3 = [29,30,48];
    corner4 = [31,32,49];
    corner5 = [57,58,75];
    corner6 = [55,56,74];
    corner7 = [81,82,100];
    corner8 = [83,84,101];
    
    for lanelet = corner1
        left_bound_x = lanelets{ 4 }(:,LaneletInfo.lx);
        left_bound_y = lanelets{ 4 }(:,LaneletInfo.ly);
        left_bound = [left_bound_x(1:end),left_bound_y(1:end)];

        right_bound_x = lanelets{ 22 }(:,LaneletInfo.rx);
        right_bound_y = lanelets{ 22 }(:,LaneletInfo.ry);
        right_bound = [right_bound_x(1:end),right_bound_y(1:end)];  
        lanelet_boundary{lanelet} = {left_bound,right_bound};
    end
    
    for lanelet = corner2
        left_bound_x = lanelets{ 6 }(:,LaneletInfo.lx);
        left_bound_y = lanelets{ 6 }(:,LaneletInfo.ly);
        left_bound = [left_bound_x(1:end),left_bound_y(1:end)];

        right_bound_x = lanelets{ 23 }(:,LaneletInfo.rx);
        right_bound_y = lanelets{ 23 }(:,LaneletInfo.ry);
        right_bound = [right_bound_x(1:end),right_bound_y(1:end)];  
        lanelet_boundary{lanelet} = {left_bound,right_bound};
    end
    
    for lanelet = corner3
        left_bound_x = lanelets{ 30 }(:,LaneletInfo.lx);
        left_bound_y = lanelets{ 30 }(:,LaneletInfo.ly);
        left_bound = [left_bound_x(1:end),left_bound_y(1:end)];

        right_bound_x = lanelets{ 48 }(:,LaneletInfo.rx);
        right_bound_y = lanelets{ 48 }(:,LaneletInfo.ry);
        right_bound = [right_bound_x(1:end),right_bound_y(1:end)];   
        lanelet_boundary{lanelet} = {left_bound,right_bound};
    end
    
    for lanelet = corner4
        left_bound_x = lanelets{ 32 }(:,LaneletInfo.lx);
        left_bound_y = lanelets{ 32 }(:,LaneletInfo.ly);
        left_bound = [left_bound_x(1:end),left_bound_y(1:end)];

        right_bound_x = lanelets{ 49 }(:,LaneletInfo.rx);
        right_bound_y = lanelets{ 49 }(:,LaneletInfo.ry);
        right_bound = [right_bound_x(1:end),right_bound_y(1:end)]; 
        lanelet_boundary{lanelet} = {left_bound,right_bound};
    end    
    
    for lanelet = corner5
        left_bound_x = lanelets{ 58 }(:,LaneletInfo.lx);
        left_bound_y = lanelets{ 58 }(:,LaneletInfo.ly);
        left_bound = [left_bound_x(1:end),left_bound_y(1:end)];

        right_bound_x = lanelets{ 75 }(:,LaneletInfo.rx);
        right_bound_y = lanelets{ 75 }(:,LaneletInfo.ry);
        right_bound = [right_bound_x(1:end),right_bound_y(1:end)]; 
        lanelet_boundary{lanelet} = {left_bound,right_bound};
    end    

    for lanelet = corner6        
        left_bound_x = lanelets{ 56 }(:,LaneletInfo.lx);
        left_bound_y = lanelets{ 56 }(:,LaneletInfo.ly);
        left_bound = [left_bound_x(1:end),left_bound_y(1:end)];

        right_bound_x = lanelets{ 74 }(:,LaneletInfo.rx);
        right_bound_y = lanelets{ 74 }(:,LaneletInfo.ry);
        right_bound = [right_bound_x(1:end),right_bound_y(1:end)]; 
        lanelet_boundary{lanelet} = {left_bound,right_bound};
    end    

    for lanelet = corner7       
        left_bound_x = lanelets{ 82 }(:,LaneletInfo.lx);
        left_bound_y = lanelets{ 82 }(:,LaneletInfo.ly);
        left_bound = [left_bound_x(1:end),left_bound_y(1:end)];

        right_bound_x = lanelets{ 100 }(:,LaneletInfo.rx);
        right_bound_y = lanelets{ 100 }(:,LaneletInfo.ry);
        right_bound = [right_bound_x(1:end),right_bound_y(1:end)]; 
        lanelet_boundary{lanelet} = {left_bound,right_bound};
    end 
    
    for lanelet = corner8         
        left_bound_x = lanelets{ 84 }(:,LaneletInfo.lx);
        left_bound_y = lanelets{ 84 }(:,LaneletInfo.ly);
        left_bound = [left_bound_x(1:end),left_bound_y(1:end)];

        right_bound_x = lanelets{ 101 }(:,LaneletInfo.rx);
        right_bound_y = lanelets{ 101 }(:,LaneletInfo.ry);
        right_bound = [right_bound_x(1:end),right_bound_y(1:end)]; 
        lanelet_boundary{lanelet} = {left_bound,right_bound};
    end
    
    % convert boundary to polygon
    n_sample = 4; % sampling to reduce polygon's sides to save computation time in later calculations
    for iLanelet=1:Nlanelets
        left_bound_sampled = sample_evenly(lanelet_boundary{iLanelet}{1,1},n_sample,1);
        right_bound_sampled = sample_evenly(lanelet_boundary{iLanelet}{1,2},n_sample,1);
        bound_sampled = [left_bound_sampled;right_bound_sampled(end:-1:1,:)];
        lanelet_boundary{iLanelet}{end+1} = polyshape(bound_sampled);
    end

    %% intersection_lanelets   
    
    % check the lanelets at intersection and assign them to be adjacent
    Nintersections = length(commonroad_data.intersection); 
    shrink_fac = 0.85; % shrink factor to avoid two lanelets being considered as overlapping if they only share one same side
    for i = 1:Nintersections
        intersection_lanelets = [];
        for n = 1:length(commonroad_data.intersection(i).incoming)
            intersection_lanelets = [intersection_lanelets,horzcat(commonroad_data.intersection(i).incoming(n).successorsRight.refAttribute)];
            intersection_lanelets = [intersection_lanelets,horzcat(commonroad_data.intersection(i).incoming(n).successorsLeft.refAttribute)];
            intersection_lanelets = [intersection_lanelets,horzcat(commonroad_data.intersection(i).incoming(n).successorsStraight.refAttribute)];            
        end
%         disp(intersection_lanelets)
        for k =1:length(intersection_lanelets)
            kIntersection = intersection_lanelets(k);
            for l = length(intersection_lanelets) : -1 : k+1
                lIntersection = intersection_lanelets(l);
                kBoundary = lanelet_boundary{1,kIntersection}{1,3}.Vertices;
                kBoundary_shrink = shrink_fac*(kBoundary-mean(kBoundary)) + mean(kBoundary);
                lBoundary = lanelet_boundary{1,lIntersection}{1,3}.Vertices;
                lBoundary_shrink = shrink_fac*(lBoundary-mean(lBoundary)) + mean(lBoundary);
                if overlaps(polyshape(kBoundary_shrink),polyshape(lBoundary_shrink)) % check if two lanelets at intersection have overlapping boundaries
                    adj(kIntersection,lIntersection) = 1;
                end
            end
        end

    end

    adjacency = adj + adj' + eye(Nlanelets, Nlanelets); % same lanelet is always adjacent
    adjacency = (adjacency>0);
    
    %% save all the road data offline
    save(road_full_path,...
        'lanelets','adjacency','semi_adjacency','intersection_lanelets', 'commonroad','lanelet_boundary',...
        '-mat')
end



