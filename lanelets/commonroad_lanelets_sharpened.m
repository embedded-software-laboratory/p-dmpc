function [lanelets,adjacency_sharpened,semi_adjacency_sharpened,intersection_lanelets, commonroad,lanelet_boundary] = commonroad_lanelets_sharpened()

% COMMONROAD_LANELETS_SHARPENED The same as COMMONROAD_LANELETS, except
% that there are more conditions for two lanelets to be adjacent and the
% boundary of one lanelet is also modified.

% Returns:
% lanelets： lanelet information of rightBound, leftBound, and central line of each lanelet
% adjacency: (nLanelets x nLanelets) matrix, entry is 1 if two lanelets are adjacent to each other
% semi_adjacency: (nLanelets x nLanelets) matrix, consecutive adjacent lanelets
% intersection_lanelets: lanelet index of the intersection
% boundary: the inner boundary and outer boundary of the Scenario
% commonroad: raw commonroad data
% lanelet_boundary: left and right boundaries of each lanelet

    % path of the road data
    [file_path,name,ext] = fileparts(mfilename('fullpath'));
    folder_target = [file_path,filesep,'offline_road_data'];
    road_name = 'commonroad_lanelets_sharpened.mat';
    road_full_path = [folder_target,filesep,road_name];

    % if the needed road data alread exist, simply load it, otherwise
    % they will be calculated and saved.
    if isfile(road_full_path)
        load(road_full_path,'lanelets','adjacency_sharpened','semi_adjacency_sharpened','intersection_lanelets', 'commonroad','lanelet_boundary');
        return
    end

%    %% lanelets 
%     commonroad_data = readstruct('LabMapCommonRoad_Update.xml');
%     save('commonroad_data.mat','commonroad_data')
    
    % commonroad data
    load('commonroad_data.mat','commonroad_data');
    commonroad = commonroad_data;

    % `lanelets`
    lanelets = get_lannelets(commonroad_data);
    
    % `semi_adjacency`
    % Predecessor and successor of the current lanelet as adjacent lanelets
    % Compare to not sharpened version, here adjacentLeft and adjacentRight are no longer considered as adjacent lanelets
    [semi_adjacency_sharpened,semi_adj] = get_semi_adjacency(commonroad_data);
    
    % `adjacency`
    % In addition to semi_adjacency, two lanelets are also adjacent in `adjacent` if they overlap with each other
    [adjacency_sharpened,adj] = get_adjacency(lanelets,semi_adj);

    % `intersection_lanelets`
    % get indecies of intersection lanelets
    intersection_lanelets = get_intersection_lanelets(commonroad_data.intersection);

    % `lanelets_boundary`
    lanelet_boundary = get_lanelet_boundary(commonroad_data,lanelets);
    
    % save all the road data offline
    save(road_full_path,...
        'lanelets','adjacency_sharpened','semi_adjacency_sharpened','intersection_lanelets', 'commonroad','lanelet_boundary',...
        '-mat')
end

%% function get_lannelets
function lanelets = get_lannelets(commonroad_data)
    Nlanelets = length(commonroad_data.lanelet); % number of lanelets
    lanelets = cell(1,Nlanelets);
    for i = 1: Nlanelets
        nPoints = length(horzcat(commonroad_data.lanelet(i).leftBound.point.x));
        l = zeros(nPoints,LaneletInfo.nCols);
        l(:,LaneletInfo.rx) = horzcat(commonroad_data.lanelet(i).rightBound.point.x);
        l(:,LaneletInfo.ry) = horzcat(commonroad_data.lanelet(i).rightBound.point.y);
        l(:,LaneletInfo.lx) = horzcat(commonroad_data.lanelet(i).leftBound.point.x);
        l(:,LaneletInfo.ly) = horzcat(commonroad_data.lanelet(i).leftBound.point.y);
        l(:,LaneletInfo.cx) = 1/2*(l(:,LaneletInfo.lx)+l(:,LaneletInfo.rx));
        l(:,LaneletInfo.cy) = 1/2*(l(:,LaneletInfo.ly)+l(:,LaneletInfo.ry));
        lanelets{i} = l;
    end
end

%% function get_semi_adjacency
function [semi_adjacency_sharpened,semi_adj] = get_semi_adjacency(commonroad_data)
    Nlanelets = length(commonroad_data.lanelet); % number of lanelets
    semi_adj = zeros(Nlanelets,Nlanelets);
    for i = 1:Nlanelets
        predecessor = commonroad_data.lanelet(i).predecessor;
        successor = commonroad_data.lanelet(i).successor;
        adjacent_index = [];
        
        if isfield(predecessor,'refAttribute')
            predecessor_index = horzcat(predecessor.refAttribute);
            adjacent_index = [adjacent_index,predecessor_index];
        end
        
        if isfield(successor,'refAttribute')
            successor_index = horzcat(successor.refAttribute);
            adjacent_index = [adjacent_index,successor_index];
        end
    
        semi_adj(i,adjacent_index) = 1;
    
    end
    semi_adjacency_sharpened = semi_adj + semi_adj' + eye(Nlanelets, Nlanelets); % same lanelet is always adjacent
    semi_adjacency_sharpened = (semi_adjacency_sharpened > 0);
end

%% function get_adjacent
function [adjacency_sharpened,adj] = get_adjacency(lanelets,adj_semi)
    
    adj = adj_semi;
    Nlanelets = length(lanelets); % number of lanelets
    shrink_fac = 0.85; % shrink factor to avoid two lanelets being considered as overlapping if they only share one same side
    for i=1:Nlanelets
        for j=i+1:Nlanelets
            middle_point_idx_i = round(size(lanelets{1,i},1)/2); % middle point index
            
            % Only extract six points to form a polygon to approximate the lenelet
            % to save computation time
            x_i = [lanelets{1,i}(1,LaneletInfo.lx);lanelets{1,i}(middle_point_idx_i,LaneletInfo.lx);lanelets{1,i}(end,LaneletInfo.lx);...
                lanelets{1,i}(end,LaneletInfo.rx);lanelets{1,i}(middle_point_idx_i,LaneletInfo.rx);lanelets{1,i}(1,LaneletInfo.rx)];
            y_i = [lanelets{1,i}(1,LaneletInfo.ly);lanelets{1,i}(middle_point_idx_i,LaneletInfo.ly);lanelets{1,i}(end,LaneletInfo.ly);...
                lanelets{1,i}(end,LaneletInfo.ry);lanelets{1,i}(middle_point_idx_i,LaneletInfo.ry);lanelets{1,i}(1,LaneletInfo.ry)];
            % Shrink the polygon to avoid two lanelets being considered as overlapping if they only share one same side
            x_i_shrink = shrink_fac*(x_i-mean(x_i))+mean(x_i);
            y_i_shrink = shrink_fac*(y_i-mean(y_i))+mean(y_i);
    %         poly_i = polyshape(x_i,y_i);
            poly_i_shrink = polyshape(x_i_shrink,y_i_shrink); % create polyshape to use the MATLAB `overlaps` function
            
            middle_point_idx_j = round(size(lanelets{1,j},1)/2); % middle point index
            
            % Only extract six points to form a polygon to approximate the lenelet
            % to save computation time
            x_j = [lanelets{1,j}(1,LaneletInfo.lx);lanelets{1,j}(middle_point_idx_j,LaneletInfo.lx);lanelets{1,j}(end,LaneletInfo.lx);...
                lanelets{1,j}(end,LaneletInfo.rx);lanelets{1,j}(middle_point_idx_j,LaneletInfo.rx);lanelets{1,j}(1,LaneletInfo.rx)];
            y_j = [lanelets{1,j}(1,LaneletInfo.ly);lanelets{1,j}(middle_point_idx_j,LaneletInfo.ly);lanelets{1,j}(end,LaneletInfo.ly);...
                lanelets{1,j}(end,LaneletInfo.ry);lanelets{1,j}(middle_point_idx_j,LaneletInfo.ry);lanelets{1,j}(1,LaneletInfo.ry)];
            % Shrink the polygon to avoid two lanelets being considered as overlapping if they only share one same side
            x_j_shrink = shrink_fac*(x_j-mean(x_j))+mean(x_j);
            y_j_shrink = shrink_fac*(y_j-mean(y_j))+mean(y_j);
    %         poly_j = polyshape(x_j,y_j);
            poly_j_shrink = polyshape(x_j_shrink,y_j_shrink); % create polyshape to use the MATLAB `overlaps` function
    
            if overlaps(poly_i_shrink,poly_j_shrink)
                adj(i,j) = 1;
            end
        end
    end
    
    adjacency_sharpened = adj + adj' + eye(Nlanelets, Nlanelets); % same lanelet is always adjacent
    adjacency_sharpened = (adjacency_sharpened > 0);   
end

%% function get_intersection_lanelets
function intersection_lanelets = get_intersection_lanelets(intersection)
    Nintersections = length(intersection); 
    for i = 1:Nintersections
        intersection_lanelets = [];
        for n = 1:length(intersection(i).incoming)
            intersection_lanelets = [intersection_lanelets,horzcat(intersection(i).incoming(n).successorsRight.refAttribute)];
            intersection_lanelets = [intersection_lanelets,horzcat(intersection(i).incoming(n).successorsLeft.refAttribute)];
            intersection_lanelets = [intersection_lanelets,horzcat(intersection(i).incoming(n).successorsStraight.refAttribute)];            
        end     
    end
end

% %% function get_lanelet_boundary
% function lanelet_boundary_sharpened = get_lanelet_boundary(commonroad_data)
% % get the boundary of one lanelet, and the union of the boundaries of both
% % this lanelet and its successor's boundary in case one lanelet is too
% % short
% 
%     Nlanelets = length(commonroad_data.lanelet); % number of lanelets
%     lanelet_boundary_sharpened = struct;
%     count = 1;
%     n_sample = 8; % number of sample points in the bounds (to reduce computation time of intercting of reachable sets and boundaries when online planning trajectory)
%     
%     for iLanelet = 1:Nlanelets
%         successors = [commonroad_data.lanelet(iLanelet).successor.refAttribute]; % get all the successors of the current lanelet
% 
%         % current lanelet
%         bound_num = num2str(iLanelet); % number of boundary/boundaries
%         lanelet_boundary_sharpened(count).laneletNum = bound_num; 
% 
%         % get left boundary data
%         left_bound_x = [commonroad_data.lanelet(iLanelet).leftBound.point.x]';
%         left_bound_y = [commonroad_data.lanelet(iLanelet).leftBound.point.y]';
%         left_bound = [left_bound_x,left_bound_y];
%         left_bound_sampled = sample_evenly(left_bound,n_sample,1);
% 
%         % get right boundary data
%         right_bound_x = [commonroad_data.lanelet(iLanelet).rightBound.point.x]';
%         right_bound_y = [commonroad_data.lanelet(iLanelet).rightBound.point.y]';
%         right_bound = [right_bound_x,right_bound_y];
%         right_bound_sampled = sample_evenly(right_bound,n_sample,1);
%         
%         % save boundary data
%         lanelet_boundary_sharpened(count).pointsSampled = [left_bound_sampled;right_bound_sampled(end:-1:1,:)];
%         % convert boundary data to polyshape
%         lanelet_boundary_sharpened(count).pointsSampledPoly = polyshape([left_bound_sampled;right_bound_sampled(end:-1:1,:)]);
%         
%         count = count + 1;
%         % current lanelet and its successor
%         for iSuccessor=successors
%             bound_num = [num2str(iLanelet),'-',num2str(iSuccessor)]; % number of boundary/boundaries
%             lanelet_boundary_sharpened(count).laneletNum = bound_num; 
% 
%             % get left boundary data
%             left_bound_x = [commonroad_data.lanelet(iLanelet).leftBound.point.x, commonroad_data.lanelet(iSuccessor).leftBound.point.x]';
%             left_bound_y = [commonroad_data.lanelet(iLanelet).leftBound.point.y, commonroad_data.lanelet(iSuccessor).leftBound.point.y]';
%             left_bound = [left_bound_x,left_bound_y];
%             left_bound_sampled = sample_evenly(left_bound,n_sample,1);
% 
%             % get right boundary data
%             right_bound_x = [commonroad_data.lanelet(iLanelet).rightBound.point.x, commonroad_data.lanelet(iSuccessor).rightBound.point.x]';
%             right_bound_y = [commonroad_data.lanelet(iLanelet).rightBound.point.y, commonroad_data.lanelet(iSuccessor).rightBound.point.y]';
%             right_bound = [right_bound_x,right_bound_y];
%             right_bound_sampled = sample_evenly(right_bound,n_sample,1);
%             
%             % save boundary data
%             lanelet_boundary_sharpened(count).pointsSampled = [left_bound_sampled;right_bound_sampled(end:-1:1,:)];
%             % convert boundary data to polyshape
%             lanelet_boundary_sharpened(count).pointsSampledPoly = polyshape([left_bound_sampled;right_bound_sampled(end:-1:1,:)]);
%             
%             count = count + 1;
%         end
%     end
% end
% %% function get_lanelet_boundary
% function lanelet_boundary_sharpened = get_lanelet_boundary(commonroad_data)
% % get the boundaries of lanelets (do not consider adjacent lanelets'
% % boundaries)
% 
%     Nlanelets = length(commonroad_data.lanelet); % number of lanelets
%     lanelet_boundary_sharpened = cell(1,Nlanelets);
%     n_sample = 4; % number of sample points in the bounds (to reduce computation time of intercting of reachable sets and boundaries when online planning trajectory)
%     
%     for iLanelet = 1:Nlanelets
%         left_bound = [horzcat(commonroad_data.lanelet(iLanelet).leftBound.point.x)',horzcat(commonroad_data.lanelet(iLanelet).leftBound.point.y)'];
%         left_bound_sampled = sample_evenly(left_bound,n_sample,1);
%         right_bound = [horzcat(commonroad_data.lanelet(iLanelet).rightBound.point.x)',horzcat(commonroad_data.lanelet(iLanelet).rightBound.point.y)'];
%         right_bound_sampled = sample_evenly(right_bound,n_sample,1);
%         bound_sampled = [left_bound_sampled;right_bound_sampled(end:-1:1,:)];
%         lanelet_boundary_sharpened{iLanelet} = {left_bound,right_bound};
%         lanelet_boundary_sharpened{iLanelet}{end+1} = polyshape(bound_sampled);
%     end
% end
%% function get_lanelet_boundary
function lanelet_boundary = get_lanelet_boundary(commonroad_data,lanelets)
    Nlanelets = length(lanelets);
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
end