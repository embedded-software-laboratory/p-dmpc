function [lanelets,adjacency,intersection_lanelets,boundary,commonroad] = commonroad_lanelets()

% COMMONROAD_LANELETS  returns the lanelets information, collision pair matrix and the boundary information

% Returns:
% lanelets： lanelet information of rightBound, leftBound, and central line of each lanelet
% collision: (nLanelets x nLanelets) matrix, entry is 1: two lanelets collide or adjacent to each other
% intersection_lanelets: lanelet index of the intersection
% boundary: the lanelets boundary of commonroad, includes inner boundary and outer boundary


    %% lanelets 
%     commonroad_data = readstruct('LabMapCommonRoad.xml');
%     save('commonroad_data.mat','commonroad_data')

%     commonroad_data = readstruct('LabMapCommonRoad_Update.xml');
%     save('commonroad_data.mat','commonroad_data')
    
    load('commonroad_data.mat');
    

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
     
    commonroad = commonroad_data;
    %% commonroad scenario boundary 
                     
    boundary = cell(1,0);
    
    % use the rightBound of the lanelets as the inner boundary
    % lanelets index of the inner boundary
    inner_boundary_lanelets = {[14,16,22,23,10,12,18] ... 
                               [49,48,42,40,44,38,36] ...  
                               [88,90,96,92,94,100,101] ... 
                               [70,64,62,75,74,68,66]};
    
    for n_inner_boundary = 1:size(inner_boundary_lanelets,2)
        boundary_segment = [];
        for nlanelets = 1:size(inner_boundary_lanelets{n_inner_boundary},2)
            boundary_x = (horzcat(commonroad_data.lanelet(inner_boundary_lanelets{n_inner_boundary}(nlanelets)).rightBound.point.x));
            boundary_y = (horzcat(commonroad_data.lanelet(inner_boundary_lanelets{n_inner_boundary}(nlanelets)).rightBound.point.y));

            boundary_next = [boundary_x(1:end);boundary_y(1:end)];
            boundary_segment = [boundary_segment, boundary_next];
        end
        boundary{end+1}=boundary_segment;
    
    end  
    
    % use the leftBound of the lanelets as the outer boundary
    
    % lanelets index of the outer boundary
    outer_boundary_lanelets = {[2,4,6,8,60,58,56,54,80,82,84,86,34,32,30,28]};    
    for n_outer_boundary = 1:size(outer_boundary_lanelets,2)
        boundary_segment = [];
        for nlanelets = 1:size(outer_boundary_lanelets{n_outer_boundary},2)
            boundary_x = (horzcat(commonroad_data.lanelet(outer_boundary_lanelets{n_outer_boundary}(nlanelets)).leftBound.point.x));
            boundary_y = (horzcat(commonroad_data.lanelet(outer_boundary_lanelets{n_outer_boundary}(nlanelets)).leftBound.point.y));

            boundary_next = [boundary_x(1:end);boundary_y(1:end)];
            boundary_segment = [boundary_segment, boundary_next];
        end
        boundary{end+1}=boundary_segment;
    
    end
        
      
    % use the leftBound of the lanelets as the outer turning boundary
    
    % lanelets index of the outer turning boundary
    turning_boundary_lanelets = {[11],[63],[13],[39],[37],[89],[65],[91]};
    for n_turning_boundary = 1:size(turning_boundary_lanelets,2)
        boundary_segment = [];
        for nlanelets = 1:size(turning_boundary_lanelets{n_turning_boundary},2)
            
            boundary_x = (horzcat(commonroad_data.lanelet(turning_boundary_lanelets{n_turning_boundary}(nlanelets)).leftBound.point.x));
            boundary_y = (horzcat(commonroad_data.lanelet(turning_boundary_lanelets{n_turning_boundary}(nlanelets)).leftBound.point.y));
            boundary_next = [boundary_x(1:end);boundary_y(1:end)];
            boundary_segment = [boundary_segment, boundary_next];
            
        end
        boundary{end+1}=boundary_segment;
    
    end  
       
    
%     % plot the boundary
%     n = length(boundary);
%     figure
%     for i = 1:n
%         hold on
%         plot(boundary{1,i}(1,:),boundary{1,i}(2,:))
%     end
%     
    
    %% adjacency pair matrix
    
    adj = zeros(Nlanelets, Nlanelets);
 
    % check the predecessor, successor, adjacentLeft, and adjacentRight of the current lanelet, and assign the pair lanelets to be adjacent
    for i = 1:Nlanelets
        predecessor = commonroad_data.lanelet(i).predecessor;
        successor = commonroad_data.lanelet(i).successor;
        adjacentLeft = commonroad_data.lanelet(i).adjacentLeft;
        adjacentRight = commonroad_data.lanelet(i).adjacentRight;
        adjacent_index = [];
        
        if isfield(predecessor,'refAttribute')
            predecessor_index = horzcat(predecessor.refAttribute);
            adjacent_index = [adjacent_index,predecessor_index];
            
            % the adjacentLeft and adjacentRight of predecessor are also considered to be adjacent
            for index = predecessor_index
                predecessor_adjacentLeft = commonroad_data.lanelet(index).adjacentLeft;
                predecessor_adjacentRight = commonroad_data.lanelet(index).adjacentRight;
                predecessor_successor = commonroad_data.lanelet(index).successor;
                
                if isfield(predecessor_adjacentLeft,'refAttribute')
                   predecessor_adjacentLeft_index = horzcat(predecessor_adjacentLeft.refAttribute);
                   adjacent_index = [adjacent_index,predecessor_adjacentLeft_index];
                end
                if isfield(predecessor_adjacentRight,'refAttribute')
                   predecessor_adjacentRight_index = horzcat(predecessor_adjacentRight.refAttribute);
                   adjacent_index = [adjacent_index,predecessor_adjacentRight_index];
                end
                
                % check the successor of predecessor to include forking lanelets
                if isfield(predecessor_successor,'refAttribute')
                   predecessor_successor_index = horzcat(predecessor_successor.refAttribute);
                   adjacent_index = [adjacent_index,predecessor_successor_index];
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
  
                if isfield(successor_adjacentLeft,'refAttribute')
                   successor_adjacentLeft_index = horzcat(successor_adjacentLeft.refAttribute);
                   adjacent_index = [adjacent_index,successor_adjacentLeft_index];
                end
                if isfield(successor_adjacentRight,'refAttribute')
                   successor_adjacentRight_index = horzcat(successor_adjacentRight.refAttribute);
                   adjacent_index = [adjacent_index,successor_adjacentRight_index];
                end
                
                % check the predecessor of successor to include merging lanelets
                if isfield(successor_predecessor,'refAttribute')
                   successor_predecessor_index = horzcat(successor_predecessor.refAttribute);
                   
                   % successor_predecessor's adjacentLeft and adjacentRight
                   
                   
                   
                   
                   adjacent_index = [adjacent_index,successor_predecessor_index];
                end
                
            end
            
        end
        
        if isfield(adjacentLeft,'refAttribute')
            adjacentLeft_index = horzcat(adjacentLeft.refAttribute);
            adjacent_index = [adjacent_index,adjacentLeft_index];
        end
        
        if isfield(adjacentRight,'refAttribute')
            adjacentRight_index = horzcat(adjacentRight.refAttribute);
            adjacent_index = [adjacent_index,adjacentRight_index];
        end

        adj(i,adjacent_index) = 1;
     
    end
    
    
    
    
    
    
    
    
    
    
    
    
    
       
    % check the lanelets at intersection and assign them to be adjacent
    
    Nintersections = length(commonroad_data.intersection); 
    
    for i = 1:Nintersections
        intersection_lanelets = [];
        for n = 1:length(commonroad_data.intersection(i).incoming)
            intersection_lanelets = [intersection_lanelets,horzcat(commonroad_data.intersection(i).incoming(n).successorsRight.refAttribute)];
            intersection_lanelets = [intersection_lanelets,horzcat(commonroad_data.intersection(i).incoming(n).successorsLeft.refAttribute)];
            intersection_lanelets = [intersection_lanelets,horzcat(commonroad_data.intersection(i).incoming(n).successorsStraight.refAttribute)];            
        end
%         disp(intersection_lanelets)
        for k =1:length(intersection_lanelets)
            for l = length(intersection_lanelets) : -1 : k+1
                adj(intersection_lanelets(k),intersection_lanelets(l)) = 1;
            end
        end
        
    end
    
    
    adjacency = adj + adj' + eye(Nlanelets, Nlanelets); % same lanelet is always adjacent
    adjacency = (adjacency>0);
    
%     save('adjacency.mat','adjacency')
    
end



