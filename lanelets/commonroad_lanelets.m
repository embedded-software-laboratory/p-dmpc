function [lanelets,boundary] = commonroad_lanelets()

% COMMONROAD_LANELETS  returns the lanelets information and the boundary information(outer boundary and inner boundary) of the CPM LAB. 
% Each lanelet contains information of leftBound, rightBound, and the centralLine.

    % commonroad_data = readstruct('LabMapCommonRoad.xml');
    % % % save('commonroad_data.mat','commonroad_data')

    load('commonroad_data.mat')

    lanelets = cell(1,0);
    Nlanelets = length(horzcat(commonroad_data.lanelet.idAttribute)); % number of lanelets


    for i = 1: Nlanelets

        nPoints = length(horzcat(commonroad_data.lanelet(i).leftBound.point.x));
        l = zeros(nPoints,LaneletInfo.nCols);
        l(:,LaneletInfo.rx) = horzcat(commonroad_data.lanelet(i).rightBound.point.x);% rightBound
        l(:,LaneletInfo.ry) = horzcat(commonroad_data.lanelet(i).rightBound.point.y);
        l(:,LaneletInfo.lx) = horzcat(commonroad_data.lanelet(i).leftBound.point.x);
        l(:,LaneletInfo.ly) = horzcat(commonroad_data.lanelet(i).leftBound.point.y);
        l(:,LaneletInfo.cx) = 1/2*(l(:,LaneletInfo.lx)+l(:,LaneletInfo.rx));
        l(:,LaneletInfo.cy) = 1/2*(l(:,LaneletInfo.ly)+l(:,LaneletInfo.ry));
        lanelets{end+1} = l;

    end
    
    
    %% commonroad scenario boundary 
    
    % lanelets index of the inner boundary
    inner_boundary_lanelets = {[26,30,38,39,18,22,34] ... 
                               [81,80,72,68,76,64,60] ...  
                               [144,148,160,152,156,164,165] ... 
                               [118,106,102,123,122,114,110]};

    % lanelets index of the inner boundary
    outer_boundary_lanelets = {[2,6,10,14,98,94,90,86,128,132,136,140,56,52,48,44]};
    
    boundary = cell(1,0);
    
    % use the rightBound of the lanelets as the inner boundary
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
    
    % lanelets index of the inner boundary
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
    
    
%     boundary_lanelets = [26,30,38,39,18,22,34; 
%                 81,80,72,68,76,64,60;  
%                 144,148,160,152,156,164,165; 
%                 118,106,102,123,122,114,110];
%             
%     obstacles = cell(1,0);
%     
% 
%     for nobstacles = 1:size(boundary_lanelets,1)
%         boundary = [];
%         for nlanelets = 1:size(boundary_lanelets(nobstacles,:),2)
%             boundary_x = (horzcat(commonroad_data.lanelet(boundary_lanelets(nobstacles,nlanelets)).rightBound.point.x));
%             boundary_y = (horzcat(commonroad_data.lanelet(boundary_lanelets(nobstacles,nlanelets)).rightBound.point.y));
% 
%         %             boundary_next = [boundary_x([1,end]);boundary_y([1,end])];
%             boundary_next = [boundary_x(1:end);boundary_y(1:end)];
%             boundary = [boundary, boundary_next]
%         end
%         obstacles{end+1}=boundary;
%     end 
   


end



