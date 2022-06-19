classdef RoadData
    %ROADDATA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        lanelets
        adjacency_lanelets
        semi_adjacency_lanelets
        intersection_lanelets
        lanelet_boundary
        road_raw_data
        lanelet_relationships
        road_name
        road_full_path
    end
    
    methods
        function obj = RoadData()
        end

        function road_data = get_road_data(road_data, road_name)
            % initialize
            if nargin==1
                road_data.road_name = 'commonroad_lanelets.mat'; % default road name
            else
                assert(isstring(road_name) || ischar(road_name))
                road_data.road_name = road_name;
            end

            % path of the road data
            [file_path,~,~] = fileparts(mfilename('fullpath'));
            folder_target = [file_path,filesep,'offline_road_data'];
            if ~isfolder(folder_target)
                % create target folder if not exist
                mkdir(folder_target)
            end
            road_data.road_full_path = [folder_target,filesep,road_data.road_name];

            %% if the needed road data alread exist, simply load it, otherwise they will be calculated and saved.
            if isfile(road_data.road_full_path)
                load(road_data.road_full_path,'road_data');
                return
            end
            %% else load and preprocess the raw data and save them at the end

            % read raw road data
            road_data.road_raw_data = readstruct('LabMapCommonRoad_Update.xml');
            
            % get lanelet coordinates
            road_data.lanelets = road_data.get_lanelets();
        
            % get adjacent lanelet relationships with `adjacency_lanelets` and `semi_adjacency_lanelets` as byproducts
            [road_data.lanelet_relationships,road_data.adjacency_lanelets,road_data.semi_adjacency_lanelets] = road_data.get_lanelet_relationships();
        
            % get lanelet boundaries
            road_data.lanelet_boundary = road_data.get_lanelet_boundary();
        
            % get IDs of lanelets at the intersection
            road_data.intersection_lanelets = road_data.get_intersection_lanelets();

            % save all the road data
            save(road_data.road_full_path,'road_data','-mat')

        end
    end


    methods (Access = private)
    % helper functions to preprocess the raw road data
        function lanelets = get_lanelets(obj)
        % returns the coordinates of the left boundary, right boundary and center line
            nLanelets = length(obj.road_raw_data.lanelet); % number of lanelets
            lanelets = cell(1,nLanelets);
            for i = 1: nLanelets
                lanelets{i}(:,LaneletInfo.rx) = horzcat(obj.road_raw_data.lanelet(i).rightBound.point.x);
                lanelets{i}(:,LaneletInfo.ry) = horzcat(obj.road_raw_data.lanelet(i).rightBound.point.y);
                lanelets{i}(:,LaneletInfo.lx) = horzcat(obj.road_raw_data.lanelet(i).leftBound.point.x);
                lanelets{i}(:,LaneletInfo.ly) = horzcat(obj.road_raw_data.lanelet(i).leftBound.point.y);
                lanelets{i}(:,LaneletInfo.cx) = 1/2*(lanelets{i}(:,LaneletInfo.lx)+lanelets{i}(:,LaneletInfo.rx));
                lanelets{i}(:,LaneletInfo.cy) = 1/2*(lanelets{i}(:,LaneletInfo.ly)+lanelets{i}(:,LaneletInfo.ry));
            end
        end

        function [lanelet_relationships,adjacency_lenelets,semi_adjacency_lanelets] = get_lanelet_relationships(obj)
        % Determine the relationship between each lanelet-pair.
        % We classify five types of relationship.
        % 1. successive 
        % 2. adjacent left or right in the same direction
        % 3. merging 
        % 4. forking
        % 5. intersecting
        % For vehicle pairs drive in the lanelet pairs with different relationship types, collision types are also different.
        % We classify two types of collision types: 
        % 1. rear-end collision, which can happen if two lanelets have relationship type 1 or 5. (Relationship 5 could cause 
        % rear-end collision when one vehicle switching to its adjacent parallel lanelet with a higher velocity 
        % than the front vehicle which is already on this lanelet.)
        % 2. side-impact collision, which can happen if two lanelets have relationship type 2 or 4. 

            lanelet_relationships = struct('ID_1',[],'ID_2',[],'type',[],'point',[]); 
            count = 1;
            nLanelets = length(obj.lanelets);
            adjacency_lenelets = ones(nLanelets,nLanelets);
            semi_adjacency_lanelets = ones(nLanelets,nLanelets);
            road_lanelets = obj.road_raw_data.lanelet;
            
            for i=1:nLanelets-1
                [adjacentLeft_i, adjacentRight_i,...
                    predecessors_i, predecessors_adjacentLeft_i, predecessors_adjacentRight_i,...
                    successors_i, successors_adjacentLeft_i, successors_adjacentRight_i] = obj.get_all_adjacent_lanelets(i, road_lanelets);
        
                for j=i+1:nLanelets % no reapted check
        %             if i==29 && j==48
        %                 disp('debug')
        %             end
        
                    % store lanelet IDs
                    lanelet_relationships(count).ID_1 = i; 
                    lanelet_relationships(count).ID_2 = j; 
        
                    % initialize variables for predecessors, successors, adjacent left and adjacent right lanelet of lanelet_j
                    [adjacentLeft_j, adjacentRight_j, predecessors_j, ~, ~,successors_j, ~, ~] = obj.get_all_adjacent_lanelets(j, road_lanelets);
                    
                    % check relationship 
                    if sum(ismember(j,predecessors_i))>=1
                        % lanelet_j is the predecessor of lanelet_i -> successive lanelets 
                        lanelet_relationships(count).type = LaneletRelationshipType.type_1;
                        % store the endpoint of the front lanelet
                        lanelet_relationships(count).point = [obj.lanelets{i}(end,LaneletInfo.cx),obj.lanelets{i}(end,LaneletInfo.cy)];
                    elseif sum(ismember(i,predecessors_j))>=1
                        % lanelet_i is the predecessor of lanelet_j -> successive lanelets 
                        lanelet_relationships(count).type = LaneletRelationshipType.type_1;
                        % store the endpoint of the front lanelet
                        lanelet_relationships(count).point = [obj.lanelets{j}(end,LaneletInfo.cx),obj.lanelets{j}(end,LaneletInfo.cy)];
                    elseif sum(ismember(j,predecessors_adjacentLeft_i))>=1
                        % lanelet_j is the adjacent left lanelet of lanelet_i's predecessor -> successive lanelets
                        lanelet_relationships(count).type = LaneletRelationshipType.type_1;
                        % store the endpoint of the front lanelet
                        lanelet_relationships(count).point = [obj.lanelets{i}(end,LaneletInfo.lx),obj.lanelets{i}(end,LaneletInfo.ly)];
                    elseif sum(ismember(j,successors_adjacentLeft_i))>=1
                        % lanelet_j is the adjacent left lanelet of lanelet_i's successor -> successive lanelets
                        lanelet_relationships(count).type = LaneletRelationshipType.type_1;
                        % store the endpoint of the front lanelet
                        lanelet_relationships(count).point = [obj.lanelets{j}(end,LaneletInfo.rx),obj.lanelets{j}(end,LaneletInfo.ry)];
                    elseif sum(ismember(j,predecessors_adjacentRight_i))>=1
                        % lanelet_j is the adjacent right lanelet of lanelet_i's predecessor -> successive lanelets
                        lanelet_relationships(count).type = LaneletRelationshipType.type_1;
                        % store the endpoint of the front lanelet
                        lanelet_relationships(count).point = [obj.lanelets{i}(end,LaneletInfo.rx),obj.lanelets{i}(end,LaneletInfo.ry)];
                    elseif sum(ismember(j,successors_adjacentRight_i))>=1
                        % lanelet_j is the adjacent right lanelet of lanelet_i's successor -> successive lanelets
                        lanelet_relationships(count).type = LaneletRelationshipType.type_1;
                        % store the endpoint of the front lanelet
                        lanelet_relationships(count).point = [obj.lanelets{j}(end,LaneletInfo.lx),obj.lanelets{j}(end,LaneletInfo.ly)];
                    elseif ~isempty(adjacentLeft_i) && j==adjacentLeft_i
                        % lanelet_j is the lanelet_i's adjacent left lanelet -> adjacent left/right lanelets in the same direction
                        lanelet_relationships(count).type = LaneletRelationshipType.type_2;
                        % store left boundary's endpoint of the lanelet_i
                        lanelet_relationships(count).point = [obj.lanelets{i}(end,LaneletInfo.lx),obj.lanelets{i}(end,LaneletInfo.ly)];
                    elseif ~isempty(adjacentRight_i) && j==adjacentRight_i
                        % lanelet_j is the lanelet_i's adjacent right lanelet -> adjacent left/right lanelets in the same direction
                        lanelet_relationships(count).type = LaneletRelationshipType.type_2;
                        % store right boundary's endpoint of the lanelet_i
                        lanelet_relationships(count).point = [obj.lanelets{i}(end,LaneletInfo.rx),obj.lanelets{i}(end,LaneletInfo.ry)];
                    elseif ~isempty(adjacentLeft_i) && ~isempty(adjacentRight_j) && adjacentLeft_i==adjacentRight_j
                        % a lanelet is in the middle of the target two lanelets and they are all in the same direction -> adjacent left/right lanelets in the same direction
                        lanelet_relationships(count).type = LaneletRelationshipType.type_2;
                        % which point to store is not interesting for lanelets with this kind of relationship
                        lanelet_relationships(count).point = [obj.lanelets{i}(end,LaneletInfo.lx),obj.lanelets{i}(end,LaneletInfo.ly)];
                    elseif ~isempty(adjacentRight_i) && ~isempty(adjacentLeft_j) && adjacentRight_i==adjacentLeft_j
                        % a lanelet is in the middle of the target two lanelets and they are all in the same direction -> adjacent left/right lanelets in the same direction
                        lanelet_relationships(count).type = LaneletRelationshipType.type_2;
                        % which point to store is not interesting for lanelets with this kind of relationship
                        lanelet_relationships(count).point = [obj.lanelets{i}(end,LaneletInfo.lx),obj.lanelets{i}(end,LaneletInfo.ly)];
                    elseif sum(ismember(successors_i,successors_j))>=1
                        % two lanelets have the same successor -> merging lanelets 
                        lanelet_relationships(count).type = LaneletRelationshipType.type_3;
                        % store the merging point (endpoint of one lanelet's center line) 
                        lanelet_relationships(count).point = [obj.lanelets{i}(end,LaneletInfo.cx),obj.lanelets{i}(end,LaneletInfo.cy)];
                    elseif sum(ismember(successors_j,successors_adjacentLeft_i))>=1 && sum(ismember(predecessors_i,predecessors_j))==0
                        % the successor of lanelet_j is the adjacent left lanelet of lanelet_i's successor && they do not have the same predecessor -> merging lanelets
                        lanelet_relationships(count).type = LaneletRelationshipType.type_3;
                        % store the merging point (endpoint of lenelet_j's right boundary) 
                        lanelet_relationships(count).point = [obj.lanelets{j}(end,LaneletInfo.rx),obj.lanelets{j}(end,LaneletInfo.ry)];
                    elseif sum(ismember(successors_j,successors_adjacentRight_i))>=1 && sum(ismember(predecessors_i,predecessors_j))==0
                        % the successor of lanelet_j is the adjacent right lanelet of lanelet_i's successor && they do not have the same predecessor -> merging lanelets
                        lanelet_relationships(count).type = LaneletRelationshipType.type_3;
                        % store the merging point (endpoint of lenelet_j's left boundary)
                        lanelet_relationships(count).point = [obj.lanelets{j}(end,LaneletInfo.lx),obj.lanelets{j}(end,LaneletInfo.ly)];
                    elseif sum(ismember(predecessors_i,predecessors_j))>=1
                        % two lanelets have the same predecessor -> forking lanelets
                        lanelet_relationships(count).type = LaneletRelationshipType.type_4;
                        % store the forking point (starting point of one lanelet's center line) 
                        lanelet_relationships(count).point = [obj.lanelets{i}(1,LaneletInfo.cx),obj.lanelets{i}(1,LaneletInfo.cy)];
                    elseif sum(ismember(predecessors_j,predecessors_adjacentLeft_i))>=1 && sum(ismember(successors_i,successors_j))==0
                        % the predecessor of lanelet_j is the adjacent left lanelet of lanelet_i's predecessor && they do not have the same successor -> forking lanelets
                        lanelet_relationships(count).type = LaneletRelationshipType.type_4;
                        % store the forking point (starting point of lenelet_j's right boundary) 
                        lanelet_relationships(count).point = [obj.lanelets{j}(1,LaneletInfo.rx),obj.lanelets{j}(1,LaneletInfo.ry)];
                    elseif sum(ismember(predecessors_j,predecessors_adjacentRight_i))>=1 && sum(ismember(successors_i,successors_j))==0
                        % the predecessor of lanelet_j is the adjacent left lanelet of lanelet_i's predecessor && they do not have the same successor -> forking lanelets
                        lanelet_relationships(count).type = LaneletRelationshipType.type_4;
                        % store the forking point (starting point of lenelet_j's left boundary)
                        lanelet_relationships(count).point = [obj.lanelets{j}(1,LaneletInfo.lx),obj.lanelets{j}(1,LaneletInfo.ly)];
                    else
                        % if not the above cases, check whether the center lines of the two lanelets intersect with each other center line. If yes -> intersecting lanelets
                        x_c_i = obj.lanelets{i}(:,LaneletInfo.cx);
                        y_c_i = obj.lanelets{i}(:,LaneletInfo.cy);
                        x_c_j = obj.lanelets{j}(:,LaneletInfo.cx);
                        y_c_j = obj.lanelets{j}(:,LaneletInfo.cy);
                        % calculate the point of intersection of their center lines
                        [x_intersect,y_intersect] = polyxpoly(x_c_i,y_c_i,x_c_j,y_c_j);
                        if length(x_intersect)==1
                            % intersect
                            lanelet_relationships(count).type = LaneletRelationshipType.type_5;
                            lanelet_relationships(count).point = [x_intersect, y_intersect];
                            % If two lanelets interset, the corresponding entry of `semi_adjacency_lanelets` should be zero 
                            % This is also the only difference between `semi_adjacency_lanelets` and `adjacency_lanelets` 
                            semi_adjacency_lanelets(i,j) = 0;
                        else
                            % if not intersect, then the two lanelets have no relationship -> delete the last row which is created at the very begining of the inner for-loop
                            lanelet_relationships(end) = [];
                            count = count - 1;
                            % set the corresponding entry in the `adjacency_lanelets` and `semi_adjacency_lanelets` matrix to be zero
                            adjacency_lenelets(i,j) = 0;
                            semi_adjacency_lanelets(i,j) = 0;
                        end
                    end
                    count = count + 1;          
                end
            end
            % extract only the elements above the main diagonal, make the matrix symmetrical and add self as adjacency 
            adjacency_lenelets = triu(adjacency_lenelets,1) + triu(adjacency_lenelets,1)' + eye(nLanelets);
            semi_adjacency_lanelets = triu(semi_adjacency_lanelets,1) + triu(semi_adjacency_lanelets,1)' + eye(nLanelets);
        end

        function lanelet_boundary = get_lanelet_boundary(obj)
        % returns lanelet boundaries
            nLanelets = length(obj.lanelets);
            road_lanelets = obj.road_raw_data.lanelet;
        
            lanelet_boundary = cell(1,nLanelets);
            for i = 1:nLanelets 
                share_boundary_with = i; % which lanelets have the same boundary
        
                [adjacentLeft_i, adjacentRight_i, ~, predecessors_adjacentLeft_i, predecessors_adjacentRight_i,...
                    ~, successors_adjacentLeft_i, successors_adjacentRight_i] = obj.get_all_adjacent_lanelets(i, road_lanelets);
        
                % if the lanelet has adjacent left lanelet, the boundary should be 
                % the leftBound of adjacentLeft and rightBound of the current lanelet
                
                left_bound_x = obj.lanelets{i}(:,LaneletInfo.lx);
                left_bound_y = obj.lanelets{i}(:,LaneletInfo.ly);
        
                right_bound_x = obj.lanelets{i}(:,LaneletInfo.rx);
                right_bound_y = obj.lanelets{i}(:,LaneletInfo.ry);
                
                if ~isempty(adjacentLeft_i) && strcmp(road_lanelets(i).adjacentLeft.drivingDirAttribute,'same') 
                    left_bound_x = obj.lanelets{adjacentLeft_i}(:,LaneletInfo.lx);
                    left_bound_y = obj.lanelets{adjacentLeft_i}(:,LaneletInfo.ly);
                    share_boundary_with = [share_boundary_with,adjacentLeft_i];
        
                elseif ~isempty(adjacentRight_i) && strcmp(road_lanelets(i).adjacentRight.drivingDirAttribute,'same')
                    right_bound_x = obj.lanelets{adjacentRight_i}(:,LaneletInfo.rx);
                    right_bound_y = obj.lanelets{adjacentRight_i}(:,LaneletInfo.ry);
                    share_boundary_with = [share_boundary_with,adjacentRight_i];
                end
                
                % find all merging lanelets
                merging_lanelets = obj.find_adjacent_lanelets_with_certain_relationship(i,obj.lanelet_relationships,LaneletRelationshipType.type_3);
                % merging lanelets can be considered as adjacent lanelet when their predecessors are adjacent
                for i_merging = merging_lanelets 
                    [adjacentLeft_merging, adjacentRight_merging, predecessors_i_merging, ~, ~, ~, ~, ~] = obj.get_all_adjacent_lanelets(i_merging, road_lanelets);
                    if sum(ismember(predecessors_i_merging,predecessors_adjacentLeft_i))>=1
                        % the adjacent left lanelet of the current lanelet's predecessor is the merging lanelet's predecessor -> extend left boundary 
                        if ~isempty(adjacentLeft_merging) && strcmp(road_lanelets(i_merging).adjacentLeft.drivingDirAttribute,'same')
                            % if the merging lenelet has adjacent left lanelet ->
                            % extend the current lanelet's left boundary to the left boundary of the adjacent left lanelet of the merging lanelet 
                            left_bound_x = obj.lanelets{adjacentLeft_merging}(:,LaneletInfo.lx);
                            left_bound_y = obj.lanelets{adjacentLeft_merging}(:,LaneletInfo.ly);
                            share_boundary_with = [share_boundary_with,i_merging,adjacentLeft_merging];
                        else
                            % otherwise extend the current lanelet's left boundary to the left boundary of the merging lanelet 
                            left_bound_x = obj.lanelets{i_merging}(:,LaneletInfo.lx);
                            left_bound_y = obj.lanelets{i_merging}(:,LaneletInfo.ly);
                            share_boundary_with = [share_boundary_with,i_merging];
                        end                
                    end
        
                    if sum(ismember(predecessors_i_merging,predecessors_adjacentRight_i))>=1
                        % the adjacent right lanelet of the current lanelet's predecessor is the merging lanelet's predecessor -> extend right boundary 
                        if ~isempty(adjacentRight_merging) && strcmp(road_lanelets(i_merging).adjacentRight.drivingDirAttribute,'same')
                            % if the merging lenelet has adjacent right lanelet ->
                            % extend the current lanelet's right boundary to the right boundary of the adjacent right lanelet of the merging lanelet 
                            right_bound_x = obj.lanelets{adjacentRight_merging}(:,LaneletInfo.rx);
                            right_bound_y = obj.lanelets{adjacentRight_merging}(:,LaneletInfo.ry);
                            share_boundary_with = [share_boundary_with,i_merging,adjacentRight_merging];
                        else
                            % otherwise extend the current lanelet's right boundary to the right boundary of the merging lanelet 
                            right_bound_x = obj.lanelets{i_merging}(:,LaneletInfo.rx);
                            right_bound_y = obj.lanelets{i_merging}(:,LaneletInfo.ry);
                            share_boundary_with = [share_boundary_with,i_merging];
                        end                
                    end
        
                end
        
                % find all forking lanelets
                forking_lanelets = obj.find_adjacent_lanelets_with_certain_relationship(i,obj.lanelet_relationships,LaneletRelationshipType.type_4);
                % forking lanelets can be considered as adjacent lanelet when their successors are adjacent 
                for i_forking = forking_lanelets 
                    [adjacentLeft_forking, adjacentRight_forking, ~, ~, ~, successors_i_forking, ~, ~] = obj.get_all_adjacent_lanelets(i_forking, road_lanelets);
                    if sum(ismember(successors_i_forking,successors_adjacentLeft_i))>=1
                        % the adjacent left lanelet of the current lanelet's successor is the forking lanelet's successor -> extend left boundary 
                        if ~isempty(adjacentLeft_forking) && strcmp(road_lanelets(i_forking).adjacentLeft.drivingDirAttribute,'same')
                            % if the forking lenelet has adjacent left lanelet ->
                            % extend the current lanelet's left boundary to the left boundary of the adjacent left lanelet of the forking lanelet 
                            left_bound_x = obj.lanelets{adjacentLeft_forking}(:,LaneletInfo.lx);
                            left_bound_y = obj.lanelets{adjacentLeft_forking}(:,LaneletInfo.ly);
                            share_boundary_with = [share_boundary_with,i_forking,adjacentLeft_forking];
                        else
                            % otherwise extend the current lanelet's left boundary to the left boundary of the forking lanelet 
                            left_bound_x = obj.lanelets{i_forking}(:,LaneletInfo.lx);
                            left_bound_y = obj.lanelets{i_forking}(:,LaneletInfo.ly);
                            share_boundary_with = [share_boundary_with,i_forking];
                        end                
                    end
        
                    if sum(ismember(successors_i_forking,successors_adjacentRight_i))>=1
                        % the adjacent right lanelet of the current lanelet's successor is the forking lanelet's successor -> extend right boundary 
                        if ~isempty(adjacentRight_forking) && strcmp(road_lanelets(i_forking).adjacentRight.drivingDirAttribute,'same')
                            % if the forking lenelet has adjacent right lanelet ->
                            % extend the current lanelet's right boundary to the right boundary of the adjacent right lanelet of the forking lanelet 
                            right_bound_x = obj.lanelets{adjacentRight_forking}(:,LaneletInfo.rx);
                            right_bound_y = obj.lanelets{adjacentRight_forking}(:,LaneletInfo.ry);
                            share_boundary_with = [share_boundary_with,i_forking,adjacentRight_forking];
                        else
                            % otherwise extend the current lanelet's right boundary to the right boundary of the forking lanelet 
                            right_bound_x = obj.lanelets{i_forking}(:,LaneletInfo.rx);
                            right_bound_y = obj.lanelets{i_forking}(:,LaneletInfo.ry);
                            share_boundary_with = [share_boundary_with,i_forking];
                        end                
                    end
        
                end
                
                left_bound = [left_bound_x(:),left_bound_y(:)];
                right_bound = [right_bound_x(:),right_bound_y(:)];
           
                lanelet_boundary{i} = {left_bound,right_bound};
        
                % convert to MATLAB polyshape object for later use (set 'Simplify' to true to remove colinear points)        
                bound_x = [left_bound_x;right_bound_x(end:-1:1)];
                bound_y = [left_bound_y;right_bound_y(end:-1:1)];
                lanelet_boundary{i}{3} = polyshape(bound_x,bound_y,'Simplify',true);
        
        %         % Narrowed lanelet boundaries, used to check if two vehicles' predicted lanelets boundaries overlap. 
        %         % Without narrowing, two lanelets are considered as overlap if they only share the same boundary
        %         narrow_fac = 0.98;
        %         bound_x_narrowed = narrow_fac*(bound_x-mean(bound_x)) + mean(bound_x);
        %         bound_y_narrowed = narrow_fac*(bound_y-mean(bound_y)) + mean(bound_y);
        %         lanelet_boundary{i}{4} = polyshape(bound_x_narrowed,bound_y_narrowed,'Simplify',true);
            end 
        end

        function intersection_lanelets = get_intersection_lanelets(obj)
        % returns the IDs of the lanelets at the intersection 
            
            nIntersections = length(obj.road_raw_data.intersection); 
            for i = 1:nIntersections
                intersection_lanelets = [];
                for n = 1:length(obj.road_raw_data.intersection(i).incoming)
                    intersection_lanelets = [intersection_lanelets,horzcat(obj.road_raw_data.intersection(i).incoming(n).successorsRight.refAttribute)];
                    intersection_lanelets = [intersection_lanelets,horzcat(obj.road_raw_data.intersection(i).incoming(n).successorsLeft.refAttribute)];
                    intersection_lanelets = [intersection_lanelets,horzcat(obj.road_raw_data.intersection(i).incoming(n).successorsStraight.refAttribute)];            
                end
            end
        end



    end


   methods (Static)
       function adjacent_lanelets_with_certain_relationship = find_adjacent_lanelets_with_certain_relationship(current_ID,lanelet_relationships,relationship)
        % returns adjacent lanelets with a certain relationship to the current lanelt
            % current lanelet could be found in the field `ID_1` and `ID_2`
            current_lanelets_idx_1 = find([lanelet_relationships.ID_1]==current_ID);
            current_lanelets_idx_2 = find([lanelet_relationships.ID_2]==current_ID);
            adjacent_lanelets = [lanelet_relationships(current_lanelets_idx_2).ID_1,lanelet_relationships(current_lanelets_idx_1).ID_2];
            current_lanelets_idx = [current_lanelets_idx_2,current_lanelets_idx_1];
            find_lanelet_by_relationship = strcmp({lanelet_relationships(current_lanelets_idx).type},relationship);
            adjacent_lanelets_with_certain_relationship = adjacent_lanelets(find_lanelet_by_relationship);
       end

        function [adjacentLeft_i,adjacentRight_i,...
            predecessors_i, predecessors_adjacentLeft_i, predecessors_adjacentRight_i,...
            successors_i, successors_adjacentLeft_i, successors_adjacentRight_i] = get_all_adjacent_lanelets(i, road_lanelets)
        % Returns all the adjacent lanelets of the given lanelet_i
        
            % initialize variables for predecessors, successors, adjacent left and adjacent right lanelet of lanelet_i
            predecessors_i = []; successors_i = []; adjacentLeft_i = []; adjacentRight_i = [];
            predecessors_adjacentLeft_i = []; predecessors_adjacentRight_i = [];
            successors_adjacentLeft_i = []; successors_adjacentRight_i = [];
        
            % get the adjacent left and right lanelet in the same direction
            if isfield(road_lanelets(i).adjacentLeft,'refAttribute') && strcmp(road_lanelets(i).adjacentLeft.drivingDirAttribute,"same")
                % one lanelet has maximal one adjacent left lanelet, thus no `vertcat` is needed
                adjacentLeft_i = road_lanelets(i).adjacentLeft.refAttribute;
            end
            if isfield(road_lanelets(i).adjacentRight,'refAttribute') && strcmp(road_lanelets(i).adjacentRight.drivingDirAttribute,"same")
                adjacentRight_i = road_lanelets(i).adjacentRight.refAttribute;
            end
        
            % get predecessors and the adjacent left and right lanelets of the predecessors in the same direction
            if isfield(road_lanelets(i).predecessor,'refAttribute')
                predecessors_i = vertcat(road_lanelets(i).predecessor.refAttribute);
                for predecessor_idx_i=1:length(predecessors_i)
                    predecessor_i = predecessors_i(predecessor_idx_i);
                    if isfield(road_lanelets(predecessor_i).adjacentLeft,'refAttribute')...
                            && strcmp(road_lanelets(predecessor_i).adjacentLeft.drivingDirAttribute,"same")
                        predecessors_adjacentLeft_i = [predecessors_adjacentLeft_i,road_lanelets(predecessor_i).adjacentLeft.refAttribute;];
                    end
                    if isfield(road_lanelets(predecessor_i).adjacentRight,'refAttribute')...
                            && strcmp(road_lanelets(predecessor_i).adjacentRight.drivingDirAttribute,"same")
                        predecessors_adjacentRight_i = [predecessors_adjacentRight_i,road_lanelets(predecessor_i).adjacentRight.refAttribute;];
                    end
                end
            end
            
            % get successors and the adjacent left and right lanelets of the successors in the same direction
            if isfield(road_lanelets(i).successor,'refAttribute')
                successors_i = vertcat(road_lanelets(i).successor.refAttribute);
                for successor_idx_i=1:length(successors_i)
                    successor_i = successors_i(successor_idx_i);
                    if isfield(road_lanelets(successor_i).adjacentLeft,'refAttribute')...
                            && strcmp(road_lanelets(successor_i).adjacentLeft.drivingDirAttribute,"same")
                        successors_adjacentLeft_i = [successors_adjacentLeft_i,road_lanelets(successor_i).adjacentLeft.refAttribute;];
                    end
                    if isfield(road_lanelets(successor_i).adjacentRight,'refAttribute')...
                            && strcmp(road_lanelets(successor_i).adjacentRight.drivingDirAttribute,"same")
                        successors_adjacentRight_i = [successors_adjacentRight_i,road_lanelets(successor_i).adjacentRight.refAttribute;];
                    end
                end
            end
        end

        function idx_lanelet_pair = find_idx_lanelet_pair(predicted_lanelet_i, predicted_lanelet_j, lanelet_relationships)
            % First try to find predicted_lanelet_i in the field `ID_1` and
            % predicted_lanelet_j in the field `ID_2`. If not find, do it again in
            % the opposite way.
            idx_lanelet_pair = [];
            id_i_j = double([predicted_lanelet_i; predicted_lanelet_j]);
            id_j_i = double([predicted_lanelet_j; predicted_lanelet_i]);
            IDs_1_2 = double([lanelet_relationships.ID_1;lanelet_relationships.ID_2]);
        
            if isempty(idx_lanelet_pair)
                idx_lanelet_pair = find(all((IDs_1_2-id_i_j)==0,1));
            end
            if isempty(idx_lanelet_pair)
                idx_lanelet_pair = find(all((IDs_1_2-id_j_i)==0,1));
            end
        end

        
   end

end

