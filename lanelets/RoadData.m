classdef RoadData
    %ROADDATA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        lanelets                % 1-by-nLanelets cell array, stores the coordinates of left bound, right bound, and centerline of each lanelet
        adjacency_lanelets      % nLanelets-by-nLanelets matrix. Entry is true iff two lanelets are adjacent
        semi_adjacency_lanelets % nLanelets-by-nLanelets matrix. Entry is true iff two lanelets are semi-adjacent
        intersection_lanelets   % indices of lanelets of intersections
        lanelet_boundary        % 1-by-nLanelet cell array with each subcell being a 1-by-3 cell array. The first and second subcells store the left and right boundary of each lanelet.
                                % The third subcell stores an object of MATLAB class `polyshape`. 
        road_raw_data           % road data before preprocessing
        lanelet_relationships   % nLanelet-by-nLanelets cell array, stores the relationship of two adjacent lanelets. Empty if they are adjacent
        road_name               % road name
        road_full_path          % path where the offline data is/should be stored
    end
    
    methods
        function obj = RoadData()
        end

        function road_data = get_road_data(road_data, road_name)
            % initialize
            if nargin==1
                road_data.road_name = 'commonroad_lanelets_advanced.mat'; % default road name
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
        
            % get IDs of lanelets at the intersection
            road_data.intersection_lanelets = road_data.get_intersection_lanelets();

            % get adjacent lanelet relationships with `adjacency_lanelets` and `semi_adjacency_lanelets` as byproducts
            [road_data.lanelet_relationships,road_data.adjacency_lanelets,road_data.semi_adjacency_lanelets] = road_data.get_lanelet_relationships();
        
            % get lanelet boundaries
            [road_data.lanelet_boundary,share_boundary_with] = road_data.get_lanelet_boundary();

            % update lanelet relationships
            road_data = road_data.update_lanelet_relationships(share_boundary_with);

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
        % 1. crossing 
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

            nLanelets = length(obj.lanelets);
            lanelet_relationships = cell(nLanelets,nLanelets);
            adjacency_lenelets = ones(nLanelets,nLanelets);
            semi_adjacency_lanelets = ones(nLanelets,nLanelets);
            road_lanelets = obj.road_raw_data.lanelet;
            
            for i=1:nLanelets-1
                [adjacentLeft_i, adjacentRight_i,...
                    predecessors_i, predecessors_adjacentLeft_i, predecessors_adjacentRight_i,...
                    successors_i, successors_adjacentLeft_i, successors_adjacentRight_i] = obj.get_all_adjacent_lanelets(i, road_lanelets);
        
                for j=i+1:nLanelets % no reapted check
                    if ~isempty(lanelet_relationships{i,j})
                        continue
                    end

                    % initialize variables for predecessors, successors, adjacent left and adjacent right lanelet of lanelet_j
                    [adjacentLeft_j, adjacentRight_j, predecessors_j, ~, ~,successors_j, ~, ~] = obj.get_all_adjacent_lanelets(j, road_lanelets);
                    
                    % check relationship 
                    if any(ismember(j,predecessors_i))
                        % lanelet_j is the predecessor of lanelet_i -> logitudinal-adjacent lanelets 
                        lanelet_relationships{i,j}.type = LaneletRelationshipType.type_1;
                        % store the endpoint of the front lanelet
                        lanelet_relationships{i,j}.point = [obj.lanelets{i}(end,LaneletInfo.cx),obj.lanelets{i}(end,LaneletInfo.cy)];
                    elseif any(ismember(i,predecessors_j))
                        % lanelet_i is the predecessor of lanelet_j -> logitudinal-adjacent lanelets 
                        lanelet_relationships{i,j}.type = LaneletRelationshipType.type_1;
                        % store the endpoint of the front lanelet
                        lanelet_relationships{i,j}.point = [obj.lanelets{j}(end,LaneletInfo.cx),obj.lanelets{j}(end,LaneletInfo.cy)];
                    elseif any(ismember(adjacentLeft_i,predecessors_j)) || any(ismember(adjacentRight_i,predecessors_j))
                        % lanelet_i's adjacent lanelet is the predecessor of lanelet_j -> logitudinal-adjacent lanelets 
                        lanelet_relationships{i,j}.type = LaneletRelationshipType.type_1;
                        % store the endpoint of the front lanelet
                        lanelet_relationships{i,j}.point = [obj.lanelets{j}(end,LaneletInfo.cx),obj.lanelets{j}(end,LaneletInfo.cy)];
                    elseif any(ismember(adjacentLeft_i,successors_j)) || any(ismember(adjacentRight_i,successors_j))
                        % lanelet_i's adjacent lanelet is the successor of lanelet_j -> logitudinal-adjacent lanelets 
                        lanelet_relationships{i,j}.type = LaneletRelationshipType.type_1;
                        % store the endpoint of the front lanelet
                        lanelet_relationships{i,j}.point = [obj.lanelets{i}(end,LaneletInfo.cx),obj.lanelets{i}(end,LaneletInfo.cy)];
                    elseif any(ismember(j,predecessors_adjacentLeft_i))
                        % lanelet_j is the adjacent left lanelet of lanelet_i's predecessor -> logitudinal-adjacent lanelets
                        lanelet_relationships{i,j}.type = LaneletRelationshipType.type_1;
                        % store the endpoint of the front lanelet
                        lanelet_relationships{i,j}.point = [obj.lanelets{i}(end,LaneletInfo.lx),obj.lanelets{i}(end,LaneletInfo.ly)];
                    elseif any(ismember(j,successors_adjacentLeft_i))
                        % lanelet_j is the adjacent left lanelet of lanelet_i's successor -> logitudinal-adjacent lanelets
                        lanelet_relationships{i,j}.type = LaneletRelationshipType.type_1;
                        % store the endpoint of the front lanelet
                        lanelet_relationships{i,j}.point = [obj.lanelets{j}(end,LaneletInfo.rx),obj.lanelets{j}(end,LaneletInfo.ry)];
                    elseif any(ismember(j,predecessors_adjacentRight_i))
                        % lanelet_j is the adjacent right lanelet of lanelet_i's predecessor -> logitudinal-adjacent lanelets
                        lanelet_relationships{i,j}.type = LaneletRelationshipType.type_1;
                        % store the endpoint of the front lanelet
                        lanelet_relationships{i,j}.point = [obj.lanelets{i}(end,LaneletInfo.rx),obj.lanelets{i}(end,LaneletInfo.ry)];
                    elseif any(ismember(j,successors_adjacentRight_i))
                        % lanelet_j is the adjacent right lanelet of lanelet_i's successor -> logitudinal-adjacent lanelets
                        lanelet_relationships{i,j}.type = LaneletRelationshipType.type_1;
                        % store the endpoint of the front lanelet
                        lanelet_relationships{i,j}.point = [obj.lanelets{j}(end,LaneletInfo.lx),obj.lanelets{j}(end,LaneletInfo.ly)];
                    elseif ~isempty(adjacentLeft_i) && j==adjacentLeft_i
                        % lanelet_j is the lanelet_i's adjacent left lanelet -> adjacent left/right lanelets in the same direction
                        lanelet_relationships{i,j}.type = LaneletRelationshipType.type_2;
                        % store left boundary's endpoint of the lanelet_i
                        lanelet_relationships{i,j}.point = [obj.lanelets{i}(end,LaneletInfo.lx),obj.lanelets{i}(end,LaneletInfo.ly)];
                    elseif ~isempty(adjacentRight_i) && j==adjacentRight_i
                        % lanelet_j is the lanelet_i's adjacent right lanelet -> adjacent left/right lanelets in the same direction
                        lanelet_relationships{i,j}.type = LaneletRelationshipType.type_2;
                        % store right boundary's endpoint of the lanelet_i
                        lanelet_relationships{i,j}.point = [obj.lanelets{i}(end,LaneletInfo.rx),obj.lanelets{i}(end,LaneletInfo.ry)];
                    elseif ~isempty(adjacentLeft_i) && ~isempty(adjacentRight_j) && adjacentLeft_i==adjacentRight_j
                        % a lanelet is in the middle of the target two lanelets and they are all in the same direction -> adjacent left/right lanelets in the same direction
                        lanelet_relationships{i,j}.type = LaneletRelationshipType.type_2;
                        % which point to store is not interesting for lanelets with this kind of relationship
                        lanelet_relationships{i,j}.point = [obj.lanelets{i}(end,LaneletInfo.lx),obj.lanelets{i}(end,LaneletInfo.ly)];
                    elseif ~isempty(adjacentRight_i) && ~isempty(adjacentLeft_j) && adjacentRight_i==adjacentLeft_j
                        % a lanelet is in the middle of the target two lanelets and they are all in the same direction -> adjacent left/right lanelets in the same direction
                        lanelet_relationships{i,j}.type = LaneletRelationshipType.type_2;
                        % which point to store is not interesting for lanelets with this kind of relationship
                        lanelet_relationships{i,j}.point = [obj.lanelets{i}(end,LaneletInfo.lx),obj.lanelets{i}(end,LaneletInfo.ly)];
                    elseif any(ismember(successors_i,successors_j))
                        % two lanelets have the same successor -> merging lanelets 
                        lanelet_relationships{i,j}.type = LaneletRelationshipType.type_3;
                        % store the merging point (endpoint of one lanelet's center line) 
                        lanelet_relationships{i,j}.point = [obj.lanelets{i}(end,LaneletInfo.cx),obj.lanelets{i}(end,LaneletInfo.cy)];
                        
                        % Their adjacent left/right lanelets are also merging lanelets if exist
                        i_s = [i,adjacentLeft_i,adjacentRight_i];
                        j_s = [j,adjacentLeft_j,adjacentRight_j];
                        for ii = i_s
                            for jj = j_s
                                ij = [ii,jj];
                                if all(~ismember(ij,obj.intersection_lanelets)) && ~(ii==i && jj==j)
%                                     disp(ij)
                                    lanelet_relationships(min(ij),max(ij)) = lanelet_relationships(i,j);
                                end
                            end
                        end
                    elseif any(ismember(successors_j,successors_adjacentLeft_i)) && all(~ismember(predecessors_i,predecessors_j))
                        % the successor of lanelet_j is the adjacent left lanelet of lanelet_i's successor && they do not have the same predecessor -> merging lanelets
                        lanelet_relationships{i,j}.type = LaneletRelationshipType.type_3;
                        % store the merging point (endpoint of lenelet_j's right boundary) 
                        lanelet_relationships{i,j}.point = [obj.lanelets{j}(end,LaneletInfo.rx),obj.lanelets{j}(end,LaneletInfo.ry)];
                    elseif any(ismember(successors_j,successors_adjacentRight_i)) && all(~ismember(predecessors_i,predecessors_j))
                        % the successor of lanelet_j is the adjacent right lanelet of lanelet_i's successor && they do not have the same predecessor -> merging lanelets
                        lanelet_relationships{i,j}.type = LaneletRelationshipType.type_3;
                        % store the merging point (endpoint of lenelet_j's left boundary)
                        lanelet_relationships{i,j}.point = [obj.lanelets{j}(end,LaneletInfo.lx),obj.lanelets{j}(end,LaneletInfo.ly)];
                    elseif any(ismember(predecessors_i,predecessors_j))
                        % two lanelets have the same predecessor -> forking lanelets
                        lanelet_relationships{i,j}.type = LaneletRelationshipType.type_4;
                        % store the forking point (starting point of one lanelet's center line) 
                        lanelet_relationships{i,j}.point = [obj.lanelets{i}(1,LaneletInfo.cx),obj.lanelets{i}(1,LaneletInfo.cy)];
                        % Their adjacent left/right lanelets are also forking lanelets if exist
                        i_s = [i,adjacentLeft_i,adjacentRight_i];
                        j_s = [j,adjacentLeft_j,adjacentRight_j];
                        for ii = i_s
                            for jj = j_s
                                ij = [ii,jj];
                                if all(~ismember(ij,obj.intersection_lanelets)) && ~(ii==i && jj==j)
%                                     disp(ij)
                                    lanelet_relationships(min(ij),max(ij)) = lanelet_relationships(i,j);
                                end
                            end
                        end
                    elseif any(ismember(predecessors_j,predecessors_adjacentLeft_i)) && all(~ismember(successors_i,successors_j))
                        % the predecessor of lanelet_j is the adjacent left lanelet of lanelet_i's predecessor && they do not have the same successor -> forking lanelets
                        lanelet_relationships{i,j}.type = LaneletRelationshipType.type_4;
                        % store the forking point (starting point of lenelet_j's right boundary) 
                        lanelet_relationships{i,j}.point = [obj.lanelets{j}(1,LaneletInfo.rx),obj.lanelets{j}(1,LaneletInfo.ry)];
                    elseif any(ismember(predecessors_j,predecessors_adjacentRight_i)) && all(~ismember(successors_i,successors_j))
                        % the predecessor of lanelet_j is the adjacent left lanelet of lanelet_i's predecessor && they do not have the same successor -> forking lanelets
                        lanelet_relationships{i,j}.type = LaneletRelationshipType.type_4;
                        % store the forking point (starting point of lenelet_j's left boundary)
                        lanelet_relationships{i,j}.point = [obj.lanelets{j}(1,LaneletInfo.lx),obj.lanelets{j}(1,LaneletInfo.ly)];
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
                            lanelet_relationships{i,j}.type = LaneletRelationshipType.type_5;
                            lanelet_relationships{i,j}.point = [x_intersect, y_intersect];
                            % If two lanelets interset, the corresponding entry of `semi_adjacency_lanelets` should be zero 
                            % This is also the only difference between `semi_adjacency_lanelets` and `adjacency_lanelets` 
                            semi_adjacency_lanelets(i,j) = 0;
                        else
                            % if not intersect, then the two lanelets have no relationship -> delete the entry that was created at the very begining of the inner for-loop
                            lanelet_relationships{i,j} = [];
                            % set the corresponding entry in the `adjacency_lanelets` and `semi_adjacency_lanelets` matrix to be zero
                            adjacency_lenelets(i,j) = 0;
                            semi_adjacency_lanelets(i,j) = 0;
                        end
                    end     
                end                    
            end
            % extract only the elements above the main diagonal, make the matrix symmetrical and add self as adjacency 
            adjacency_lenelets = triu(adjacency_lenelets,1) + triu(adjacency_lenelets,1)' + eye(nLanelets);
            semi_adjacency_lanelets = triu(semi_adjacency_lanelets,1) + triu(semi_adjacency_lanelets,1)' + eye(nLanelets);
        end

        function [lanelet_boundary_extended, share_boundary_with] = get_lanelet_boundary(obj)
        % returns lanelet boundaries
            nLanelets = length(obj.lanelets);
            road_lanelets = obj.road_raw_data.lanelet;
            share_boundary_with = cell(nLanelets,1);
        
            lanelet_boundary_tmp = cell(1,nLanelets);
            for i = 1:nLanelets 
                share_boundary_with_i = i; % which lanelets have the same boundary
        
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
                    share_boundary_with_i = [share_boundary_with_i,adjacentLeft_i];
        
                elseif ~isempty(adjacentRight_i) && strcmp(road_lanelets(i).adjacentRight.drivingDirAttribute,'same')
                    right_bound_x = obj.lanelets{adjacentRight_i}(:,LaneletInfo.rx);
                    right_bound_y = obj.lanelets{adjacentRight_i}(:,LaneletInfo.ry);
                    share_boundary_with_i = [share_boundary_with_i,adjacentRight_i];
                end
                
                % find all merging lanelets
                merging_lanelets = obj.find_adjacent_lanelets_with_certain_relationship(i,obj.lanelet_relationships,LaneletRelationshipType.type_3);
                % merging lanelets can be considered as adjacent lanelet when their predecessors are adjacent
                for i_merging = merging_lanelets 
                    [adjacentLeft_merging, adjacentRight_merging, predecessors_i_merging, ~, ~, ~, ~, ~] = obj.get_all_adjacent_lanelets(i_merging, road_lanelets);
                    if any(ismember(predecessors_i_merging,predecessors_adjacentLeft_i))
                        % the adjacent left lanelet of the current lanelet's predecessor is the merging lanelet's predecessor -> extend left boundary 
                        if ~isempty(adjacentLeft_merging) && strcmp(road_lanelets(i_merging).adjacentLeft.drivingDirAttribute,'same')
                            % if the merging lenelet has adjacent left lanelet ->
                            % extend the current lanelet's left boundary to the left boundary of the adjacent left lanelet of the merging lanelet 
                            left_bound_x = obj.lanelets{adjacentLeft_merging}(:,LaneletInfo.lx);
                            left_bound_y = obj.lanelets{adjacentLeft_merging}(:,LaneletInfo.ly);
                            share_boundary_with_i = [share_boundary_with_i,i_merging,adjacentLeft_merging];
                        else
                            % otherwise extend the current lanelet's left boundary to the left boundary of the merging lanelet 
                            left_bound_x = obj.lanelets{i_merging}(:,LaneletInfo.lx);
                            left_bound_y = obj.lanelets{i_merging}(:,LaneletInfo.ly);
                            share_boundary_with_i = [share_boundary_with_i,i_merging];
                        end                
                    end
        
                    if any(ismember(predecessors_i_merging,predecessors_adjacentRight_i))
                        % the adjacent right lanelet of the current lanelet's predecessor is the merging lanelet's predecessor -> extend right boundary 
                        if ~isempty(adjacentRight_merging) && strcmp(road_lanelets(i_merging).adjacentRight.drivingDirAttribute,'same')
                            % if the merging lenelet has adjacent right lanelet ->
                            % extend the current lanelet's right boundary to the right boundary of the adjacent right lanelet of the merging lanelet 
                            right_bound_x = obj.lanelets{adjacentRight_merging}(:,LaneletInfo.rx);
                            right_bound_y = obj.lanelets{adjacentRight_merging}(:,LaneletInfo.ry);
                            share_boundary_with_i = [share_boundary_with_i,i_merging,adjacentRight_merging];
                        else
                            % otherwise extend the current lanelet's right boundary to the right boundary of the merging lanelet 
                            right_bound_x = obj.lanelets{i_merging}(:,LaneletInfo.rx);
                            right_bound_y = obj.lanelets{i_merging}(:,LaneletInfo.ry);
                            share_boundary_with_i = [share_boundary_with_i,i_merging];
                        end                
                    end
        
                end
        
                % find all forking lanelets
                forking_lanelets = obj.find_adjacent_lanelets_with_certain_relationship(i,obj.lanelet_relationships,LaneletRelationshipType.type_4);
                % forking lanelets can be considered as adjacent lanelet when their successors are adjacent 
                for i_forking = forking_lanelets 
                    [adjacentLeft_forking, adjacentRight_forking, ~, ~, ~, successors_i_forking, ~, ~] = obj.get_all_adjacent_lanelets(i_forking, road_lanelets);
                    if any(ismember(successors_i_forking,successors_adjacentLeft_i))
                        % the adjacent left lanelet of the current lanelet's successor is the forking lanelet's successor -> extend left boundary 
                        if ~isempty(adjacentLeft_forking) && strcmp(road_lanelets(i_forking).adjacentLeft.drivingDirAttribute,'same')
                            % if the forking lenelet has adjacent left lanelet ->
                            % extend the current lanelet's left boundary to the left boundary of the adjacent left lanelet of the forking lanelet 
                            left_bound_x = obj.lanelets{adjacentLeft_forking}(:,LaneletInfo.lx);
                            left_bound_y = obj.lanelets{adjacentLeft_forking}(:,LaneletInfo.ly);
                            share_boundary_with_i = [share_boundary_with_i,i_forking,adjacentLeft_forking];
                        else
                            % otherwise extend the current lanelet's left boundary to the left boundary of the forking lanelet 
                            left_bound_x = obj.lanelets{i_forking}(:,LaneletInfo.lx);
                            left_bound_y = obj.lanelets{i_forking}(:,LaneletInfo.ly);
                            share_boundary_with_i = [share_boundary_with_i,i_forking];
                        end                
                    end
        
                    if any(ismember(successors_i_forking,successors_adjacentRight_i))
                        % the adjacent right lanelet of the current lanelet's successor is the forking lanelet's successor -> extend right boundary 
                        if ~isempty(adjacentRight_forking) && strcmp(road_lanelets(i_forking).adjacentRight.drivingDirAttribute,'same')
                            % if the forking lenelet has adjacent right lanelet ->
                            % extend the current lanelet's right boundary to the right boundary of the adjacent right lanelet of the forking lanelet 
                            right_bound_x = obj.lanelets{adjacentRight_forking}(:,LaneletInfo.rx);
                            right_bound_y = obj.lanelets{adjacentRight_forking}(:,LaneletInfo.ry);
                            share_boundary_with_i = [share_boundary_with_i,i_forking,adjacentRight_forking];
                        else
                            % otherwise extend the current lanelet's right boundary to the right boundary of the forking lanelet 
                            right_bound_x = obj.lanelets{i_forking}(:,LaneletInfo.rx);
                            right_bound_y = obj.lanelets{i_forking}(:,LaneletInfo.ry);
                            share_boundary_with_i = [share_boundary_with_i,i_forking];
                        end                
                    end
        
                end
                
                left_bound = [left_bound_x(:),left_bound_y(:)];
                right_bound = [right_bound_x(:),right_bound_y(:)];
           
                lanelet_boundary_tmp{i} = {left_bound,right_bound};
        
                % convert to MATLAB polyshape object for later use (set 'Simplify' to true to remove colinear points)        
                bound_x = [left_bound_x;right_bound_x(end:-1:1)];
                bound_y = [left_bound_y;right_bound_y(end:-1:1)];
                lanelet_boundary_tmp{i}{3} = polyshape(bound_x,bound_y,'Simplify',true);

                share_boundary_with{i} = sort(share_boundary_with_i,'ascend');
            end

            % if two lanelets share some part of area, they should have
            % the same boundary
            for j = 1:nLanelets
                share_boundary_with_j = share_boundary_with{j};
                find_lanelets_share_boundary_with_j = find(cellfun(@(c) ismember(j,c),share_boundary_with));
                lanelets_all_j = unique([share_boundary_with{find_lanelets_share_boundary_with_j}]);
                lanelets_all_j = sort(lanelets_all_j,'ascend');
                find_lanelet_with_same_boundary = find(cellfun(@(c) length(lanelets_all_j)==length(c)&&all(lanelets_all_j==c),share_boundary_with));
                for k = find_lanelet_with_same_boundary(:)'
                    if length(share_boundary_with_j)<length(share_boundary_with{k})
                        % exclude self and lanelet with exactly the same
                        % lanelet boundary
                        lanelet_boundary_tmp(j) = lanelet_boundary_tmp(k);
                        share_boundary_with(j) = share_boundary_with(k);
                        break
                    end
                end
            end

            % If the closing (opening) of merging (forking) lanelets are
            % different, make them become the same to avoid sudden change
            % of lanelet boundaries
            
            lanelet_boundary_extended = lanelet_boundary_tmp;
            is_extend = false(nLanelets,1);
            for iL = 1:nLanelets
                if is_extend(iL)
                    % avoid repeated entending
                    continue
                end
                lanelet_boundary_i = lanelet_boundary_tmp{iL};

                % find merging lanelets
                iMs = obj.find_adjacent_lanelets_with_certain_relationship(iL,obj.lanelet_relationships,LaneletRelationshipType.type_3);
                for iM_idx=1:length(iMs)
                    iM = iMs(iM_idx);
                    lanelet_boundary_iM = lanelet_boundary_tmp{iM};
                    if norm(lanelet_boundary_i{1}(end,:)-lanelet_boundary_iM{1}(end,:))>1e-2 || norm(lanelet_boundary_i{2}(end,:)-lanelet_boundary_iM{2}(end,:))>1e-2
                        % If left end points or right end points are not identical
                        is_extend(iL) = true;
                        % Find the point of intersection of their lanelet boundaries
                        % Its own left boundary and merging lanelet's right boundary
                        POI_l_r = InterX(lanelet_boundary_i{1}',lanelet_boundary_iM{2}',true);
                        if ~isempty(POI_l_r)
                            assert(size(POI_l_r,2)==1) % if multiple POI, code adaption is needed
                            % Left-right boundary intersect
                            % straight line connecting the POI and the end
                            % point of the left boundary of the merging
                            % lanelet
                            straight_line = [POI_l_r,lanelet_boundary_iM{1}(end,:)'];
                            % check if the line intersects with the its own left lanelet boundary
                            POI_line_left = InterX(lanelet_boundary_i{1}',straight_line,true);
                            % delete the POI if it is very closed to the POI 
                            % of its own right boundary and merging lanelet's left boundary
                            points_deleted = false(1,size(POI_line_left,2));
                            for iPOI = 1:size(POI_line_left,2)
                                distance_POI = sqrt(sum((POI_line_left-POI_l_r).^2,1));
                                if distance_POI < 1e-2
                                    points_deleted(iPOI) = true;
                                end
                            end
                            POI_line_left = POI_line_left(:,~points_deleted);

                            if ~isempty(POI_line_left)
                                if size(POI_line_left,2)>1
                                    % if multiple POI, delete the one that is very colsed to the POI 
                                    % of its own left boundary and merging lanelet's right boundary
                                    [~,idx_del] = min(sum((POI_line_left-POI_l_r).^2,1));
                                    POI_line_left(:,idx_del) = [];
                                    assert(size(POI_line_left,2)==1)
                                end
                                [~,idx_min_2] = mink(sum((lanelet_boundary_i{1}'-POI_line_left).^2,1),2);
                                assert(abs(idx_min_2(1)-idx_min_2(2))==1)
                                n_sample = size(lanelet_boundary_i{1},1) - min(idx_min_2);
                                x_sampled = linspace(POI_line_left(1),lanelet_boundary_iM{1}(end,1),n_sample);
                                y_sampled = linspace(POI_line_left(2),lanelet_boundary_iM{1}(end,2),n_sample);
                            else
                                [~,idx_min_2] = mink(sum((lanelet_boundary_i{1}'-POI_l_r).^2,1),2);
                                assert(abs(idx_min_2(1)-idx_min_2(2))==1)
                                n_sample = size(lanelet_boundary_i{1},1) - min(idx_min_2);
                                x_sampled = linspace(POI_l_r(1),lanelet_boundary_iM{1}(end,1),n_sample);
                                y_sampled = linspace(POI_l_r(2),lanelet_boundary_iM{1}(end,2),n_sample);
                            end
                            lanelet_boundary_i{1} = [lanelet_boundary_i{1}(1:min(idx_min_2),:);[x_sampled;y_sampled]'];
                        end

                        % its own right boundary and merging lanelet's left boundary
                        POI_r_l = InterX(lanelet_boundary_i{2}',lanelet_boundary_iM{1}',true);
                        if ~isempty(POI_r_l)
                            assert(size(POI_r_l,2)==1) % if multiple POI, code adaption is needed
                            % right-left boundary intersect
                            % straight line connecting the POI and the end
                            % point of the left boundary of the merging
                            % lanelet
                            straight_line = [POI_r_l,lanelet_boundary_iM{2}(end,:)'];

                            % check if the line intersects with the its own right lanelet boundary
                            POI_line_right = InterX(lanelet_boundary_i{2}',straight_line,true);

                            % delete the POI if it is very closed to the POI 
                            % of its own right boundary and merging lanelet's left boundary
                            points_deleted = false(1,size(POI_line_right,2));
                            for iPOI = 1:size(POI_line_right,2)
                                distance_POI = sqrt(sum((POI_line_right-POI_r_l).^2,1));
                                if distance_POI < 1e-2
                                    points_deleted(iPOI) = true;
                                end
                            end
                            POI_line_right = POI_line_right(:,~points_deleted);

                            if ~isempty(POI_line_right)
                                if size(POI_line_right,2)>1
                                    % if multiple POI, delete the one that is very colsed to the POI 
                                    % of its own left boundary and merging lanelet's right boundary
                                    [~,idx_del] = min(sum((POI_line_right-POI_r_l).^2,1));
                                    POI_line_right(:,idx_del) = [];
                                    assert(size(POI_line_right,2)==1)
                                end
                                [~,idx_min_2] = mink(sum((lanelet_boundary_i{2}'-POI_line_right).^2,1),2);
                                assert(abs(idx_min_2(1)-idx_min_2(2))==1)
                                n_sample = size(lanelet_boundary_i{2},1) - min(idx_min_2);
                                x_sampled = linspace(POI_line_right(1),lanelet_boundary_iM{2}(end,1),n_sample);
                                y_sampled = linspace(POI_line_right(2),lanelet_boundary_iM{2}(end,2),n_sample);
%                                 lanelet_boundary_i{2} = [lanelet_boundary_i{2}(1:min(idx_min_2),:);POI_line_right';lanelet_boundary_iM{2}(end,:)];
                            else
                                [~,idx_min_2] = mink(sum((lanelet_boundary_i{2}'-POI_r_l).^2,1),2);
                                assert(abs(idx_min_2(1)-idx_min_2(2))==1)
                                n_sample = size(lanelet_boundary_i{1},1) - min(idx_min_2);
                                x_sampled = linspace(POI_r_l(1),lanelet_boundary_iM{2}(end,1),n_sample);
                                y_sampled = linspace(POI_r_l(2),lanelet_boundary_iM{2}(end,2),n_sample);
%                                 lanelet_boundary_i{2} = [lanelet_boundary_i{2}(1:min(idx_min_2),:);POI_r_l';lanelet_boundary_iM{2}(end,:)];
                            end
                            lanelet_boundary_i{2} = [lanelet_boundary_i{2}(1:min(idx_min_2),:);[x_sampled;y_sampled]'];
                        end
                    end
                    break
                end
                
                % find forking lanelets
                iFs = obj.find_adjacent_lanelets_with_certain_relationship(iL,obj.lanelet_relationships,LaneletRelationshipType.type_4);
                for iF_idx=1:length(iFs)
                    iF = iFs(iF_idx);
                    lanelet_boundary_iF = lanelet_boundary_tmp{iF};
                    if norm(lanelet_boundary_i{1}(1,:)-lanelet_boundary_iF{1}(1,:))>1e-2 || norm(lanelet_boundary_i{2}(1,:)-lanelet_boundary_iF{2}(1,:))>1e-2
                        % If left start points or right start points are not identical
                        is_extend(iL) = true;
                        % Find the point of intersection of their lanelet boundaries
                        % Its own left boundary and forking lanelet's right boundary
                        POI_l_r = InterX(lanelet_boundary_i{1}',lanelet_boundary_iF{2}',true);
                        if ~isempty(POI_l_r)
                            assert(size(POI_l_r,2)==1) % if multiple POI, code adaption is needed
                            % Left-right boundary intersect
                            
                            % Straight line connecting the POI and the end
                            % point of the left boundary of the forking
                            % lanelet
                            straight_line = [POI_l_r,lanelet_boundary_iF{1}(1,:)'];
                            % check if the line intersects with the its own left lanelet boundary
                            POI_line_left = InterX(lanelet_boundary_i{1}',straight_line,true);
                            points_deleted = false(1,size(POI_line_left,2));
                            for iPOI = 1:size(POI_line_left,2)
                                distance_POI = sqrt(sum((POI_line_left-POI_l_r).^2,1));
                                if distance_POI < 1e-2
                                    points_deleted(iPOI) = true;
                                end
                            end
                            POI_line_left = POI_line_left(:,~points_deleted);

                            if ~isempty(POI_line_left)
                                if size(POI_line_left,2)>1
                                    % if multiple POI, delete the one that is very colsed to the POI 
                                    % of its own left boundary and forking lanelet's right boundary
                                    [~,idx_del] = min(sum((POI_line_left-POI_l_r).^2,1));
                                    POI_line_left(:,idx_del) = [];
                                    assert(size(POI_line_left,2)==1)
                                end
                                [~,idx_min_2] = mink(sum((lanelet_boundary_i{1}'-POI_line_left).^2,1),2);
                                assert(abs(idx_min_2(1)-idx_min_2(2))==1)
                                n_sample = min(idx_min_2);
                                x_sampled = linspace(lanelet_boundary_iF{1}(1,1),POI_line_left(1),n_sample);
                                y_sampled = linspace(lanelet_boundary_iF{1}(1,2),POI_line_left(2),n_sample);
%                                 lanelet_boundary_i{1} = [lanelet_boundary_iF{1}(1,:);POI_line_left';lanelet_boundary_i{1}(max(idx_min_2):end,:)];
                            else
                                [~,idx_min_2] = mink(sum((lanelet_boundary_i{1}'-POI_l_r).^2,1),2);
                                assert(abs(idx_min_2(1)-idx_min_2(2))==1)
                                n_sample = min(idx_min_2);
                                x_sampled = linspace(lanelet_boundary_iF{1}(1,1),POI_l_r(1),n_sample);
                                y_sampled = linspace(lanelet_boundary_iF{1}(1,2),POI_l_r(2),n_sample);
%                                 lanelet_boundary_i{1} = [lanelet_boundary_iF{1}(1,:);POI_l_r';lanelet_boundary_i{1}(max(idx_min_2):end,:)];
                            end
                            lanelet_boundary_i{1} = [[x_sampled;y_sampled]';lanelet_boundary_i{1}(max(idx_min_2):end,:)];
                        end

                        % its own right boundary and forking lanelet's left boundary
                        POI_r_l = InterX(lanelet_boundary_i{2}',lanelet_boundary_iF{1}',true);
                        if ~isempty(POI_r_l)
                            assert(size(POI_r_l,2)==1) % if multiple POI, code adaption is needed
                            % right-left boundary intersect
                            % straight line connecting the POI and the end
                            % point of the left boundary of the forking
                            % lanelet
                            straight_line = [POI_r_l,lanelet_boundary_iF{2}(1,:)'];

                            % check if the line intersects with the its own right lanelet boundary
                            POI_line_right = InterX(lanelet_boundary_i{2}',straight_line,true);

                            % delete the POI if it is very closed to the POI 
                            % of its own right boundary and forking lanelet's left boundary
                            points_deleted = false(1,size(POI_line_right,2));
                            for iPOI = 1:size(POI_line_right,2)
                                distance_POI = sqrt(sum((POI_line_right-POI_r_l).^2,1));
                                if distance_POI < 1e-2
                                    points_deleted(iPOI) = true;
                                end
                            end
                            POI_line_right = POI_line_right(:,~points_deleted);
                                    
                            if ~isempty(POI_line_right)
                                if size(POI_line_right,2)>1
                                    % if multiple POI, delete the one that is very colsed to the POI 
                                    % of its own left boundary and forking lanelet's right boundary
                                    POI_line_right(:,idx_del) = [];
                                    assert(size(POI_line_right,2)==1)
                                end
                                [~,idx_min_2] = mink(sum((lanelet_boundary_i{2}'-POI_line_right).^2,1),2);
                                assert(abs(idx_min_2(1)-idx_min_2(2))==1)
                                n_sample = min(idx_min_2);
                                x_sampled = linspace(lanelet_boundary_iF{2}(1,1),POI_line_right(1),n_sample);
                                y_sampled = linspace(lanelet_boundary_iF{2}(1,2),POI_line_right(2),n_sample);
%                                 lanelet_boundary_i{2} = [lanelet_boundary_iF{2}(1,:);POI_line_right';lanelet_boundary_i{2}(max(idx_min_2):end,:)];
                            else
                                [~,idx_min_2] = mink(sum((lanelet_boundary_i{2}'-POI_r_l).^2,1),2);
                                assert(abs(idx_min_2(1)-idx_min_2(2))==1)
                                n_sample = min(idx_min_2);
                                x_sampled = linspace(lanelet_boundary_iF{2}(1,1),POI_r_l(1),n_sample);
                                y_sampled = linspace(lanelet_boundary_iF{2}(1,2),POI_r_l(2),n_sample);
%                                 lanelet_boundary_i{2} = [lanelet_boundary_iF{2}(1,:);POI_r_l';lanelet_boundary_i{2}(max(idx_min_2):end,:)];
                            end
                            lanelet_boundary_i{2} = [[x_sampled;y_sampled]';lanelet_boundary_i{2}(max(idx_min_2):end,:)];
                        end
                    end
                    break
                end

                % extend lanelet boundary
                if is_extend(iL)
                    lanelet_boundary_extended{iL} = lanelet_boundary_i;
                    lanelet_boundary_extended{iL}{3} = polyshape([lanelet_boundary_i{1};lanelet_boundary_i{2}(end:-1:1,:)]);
                    % those lanelets sharing exactly the same boundary should
                    % also be extended
                    find_lan_share_same_boundary = find(cellfun(@(c) length(c)==length(share_boundary_with{iL})&&all(c==share_boundary_with{iL}),share_boundary_with));
                    find_lan_share_same_boundary = setdiff(find_lan_share_same_boundary,iL); % exclude self
                    if ~isempty(find_lan_share_same_boundary)
                        is_extend(find_lan_share_same_boundary) = true;
                        lanelet_boundary_extended(find_lan_share_same_boundary) = lanelet_boundary_extended(iL);
%                         plot(lanelet_boundary_extended{find_lan_share_same_boundary}{3})
                    end
                    % left boundary has the same number of points as the
                    % right boundary
                    assert(size(lanelet_boundary_extended{iL}{1},1)==size(lanelet_boundary_extended{iL}{2},1))
                end
%                 pause(1)
%                 plot(lanelet_boundary_extended{iL}{3})
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
        
        function obj = update_lanelet_relationships(obj,share_boundary_with)
            % Add more lanelets that have relationship with others
            % Lanelets that share the same lanelet boundary should have the
            % same relationship
            nLanelets = length(obj.lanelets);
            for iLan = 1:nLanelets-1
                for jLan = iLan+1:nLanelets
                    if ~isempty(obj.lanelet_relationships{iLan,jLan})
                        share_boundary_with_i = setdiff(share_boundary_with{iLan},iLan); % exclude self
                        share_boundary_with_j = setdiff(share_boundary_with{jLan},jLan); % exclude self
                        for iShare = share_boundary_with_i
                            for jShare = share_boundary_with_j
                                ijShare = [iShare,jShare];
                                if isempty(obj.lanelet_relationships{min(ijShare),max(ijShare)}) && iShare~=jShare && ~all(ismember(ijShare,obj.intersection_lanelets))
                                    obj.lanelet_relationships{min(ijShare),max(ijShare)} = obj.lanelet_relationships{iLan,jLan};
                                    obj.adjacency_lanelets(min(ijShare),max(ijShare)) = 1;
                                    obj.adjacency_lanelets(max(ijShare),min(ijShare)) = 1;
                                    obj.semi_adjacency_lanelets(min(ijShare),max(ijShare)) = 1;
                                    obj.semi_adjacency_lanelets(max(ijShare),min(ijShare)) = 1;
                                end
                            end
                        end
                    end
                end
            end
        end
    end


   methods (Static)
       function adjacent_lanelets_with_certain_relationship = find_adjacent_lanelets_with_certain_relationship(current_ID,lanelet_relationships,relationship)
        % returns adjacent lanelets with a certain relationship to the current lanelt
            % current lanelet could be found in the field `ID_1` and `ID_2`
            adjacent_idx_1 = find(~cellfun(@isempty,lanelet_relationships(current_ID,:)));
            adjacent_1 = lanelet_relationships(current_ID,adjacent_idx_1);

            adjacent_idx_2 = find(~cellfun(@isempty,lanelet_relationships(:,current_ID)));
            adjacent_2 = lanelet_relationships(adjacent_idx_2,current_ID);

            adjacent_idx = [adjacent_idx_1,adjacent_idx_2'];
            adjacent = [adjacent_1,adjacent_2'];

            
            find_target = cellfun(@(c) strcmp(c.type,relationship),adjacent);
            adjacent_lanelets_with_certain_relationship = adjacent_idx(find_target);
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
   end

end

