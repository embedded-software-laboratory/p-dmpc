function lanelet_boundary = get_lanelets_boundary(predicted_lanelets, lanelet_boundaries,lanelets_index, environment, is_loop)
% GET_LANELETS_BOUNDARY Return the lanelet boundaries
% INPUT:
%   predicted_lanelets: vector, IDs of predicted lanelets
% 
%   lanelet_boundaries: boundaries of all lanelets
% 
%   lanelets_index: IDs of the full lanelets
% 
%   environment: which environment is used (simulation, cpmlab, unified lab api)
%
%   is_loop: if the reference path is a loop
% 
% OUTPUT:
%   lanelet_boundary: 1-by-3 cell array
%                       first cell: left boundary
%                       second cell: right boundary
%                       third cell: MATLAB polyshape of the boundary

    lanelet_boundary = cell(1,3);
    % extract the target lanelets
    predicted_lanelet_boundaries = lanelet_boundaries(predicted_lanelets);

    if environment == Environment.Simulation
        % determine the boundaries for each vehicle
        % Note that the last point of each lanelet boundary is deleted for a
        % smooth transition between two lanelet boundaries

        left_bound_cell = cellfun(@(c)c{1}(1:end-1,:)', predicted_lanelet_boundaries, 'UniformOutput',false);
        left_bound = [left_bound_cell{:}];
        left_bound = [left_bound,predicted_lanelet_boundaries{end}{1}(end,:)']; % add the endpoint

        right_bound_cell = cellfun(@(c)c{2}(1:end-1,:)', predicted_lanelet_boundaries, 'UniformOutput',false);
        right_bound = [right_bound_cell{:}];
        right_bound = [right_bound,predicted_lanelet_boundaries{end}{2}(end,:)']; % add the endpoint


        % add several points of predecessor lanelet to ensure the whole
        % vehicle body is inside its lanelet boundary
        % see num_added below
        find_first_predicted_lan = find(predicted_lanelets(1)==lanelets_index);
        if find_first_predicted_lan == 1
            if is_loop
                % loop back
                predecessor_index = lanelets_index(end);
            else
                predecessor_index = [];
            end
        else
            predecessor_index = lanelets_index(find_first_predicted_lan-1);
        end        
        if ~isempty(predecessor_index)
            predecessor_lanelet_left = lanelet_boundaries{predecessor_index}{1};
            predecessor_lanelet_right = lanelet_boundaries{predecessor_index}{2};

            % Usually add for points of predecessor, but less if the
            % predecessor lanelet only consists of less elements
            num_added = min(min(4, height(predecessor_lanelet_right)-1), height(predecessor_lanelet_left)-1);

            % avoid add the last point since it is almost identical with the
            % first point of its successor
            left_bound = [predecessor_lanelet_left(end-num_added:end-1,:)',left_bound];
            right_bound = [predecessor_lanelet_right(end-num_added:end-1,:)',right_bound];
        end
    
        lanelet_boundary{1} = left_bound;
        lanelet_boundary{2} = right_bound;
        x = [left_bound(1,:),right_bound(1,end:-1:1)];
        y = [left_bound(2,:),right_bound(2,end:-1:1)];
        % if x- and y-coordinates are ensured to be in the right order, set
        % 'Simplify' to false to save computation time
        lanelet_boundary{3} = polyshape(x,y,'Simplify',false);
        assert(lanelet_boundary{3}.NumRegions==1)
%         plot_obstacles(lanelet_boundary{3})
    else
        % determine the boundaries for each vehicle
        left_bound = cellfun(@(c)c{1}', predicted_lanelet_boundaries, 'UniformOutput',false);
        left_bound = [left_bound{:}];

        right_bound = cellfun(@(c)c{2}', predicted_lanelet_boundaries, 'UniformOutput',false);
        right_bound = [right_bound{:}];
        
        lanelet_boundary{1} = left_bound;
        lanelet_boundary{2} = right_bound;

        x = [left_bound(1,:),right_bound(1,end:-1:1)];
        y = [left_bound(2,:),right_bound(2,end:-1:1)];
        % if x- and y-coordinates are ensured to be in the right order, set
        % 'Simplify' to false to save computation time
        lanelet_boundary{3} = polyshape(x,y,'Simplify',false);
        assert(lanelet_boundary{3}.NumRegions==1)

    end
end
