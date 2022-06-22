function lanelet_boundary = get_lanelets_boundary(predicted_lanelets, lanelet_boundaries, is_sim_lab)
% GET_LANELETS_BOUNDARY Return the lanelet boundaries based on the
% target_lanelet_boundaries: predicted lanelets boundaries
    
    % first cell: left boundary
    % second cell: right boundary
    % third cell: MATLAB polyshape of the boundary
    lanelet_boundary = cell(1,3);
    % extract the target lanelets
    lanelet_boundaries = lanelet_boundaries(predicted_lanelets);

    if is_sim_lab
        % determine the boundaries for each vehicle
        % Note that the first point of each lanelet boundary is deleted for a
        % smooth transition between two lanelet boundaries
        left_bound = cellfun(@(c)c{1}(2:end,:)', lanelet_boundaries, 'UniformOutput',false);
        left_bound = [left_bound{:}];

        right_bound = cellfun(@(c)c{2}(2:end,:)', lanelet_boundaries, 'UniformOutput',false);
        right_bound = [right_bound{:}];
        
        lanelet_boundary{1} = left_bound;
        lanelet_boundary{2} = right_bound;

        x = [left_bound(1,:),right_bound(1,end:-1:1)];
        y = [left_bound(2,:),right_bound(2,end:-1:1)];
        % if x- and y-coordinates are ensured to be in the right order, set
        % 'Simplify' to false to save computation time
        lanelet_boundary{3} = polyshape(x,y,'Simplify',false);
        assert(lanelet_boundary{3}.NumRegions==1)
    else
        % determine the boundaries for each vehicle
        left_bound = cellfun(@(c)c{1}', lanelet_boundaries, 'UniformOutput',false);
        left_bound = [left_bound{:}];

        right_bound = cellfun(@(c)c{2}', lanelet_boundaries, 'UniformOutput',false);
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
