function lanelet_boundary = get_lanelets_boundary(predicted_lanelets, lanelet_boundaries, lanelets_index)
% GET_LANELETS_BOUNDARY Return the lanelet boundaries based on the
% predicted lanelets
    
    % first cell: left boundary
    % second cell: right boundary
    % third cell: MATLAB polyshape of the boundary
    lanelet_boundary = cell(1,3);

    % extract the target lanelets
    target_lanelet_boundaries = lanelet_boundaries(predicted_lanelets);

    % find the predecessor lanelet
    find_first_lanelet_idx = find(predicted_lanelets(1)==lanelets_index);

    if find_first_lanelet_idx==1
        % loop back to the last lanelet
        predecessor_fitst_lanelet = lanelets_index(length(lanelets_index));
    else
        predecessor_fitst_lanelet = lanelets_index(find_first_lanelet_idx-1);
    end
    
    predecessor_fitst_lanelet_boundary = lanelet_boundaries{predecessor_fitst_lanelet};

    % determine the boundaries for each vehicle
    left_bound = cellfun(@(c)c{1}(1:end-1,:)', target_lanelet_boundaries, 'UniformOutput',false);
    % add the end part of the predecessor lanelet boundary to avoid vehicle
    % being too close to the starting part of the labelet boundary 
    end_part_predecessor_left = predecessor_fitst_lanelet_boundary{1}(end-3:end-1,:)';
    left_bound = [end_part_predecessor_left, left_bound{:}];

    right_bound = cellfun(@(c)c{2}(1:end-1,:)', target_lanelet_boundaries, 'UniformOutput',false);
    end_part_predecessor_right = predecessor_fitst_lanelet_boundary{2}(end-3:end-1,:)';
    right_bound = [end_part_predecessor_right, right_bound{:}];
    
    lanelet_boundary{1} = left_bound;
    lanelet_boundary{2} = right_bound;

    x = [left_bound(1,:),right_bound(1,end:-1:1)];
    y = [left_bound(2,:),right_bound(2,end:-1:1)];
    % if x- and y-coordinates are ensured to be in the right order, set
    % 'Simplify' to false to save computation time
    lanelet_boundary{3} = polyshape(x,y,'Simplify',false);
    assert(lanelet_boundary{3}.NumRegions==1)
end
