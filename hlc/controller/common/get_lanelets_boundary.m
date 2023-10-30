function lanelet_boundary = get_lanelets_boundary(predicted_lanelets, lanelet_boundaries, lanelets_index, is_loop)
    % GET_LANELETS_BOUNDARY Return the lanelet boundaries
    % INPUT:
    %   predicted_lanelets: vector, IDs of predicted lanelets
    %
    %   lanelet_boundaries: boundaries of all lanelets
    %
    %   lanelets_index: IDs of the full lanelets
    %
    %   is_loop: if the reference path is a loop
    %
    % OUTPUT:
    %   lanelet_boundary: 1-by-3 cell array
    %                       first cell: left boundary
    %                       second cell: right boundary
    %                       third cell: MATLAB polyshape of the boundary

    lanelet_boundary = cell(1, 3);
    % extract the target lanelets
    predicted_lanelet_boundaries = lanelet_boundaries(predicted_lanelets);

    % determine the boundaries for each vehicle
    % Note that the last point of each lanelet boundary is deleted for a
    % smooth transition between two lanelet boundaries

    left_bound_cell = cellfun(@(c)c{1}(1:end - 1, :)', predicted_lanelet_boundaries, 'UniformOutput', false);
    left_bound = [left_bound_cell{:}];
    left_bound = [left_bound, predicted_lanelet_boundaries{end}{1}(end, :)']; % add the endpoint

    right_bound_cell = cellfun(@(c)c{2}(1:end - 1, :)', predicted_lanelet_boundaries, 'UniformOutput', false);
    right_bound = [right_bound_cell{:}];
    right_bound = [right_bound, predicted_lanelet_boundaries{end}{2}(end, :)']; % add the endpoint

    % it must be ensured that the whole vehicle body in inside the lanelet boundary
    % therefore the predecessor lanelet is considered
    % and several point from the predecessor are added

    % find the predecessor lanelet
    index_predicted_lanelets_in_reference_lanelets = find(predicted_lanelets(1) == lanelets_index);

    if index_predicted_lanelets_in_reference_lanelets ~= 1
        % if lanelet is not first lanelet of reference path
        predecessor_index = lanelets_index(index_predicted_lanelets_in_reference_lanelets - 1);
    elseif is_loop
        % if lanelet is first lanelet of reference path and the path is a loop
        predecessor_index = lanelets_index(end);
    else
        predecessor_index = [];
    end

    % add points to lanelet boundary
    if ~isempty(predecessor_index)
        % if a predecessor lanelet exist
        predecessor_lanelet_left = lanelet_boundaries{predecessor_index}{1};
        predecessor_lanelet_right = lanelet_boundaries{predecessor_index}{2};

        % Usually add four points of predecessor, but less if the
        % predecessor lanelet only consists of less elements
        num_added = min(4, min(height(predecessor_lanelet_right) - 1, height(predecessor_lanelet_left) - 1));

        % avoid add the last point since it is almost identical with the
        % first point of its successor
        left_bound = [predecessor_lanelet_left(end - num_added:end - 1, :)', left_bound];
        right_bound = [predecessor_lanelet_right(end - num_added:end - 1, :)', right_bound];
    end

    lanelet_boundary{1} = left_bound;
    lanelet_boundary{2} = right_bound;
    x = [left_bound(1, :), right_bound(1, end:-1:1)];
    y = [left_bound(2, :), right_bound(2, end:-1:1)];

    % if x- and y-coordinates are ensured to be in the right order, set
    % 'Simplify' to false to save computation time
    lanelet_boundary{3} = polyshape(x, y, 'Simplify', false);
    assert(lanelet_boundary{3}.NumRegions == 1)

end
