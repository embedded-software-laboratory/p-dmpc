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
        % check if the boundaries of one lanelet to its successor lanelet is shrinked
        threshold_shrink = 0.05;
        left_bound = lanelet_boundaries{1}{1};
        right_bound = lanelet_boundaries{1}{2};
        for i = 1:length(lanelet_boundaries)-1
            left_i = lanelet_boundaries{i}{1};
            right_i = lanelet_boundaries{i}{2};
            left_successor = lanelet_boundaries{i+1}{1};
            right_successor = lanelet_boundaries{i+1}{2};
            if norm(left_i(end,:)-right_i(end,:)) - norm(left_successor(1,:)-right_successor(1,:)) > threshold_shrink
                left_bound = check_lanelet_shrink(left_bound,left_i,left_successor,threshold_shrink);
                right_bound = check_lanelet_shrink(right_bound,right_i,right_successor,threshold_shrink);
            end
            % add successor lanelet boundary
            % Since measure error of lanelet boundary exist, the first
            % point of successor lanelet boundary is deleted in order to
            % get a well-defined polyshape later
            left_bound = [left_bound;left_successor(2:end,:)];
            right_bound = [right_bound;right_successor(2:end,:)];
        end

%         left_bound = cellfun(@(c)c{1}(2:end,:)', lanelet_boundaries, 'UniformOutput',false);
%         left_bound = [left_bound{:}];
% 
%         right_bound = cellfun(@(c)c{2}(2:end,:)', lanelet_boundaries, 'UniformOutput',false);
%         right_bound = [right_bound{:}];
        left_bound = left_bound';
        right_bound = right_bound';
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


%% local function
function bound_new = check_lanelet_shrink(bound,boundary,boundary_successor,threshold_shrink)
    if abs(norm(boundary(end,:)-boundary_successor(1,:))) > threshold_shrink
        % left boundary is shrunken: replace the second half
        % points of left boundary by linear interpolation to make trasition smoother
        n_replace_points = round(length(boundary)/2);
        linear_inter = bound(end-n_replace_points,:) + (boundary_successor(1,:) - bound(end-n_replace_points,:))/n_replace_points.* (1:n_replace_points)';
        bound_new = [bound(1:end-n_replace_points,:);linear_inter];
    else
        bound_new = bound;
    end
    assert(length(bound_new)==length(bound))
end
