function lanelet_boundary = get_lanelets_boundary(predicted_lanelets, lanelet_boundaries,lanelets_index, is_sim_lab)
% GET_LANELETS_BOUNDARY Return the lanelet boundaries based on the
% target_lanelet_boundaries: predicted lanelets boundaries
    
    % first cell: left boundary
    % second cell: right boundary
    % third cell: MATLAB polyshape of the boundary
    lanelet_boundary = cell(1,3);
    % extract the target lanelets
    predicted_lanelet_boundaries = lanelet_boundaries(predicted_lanelets);

    if is_sim_lab
        % determine the boundaries for each vehicle
        % Note that the last point of each lanelet boundary is deleted for a
        % smooth transition between two lanelet boundaries

%         % check if the boundaries of one lanelet to its successor lanelet is shrinked
%         threshold_shrink = 0.05;
%         left_bound = predicted_lanelet_boundaries{1}{1};
%         right_bound = predicted_lanelet_boundaries{1}{2};
%         for i = 1:length(predicted_lanelet_boundaries)-1
%             left_i = predicted_lanelet_boundaries{i}{1};
%             right_i = predicted_lanelet_boundaries{i}{2};
%             left_successor = predicted_lanelet_boundaries{i+1}{1};
%             right_successor = predicted_lanelet_boundaries{i+1}{2};
%             if norm(left_i(end,:)-right_i(end,:)) - norm(left_successor(1,:)-right_successor(1,:)) > threshold_shrink
%                 left_bound = check_lanelet_shrink(left_bound,left_i,left_successor,threshold_shrink);
%                 right_bound = check_lanelet_shrink(right_bound,right_i,right_successor,threshold_shrink);
%             end
%             % add successor lanelet boundary
%             % Since measure error of lanelet boundary exist, the first
%             % point of successor lanelet boundary is deleted in order to
%             % get a well-defined polyshape later
%             left_bound = [left_bound;left_successor(2:end,:)];
%             right_bound = [right_bound;right_successor(2:end,:)];
%         end

        left_bound_cell = cellfun(@(c)c{1}(1:end-1,:)', predicted_lanelet_boundaries, 'UniformOutput',false);
        left_bound = [left_bound_cell{:}];

        right_bound_cell = cellfun(@(c)c{2}(1:end-1,:)', predicted_lanelet_boundaries, 'UniformOutput',false);
        right_bound = [right_bound_cell{:}];

        % add several points of predecessor lanelet to ensure the whole
        % vehicle body is inside its lanelet boundary
        num_added = 3;
        find_first_predicted_lan = find(predicted_lanelets(1)==lanelets_index);
        if find_first_predicted_lan == 1
            % loop back
            predecessor_index = lanelets_index(end);
        else
            predecessor_index = lanelets_index(find_first_predicted_lan-1);
        end        
        predecessor_lanelet_left = lanelet_boundaries{predecessor_index}{1};
        predecessor_lanelet_right = lanelet_boundaries{predecessor_index}{2};
        % avoid add the last point since it is almost identical with the
        % first point of its successor
        left_bound = [predecessor_lanelet_left(end-num_added:end-1,:)',left_bound];
        right_bound = [predecessor_lanelet_right(end-num_added:end-1,:)',right_bound];

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
