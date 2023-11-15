function [reference_path_struct] = generate_reference_path_loop(lanelets_index, lanelets)
    % generate_reference_path_loop returns a reference_path_struct
    %
    % Output:
    % reference_path_struct.lanelets_index: lanelet index of the reference path
    % reference_path_struct.path: reference path including x and y information
    % reference_path_struct.points_index: count the max index of reference points for each lanelets

    arguments
        lanelets_index (1, :) double
        lanelets (1, :) cell % with definitions for each lanelet
    end

    reference_path_struct = struct;

    reference_path_struct.lanelets_index = lanelets_index;

    lanelets_target = lanelets(reference_path_struct.lanelets_index);

    reference_path_x = cellfun(@(c)c(:, LaneletInfo.cx), lanelets_target, 'UniformOutput', false);
    reference_path_x = vertcat(reference_path_x{:});
    reference_path_y = cellfun(@(c)c(:, LaneletInfo.cy), lanelets_target, 'UniformOutput', false);
    reference_path_y = vertcat(reference_path_y{:});

    path = [reference_path_x, reference_path_y];

    % find identical successive points and delete them
    % Those are cases for the endpoint of one lanelet and the starting
    % point of its successor lanelet
    redundant_points_idx = ismembertol(sum(diff(path, 1), 2), 0, 1e-4);
    redundant_points_idx = [false; redundant_points_idx]; % add one element to keep size
    path_reduced = path(~redundant_points_idx, :);

    reference_path_struct.path = path_reduced;

    % the max points index of each lanelet
    points_length = cellfun(@(c)height(c), lanelets_target);

    n_cumsum_lanelets_length = cumsum(points_length);

    % consider the removed redandant points
    n_cumsum_redandant = cumsum(redundant_points_idx(:)');

    reference_path_struct.points_index = n_cumsum_lanelets_length - n_cumsum_redandant(n_cumsum_lanelets_length);

end
