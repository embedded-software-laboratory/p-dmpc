function ref_path = generate_ref_path(vehid, lanelets)
    % GENERATE_REF_PATH    returns a ref_path struct
    % ref_path.lanelets_index: lanelet index of the reference path
    % ref_path.path: reference path including x and y information
    % ref_path.points_index: count the max index of reference points for each lanelets

    ref_path = struct;

    switch vehid

        case 1
            ref_path.lanelets_index = [3, 5, 7, 59, 57, 55, 67, 65, 98, 37, 35, 31, 29, 27, 1];

        case 2
            ref_path.lanelets_index = [5, 7, 59, 57, 55, 67, 65, 98, 37, 35, 31, 29, 27, 1, 3];

        case 3
            ref_path.lanelets_index = [7, 59, 57, 55, 67, 65, 98, 37, 35, 31, 29, 27, 1, 3, 5];

        case 4
            ref_path.lanelets_index = [59, 57, 55, 67, 65, 98, 37, 35, 31, 29, 27, 1, 3, 5, 7];

        case 5
            ref_path.lanelets_index = [57, 55, 67, 65, 98, 37, 35, 31, 29, 27, 1, 3, 5, 7, 59];

        case 6
            ref_path.lanelets_index = [29, 41, 39, 20, 63, 61, 57, 55, 53, 79, 81, 83, 85, 33, 31];

        case 7
            ref_path.lanelets_index = [53, 79, 81, 83, 85, 33, 31, 29, 41, 39, 20, 63, 61, 57, 55];

        case 8
            ref_path.lanelets_index = [79, 81, 83, 85, 33, 31, 29, 41, 39, 20, 63, 61, 57, 55, 53];

        case 9
            ref_path.lanelets_index = [81, 83, 87, 89, 46, 13, 15, 3, 5, 7, 59, 57, 55, 53, 79];

        case 10
            ref_path.lanelets_index = [11, 72, 91, 93, 81, 83, 85, 33, 31, 29, 27, 1, 3, 5, 9];

        case 11
            ref_path.lanelets_index = [85, 33, 31, 29, 27, 1, 3, 5, 9, 11, 72, 91, 93, 81, 83];

        case 12
            ref_path.lanelets_index = [33, 31, 29, 27, 1, 3, 5, 9, 11, 72, 91, 93, 81, 83, 85];

        case 13
            ref_path.lanelets_index = [31, 29, 41, 39, 20, 63, 61, 57, 55, 53, 79, 81, 83, 85, 33];

        case 14
            ref_path.lanelets_index = [20, 63, 61, 57, 55, 53, 79, 81, 83, 85, 33, 31, 29, 41, 39];

        case 15
            ref_path.lanelets_index = [27, 1, 3, 5, 9, 11, 26, 52, 37, 35, 31, 29];

        case 16
            ref_path.lanelets_index = [26, 52, 37, 35, 31, 29, 27, 1, 3, 5, 9, 11];

        case 17
            ref_path.lanelets_index = [89, 104, 78, 63, 61, 57, 55, 53, 79, 81, 83, 87];

        case 18
            ref_path.lanelets_index = [76, 24, 13, 15, 3, 5, 7, 59, 57, 55, 67, 65];

        case 19
            ref_path.lanelets_index = [91, 93, 81, 83, 85, 33, 31, 29, 41, 39, 50, 102];

        case 20
            ref_path.lanelets_index = [104, 78, 63, 61, 57, 55, 53, 79, 81, 83, 87, 89];

        case 21
            ref_path.lanelets_index = [93, 81, 83, 85, 33, 31, 29, 41, 39, 50, 102, 91];

        case 22
            ref_path.lanelets_index = [65, 77, 63, 61, 57, 55, 67];

        case 23
            ref_path.lanelets_index = [13, 15, 3, 5, 7, 59, 57, 55, 67, 65, 76, 24];

        case 24
            ref_path.lanelets_index = [55, 53, 79, 81, 83, 85, 33, 31, 29, 41, 39, 20, 63, 61, 57];

        case 25
            ref_path.lanelets_index = [1, 3, 5, 7, 59, 57, 55, 53, 79, 81, 83, 85, 33, 31, 29, 27];

        case 26
            ref_path.lanelets_index = [19, 14, 16, 22, 5, 7, 59, 57, 74, 68, 66, 71];

        case 27
            ref_path.lanelets_index = [83, 85, 33, 31, 29, 27, 1, 3, 5, 9, 11, 72, 91, 93, 81];

        case 28
            ref_path.lanelets_index = [52, 37, 35, 31, 29, 27, 1, 3, 5, 9, 11, 26];

        case 29
            ref_path.lanelets_index = [37, 35, 31, 29, 27, 1, 3, 5, 9, 11, 26, 52];

        case 30
            ref_path.lanelets_index = [35, 31, 29, 27, 1, 3, 5, 9, 11, 26, 52, 37];

        case 31
            ref_path.lanelets_index = [39, 50, 102, 91, 93, 81, 83, 85, 33, 31, 29, 41];

        case 32
            ref_path.lanelets_index = [63, 61, 57, 55, 53, 79, 81, 83, 85, 33, 31, 29, 41, 39, 20];

        case 33
            ref_path.lanelets_index = [61, 57, 55, 53, 79, 81, 83, 85, 33, 31, 29, 41, 39, 20, 63];

        case 34
            ref_path.lanelets_index = [16, 22, 5, 7, 59, 57, 74, 68, 66, 71, 19, 14];

        case 35
            ref_path.lanelets_index = [66, 71, 19, 14, 16, 22, 5, 7, 59, 57, 74, 68];
    end

    lanelets_target = lanelets(ref_path.lanelets_index);

    refPath_x = cellfun(@(c)c(:, LaneletInfo.cx), lanelets_target, 'UniformOutput', false);
    refPath_x = vertcat(refPath_x{:});
    refPath_y = cellfun(@(c)c(:, LaneletInfo.cy), lanelets_target, 'UniformOutput', false);
    refPath_y = vertcat(refPath_y{:});

    path = [refPath_x, refPath_y];

    % find identical successive points and delete them
    % Those are cases for the endpoint of one lanelet and the starting
    % point of its successor lanelet
    redundant_points_idx = ismembertol(sum(diff(path, 1), 2), 0, 1e-4);
    redundant_points_idx = [false; redundant_points_idx]; % add one element to keep size
    path_reduced = path(~redundant_points_idx, :);

    ref_path.path = path_reduced;

    % the max points index of each lanelet
    points_length = cellfun(@(c)length(c), lanelets_target);

    n_cumsum_lanelets_length = cumsum(points_length);

    % consider the removed redandant points
    n_cumsum_redandant = cumsum(redundant_points_idx(:)');

    ref_path.points_index = n_cumsum_lanelets_length - n_cumsum_redandant(n_cumsum_lanelets_length);

end
