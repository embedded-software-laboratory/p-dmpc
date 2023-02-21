function [ref_path,loop] = generate_ref_path_loop(vehid, lanelets, lanelets_index)
% GENERATE_REF_PATH    returns a ref_path struct
% ref_path.lanelets_index: lanelet index of the reference path
% ref_path.path: reference path including x and y information
% ref_path.points_index: count the max index of reference points for each lanelets

% Totally 7 loops of paths are designed
ref_path_loops = {[4,6,8,60,58,56,54,80,82,84,86,34,32,30,28,2],...
                  [1,3,23,10,12,17,43,38,36,49,29,27],...
                  [64,62,75,55,53,79,81,101,88,90,95,69],...
                  [40,45,97,92,94,100,83,85,33,31,48,42],...
                  [5,7,59,57,74,68,66,71,19,14,16,22],...
                  [41,39,20,63,61,57,55,67,65,98,37,35,31,29],...
                  [3,5,9,11,72,91,93,81,83,87,89,46,13,15]};

ref_path = struct;

    if isempty(lanelets_index)
        switch vehid
            case 1
                loop = 1; starting_lanelet = 4;
            case 2
                loop = 1; starting_lanelet = 8;
            case 3
                loop = 1; starting_lanelet = 58;
            case 4 
                loop = 1; starting_lanelet = 54;
            case 5 
                loop = 1; starting_lanelet = 82;
            case 6
                loop = 1; starting_lanelet = 86;
            case 7
                loop = 1; starting_lanelet = 32;
            case 8
                loop = 1; starting_lanelet = 28;

            case 9
                loop = 2; starting_lanelet = 1;
            case 10
                loop = 2; starting_lanelet = 10;
            case 11
                loop = 2; starting_lanelet = 17;
            case 12
                loop = 2; starting_lanelet = 38;
            case 13
                loop = 2; starting_lanelet = 49;
                
            case 14
                loop = 3; starting_lanelet = 64;
            case 15
                loop = 3; starting_lanelet = 75;
            case 16
                loop = 3; starting_lanelet = 79;
            case 17
                loop = 3; starting_lanelet = 88;
            case 18
                loop = 3; starting_lanelet = 95;
                
            case 19
                loop = 4; starting_lanelet = 42;
            case 20
                loop = 4; starting_lanelet = 45;
            case 21
                loop = 4; starting_lanelet = 92;
            case 22
                loop = 4; starting_lanelet = 100;
            case 23
                loop = 4; starting_lanelet = 33;
                
            case 24
                loop = 5; starting_lanelet = 22;
            case 25
                loop = 5; starting_lanelet = 59;
            case 26
                loop = 5; starting_lanelet = 68;
            case 27
                loop = 5; starting_lanelet = 19;
            case 28
                loop = 5; starting_lanelet = 14;
                
            case 29
                loop = 6; starting_lanelet = 39;
            case 30
                loop = 6; starting_lanelet = 61;
            case 31
                loop = 6; starting_lanelet = 55;
            case 32
                loop = 6; starting_lanelet = 65;
            case 33
                loop = 6; starting_lanelet = 35;
            case 34
                loop = 6; starting_lanelet = 29;
                
            case 35
                loop = 7; starting_lanelet = 15;
            case 36
                loop = 7; starting_lanelet = 5;
            case 37
                loop = 7; starting_lanelet = 11;
            case 38
                loop = 7; starting_lanelet = 93;
            case 39
                loop = 7; starting_lanelet = 83;
            case 40
                loop = 7; starting_lanelet = 89;
            case 41
                loop = 5; starting_lanelet = 71;
        end 

        ref_path_loop = ref_path_loops{loop};
        find_start_idx = find(ref_path_loop==starting_lanelet);
        if find_start_idx==1
            ref_path.lanelets_index = ref_path_loop;
        else
            ref_path.lanelets_index = [ref_path_loop(find_start_idx:end),ref_path_loop(1:find_start_idx-1)];
        end
    else
        ref_path.lanelets_index = lanelets_index;
    end

    lanelets_target = lanelets(ref_path.lanelets_index);

    refPath_x = cellfun(@(c)c(:,LaneletInfo.cx), lanelets_target, 'UniformOutput', false);
    refPath_x = vertcat(refPath_x{:});
    refPath_y = cellfun(@(c)c(:,LaneletInfo.cy), lanelets_target, 'UniformOutput', false);
    refPath_y = vertcat(refPath_y{:});

    path = [refPath_x, refPath_y];

    % find identical successive points and delete them
    % Those are cases for the endpoint of one lanelet and the starting
    % point of its successor lanelet
    redundant_points_idx = ismembertol(sum(diff(path,1),2),0,1e-4);
    redundant_points_idx = [false;redundant_points_idx]; % add one element to keep size
    path_reduced = path(~redundant_points_idx,:);

    ref_path.path = path_reduced;

    % the max points index of each lanelet
    points_length = cellfun(@(c)length(c), lanelets_target);
    
    n_cumsum_lanelets_length = cumsum(points_length);

    % consider the removed redandant points
    n_cumsum_redandant = cumsum(redundant_points_idx(:)');

    ref_path.points_index = n_cumsum_lanelets_length - n_cumsum_redandant(n_cumsum_lanelets_length);

end
