function ref_path = generate_ref_path(vehid)
% GENERATE_REF_PATH    It returns a ref_path struct
% ref_path.lanelets_index: lanelet index of the reference path
% ref_path.path: reference path including x and y information
% ref_path.points_index: count the max index of reference points for each lanelets

ref_path = struct;

    switch vehid

        case 1
            ref_path.lanelets_index = [5,9,13,97,93,89,115,112,162,65,62,51,47,43,1];

        case 2
            ref_path.lanelets_index = [13,97,93,89,115,112,162,65,62,51,47,43,1,5,9];

        case 3
            ref_path.lanelets_index = [13,97,93,89,113,109,162,63,59,51,47,43,1,5,9];
            
        case 4 
            ref_path.lanelets_index = [97,93,89,113,109,162,63,59,51,47,43,1,5,9,13];

        case 5 
            ref_path.lanelets_index = [93,89,113,109,162,63,59,51,47,71,67,36,105,101];

        case 6
            ref_path.lanelets_index = [93,89,85,127,131,135,139,55,51,47,71,67,36,105,101];

        case 7
            ref_path.lanelets_index = [85,127,131,135,139,55,51,47,71,67,36,105,101,93,89];

        case 8
            ref_path.lanelets_index = [127,131,135,146,149,78,28,30,5,9,13,97,93,89,85];

        case 9
            ref_path.lanelets_index = [131,135,143,147,78,25,29,5,9,13,97,93,89,85,127];

        case 10
            ref_path.lanelets_index = [131,135,139,55,51,47,43,1,5,9,17,21,120,151,155];

        case 11
            ref_path.lanelets_index = [139,55,51,47,43,1,5,9,17,21,120,151,155,131,135];

        case 12
            ref_path.lanelets_index = [55,51,47,71,67,36,105,101,93,89,85,127,131,135,139];

        case 13
            ref_path.lanelets_index = [51,47,73,70,36,105,101,93,89,85,127,131,135,139,55];

        case 14
            ref_path.lanelets_index = [51,47,73,70,36,107,104,93,89,115,112,162,65,62];  

        case 15
            ref_path.lanelets_index = [43,1,5,9,20,22,33,75,64,62,51,47];

        case 16
            ref_path.lanelets_index = [84,63,59,51,47,71,67,82,166,151,155,131,135,146,148,159,117,106,104,93,89,113,109,124,40,25,29,5,9,20,23,42];

        case 17
            ref_path.lanelets_index = [117,106,104,93,89,113,109,124,40,25,29,5,9,20,23,42,84,63,59,51,47,71,67,82,166,151,155,131,135,146,148,159];

        case 18
            ref_path.lanelets_index = [124,40,25,29,5,9,20,23,42,84,63,59,51,47,71,67,82,166,151,155,131,135,146,148,159,117,106,104,93,89,113,109];

         case 19
            ref_path.lanelets_index = [151,155,131,135,146,148,159,117,106,104,93,89,113,109,124,40,25,29,5,9,20,23,42,84,63,59,51,47,71,67,82,166];

         case 20
            ref_path.lanelets_index = [126,107,104,93,89,113,109,124,40,25,29,5,9,20,23,42,84,63,59,51,47,71,67,82,166,151,155,131,135,146,149,168];

    end 

    % reference path
    path = [];
    [lanelets, ~,~] = commonroad_lanelets(); 
    
    for nlanelets = 1:length(ref_path.lanelets_index)
        % choose the center line of the lanelet as reference path
        refPath_x = lanelets{ ref_path.lanelets_index(nlanelets)}(:,LaneletInfo.cx);
        refPath_y = lanelets{ ref_path.lanelets_index(nlanelets)}(:,LaneletInfo.cy);
        refPath_next = [refPath_x(1:end),refPath_y(1:end)];
%         refPath_next = [refPath_x([1,6,end]),refPath_y([1,6,end])];
        path = [path; refPath_next];

    end 
    ref_path.path = path;
    
    % the max points index of each lanelet
    lanelet_point_max = 0;
    points_index = zeros(1,length( ref_path.lanelets_index));
    for nlanelets = 1:length( ref_path.lanelets_index)
        % count the number of points of each lanelets
        Npoints = length(lanelets{ ref_path.lanelets_index(nlanelets)}(:,LaneletInfo.cx));
        % max point index of each lanelet
        lanelet_point_max = lanelet_point_max + Npoints;
        points_index(nlanelets) = lanelet_point_max;

    end 
    ref_path.points_index = points_index;
    

% figure
% plot(path(:,1),path(:,2))
% 
% xlim([0,4.5])
% ylim([0,4])

end











































