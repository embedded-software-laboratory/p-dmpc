function refPath = generate_ref_path(vehid)
% GENERATE_REF_PATH    It returns reference path based on lanelets index
% for later trajectory planning



    switch vehid

        case 1
            lanelets_index = [5,9,13,97,93,89,113,109,162,63,59,51,47,43,1];

        case 2
            lanelets_index = [9,13,97,93,89,113,109,162,63,59,51,47,43,1,5];

        case 3
            lanelets_index = [13,97,93,89,113,109,162,63,59,51,47,43,1,5,9];
            
        case 4 
            lanelets_index = [97,93,89,113,109,162,63,59,51,47,43,1,5,9,13];

        case 5 
            lanelets_index = [93,89,113,109,162,63,59,51,47,71,67,36,105,101];

        case 6
            lanelets_index = [89,85,127,131,135,139,55,51,47,71,67,36,105,101,93];

        case 7
            lanelets_index = [85,127,131,135,139,55,51,47,71,67,36,105,101,93,89];

        case 8
            lanelets_index = [127,131,135,143,147,78,25,29,5,9,13,97,93,89,85];

        case 9
            lanelets_index = [131,135,143,147,78,25,29,5,9,13,97,93,89,85,127];

        case 10
            lanelets_index = [135,139,55,51,47,43,1,5,9,17,21,120,151,155,131];

        case 11
            lanelets_index = [139,55,51,47,43,1,5,9,17,21,120,151,155,131,135];

        case 12
            lanelets_index = [55,51,47,71,67,36,105,101,93,89,85,127,131,135,139];

        case 13
            lanelets_index = [51,47,71,67,36,105,101,93,89,85,127,131,135,139,55];

        case 14
            lanelets_index = [47,71,67,36,105,101,93,89,113,109,162,63,59,51];  

        case 15
            lanelets_index = [43,1,5,9,20,22,33,75,64,62,51,47];

        case 16
            lanelets_index = [42,84,63,59,51,47,71,67,82,166,151,155,131,135,146,148,159,117,106,104,93,89,113,109,124,40,25,29,5,9,20,23];

        case 17
            lanelets_index = [106,104,93,89,113,109,124,40,25,29,5,9,20,23,42,84,63,59,51,47,71,67,82,166,151,155,131,135,146,148,159,117];

        case 18
            lanelets_index = [124,40,25,29,5,9,20,23,42,84,63,59,51,47,71,67,82,166,151,155,131,135,146,148,159,117,106,104,93,89,113,109];

         case 19
            lanelets_index = [151,155,131,135,146,148,159,117,106,104,93,89,113,109,124,40,25,29,5,9,20,23,42,84,63,59,51,47,71,67,82,166];

         case 20
            lanelets_index = [159,117,106,104,93,89,113,109,124,40,25,29,5,9,20,23,42,84,63,59,51,47,71,67,82,166,151,155,131,135,146,148];

    end 

    
    refPath = [];
    [lanelets, ~] = commonroad_lanelets(); 
    
    for nlanelets = 1:length(lanelets_index)
        % choose the center line of the lanelet as reference path
        refPath_x = lanelets{lanelets_index(nlanelets)}(:,LaneletInfo.cx);
        refPath_y = lanelets{lanelets_index(nlanelets)}(:,LaneletInfo.cy);
        refPath_next = [refPath_x(1:end),refPath_y(1:end)];
        refPath = [refPath; refPath_next];

    end 

% figure
% plot(refPath(:,1),refPath(:,2))
% 
% xlim([0,4.5])
% ylim([0,4])

end











































