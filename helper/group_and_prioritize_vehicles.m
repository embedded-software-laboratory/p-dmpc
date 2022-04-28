function groups_info = group_and_prioritize_vehicles(scenario,iter)
%GROUP_AND_PRIORITIZE_VEHICLES Summary of this function goes here
%   Detailed explanation goes here
    % todo
%     road_info = get_road_information(scenario,iter);
    % vehicles in the same line form a group with size smaller than 3
    % vehicles get close to each other and has intersection point form a group
    
    groups = struct;
    groups(1).members = [1,4];
    groups(1).coupled_with_same_grp = {[],[1]};
    groups(1).coupled_with_other_grps = {[],[]};
    groups(2).members = [2,3];
    groups(2).coupled_with_same_grp = {[],[2]};
    groups(2).coupled_with_other_grps = {[4],[1]};
    groups_info = cell(scenario.nVeh,1);
    for i=1:scenario.nVeh
        groups_info{i} = groups;
    end
end