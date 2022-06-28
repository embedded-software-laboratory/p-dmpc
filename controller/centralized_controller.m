function [u, y_pred, info] = centralized_controller(scenario, iter)
% CENTRALIZED_CONTROLLER    Plan trajectory for one time step using a centralized controller.

    info = struct;
    info.vehicle_fullres_path = cell(scenario.nVeh,1);
    info.trim_indices = (-1)*ones(scenario.nVeh,1);
    info.subcontroller_runtime = zeros(scenario.nVeh,1);
    info.shapes = cell(scenario.nVeh,scenario.Hp);
    info.next_node = node(-1, zeros(scenario.nVeh,1), zeros(scenario.nVeh,1), zeros(scenario.nVeh,1), zeros(scenario.nVeh,1), -1, -1);
    info.n_expanded = 0;
    
    sub_controller = @scenario.sub_controller;
    % falsifies controller_runtime slightly
    subcontroller_timer = tic;
    [u, y_pred, info_v] = sub_controller(scenario, iter);
    
    % Check if feasible
    info.is_feasible = info_v.is_feasible;
    if (~info.is_feasible), return, end

    % prepare output data
    info.tree = info_v.tree; % only for node explorationslee
    info.subcontroller_runtime = toc(subcontroller_timer);
    info.n_expanded = info.n_expanded + info_v.tree.size();
    info.next_node = set_node(info.next_node,1:scenario.nVeh,info_v);
    info.shapes = info_v.shapes;
    info.vehicle_fullres_path = path_between(info_v.tree_path(1),info_v.tree_path(2),info_v.tree,scenario.mpa);
    info.trim_indices = info_v.trim_indices;
end
