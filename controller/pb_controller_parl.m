function [u, y_pred, info] = pb_controller_parl(scenario, iter)
% PB_CONTROLLER    Plan trajectory for one time step using a priority-based controller.
%     Controller simulates multiple distributed controllers.

    assert( ~isempty(scenario.adjacency) )

    % determine planning levels
    if scenario.assignPrios || isempty(scenario.directed_coupling)
        [isDAG, topo_groups] = topological_sorting_coloring(scenario.adjacency);
    else
        [isDAG, topo_groups] = kahn(scenario.directed_coupling);
    end
    assert( isDAG, 'Coupling matrix is not a DAG' );

    % extract the needed data from structure to single variable, since parfor is 'unfriendly' to
    % structure variable
    Hp = scenario.Hp;
    nVeh = scenario.nVeh; % total number of vehicles
    scenario_mpa = scenario.mpa;

    % get planning groups and their predecessors
    topo_groups = eye(nVeh);
    groups = PB_predecessor_groups(topo_groups); 

    y_pred = cell(nVeh,1);
    u = zeros(nVeh,1);

    % create temporary variables used in parfor
    y_pred_tmp = cell(length(groups),nVeh);  
    u_tmp = zeros(length(groups),nVeh);

    info = struct;
    info_vehicle_fullres_path = {};
    info_trim_indices = (-1)*ones(length(groups),nVeh);
    info_subcontroller_runtime = zeros(length(groups),nVeh);
    info_shapes = cell(length(groups),nVeh,Hp);
    info_next_node = [];
    info_n_expanded = zeros(length(groups),nVeh);

    sub_controller = @(scenario, iter)...
        graph_search(scenario, iter);
    
    parfor grp_idx = 1:length(groups)
        group = groups(grp_idx);

        % temporary veriables to store data
%         info_subcontroller_runtime_tmp = [];
%         info_n_expanded_tmp = 0;
        nVehGrp = length(group.members); % number of vehicles in the current group
        info_next_node_tmp = node(-1, zeros(nVehGrp,1), zeros(nVehGrp,1), zeros(nVehGrp,1), zeros(nVehGrp,1), -1, -1);
        info_shapes_tmp = cell(nVehGrp,Hp);
%         info_trim_indices_tmp = zeros(size(info_trim_indices));
        u_tmp = zeros(nVeh,1);

        for grp_member_idx = 1:nVeh % nested for-loop variable nust be a constant value 
            if grp_member_idx <= nVehGrp
                subcontroller_timer = tic;
                vehicle_idx = group.members(grp_member_idx);
                
                % Filter out vehicles with lower or same priority.
                priority_filter = false(1,nVeh);
    %             priority_filter(group.predecessors) = true; % keep all with higher priority
                priority_filter(vehicle_idx) = true; % keep self
                scenario_filtered = filter_scenario(scenario, priority_filter);
                iter_filtered = filter_iter(iter, priority_filter);
    
                self_index = sum(priority_filter(1:vehicle_idx));        
                v2o_filter = true(1,scenario_filtered.nVeh);
                v2o_filter(self_index) = false;
    
                % add predicted trajecotries as obstacle
    %             [scenario_v, iter_v] = vehicles_as_obstacles(scenario_filtered, iter_filtered, v2o_filter, info_shapes(group.predecessors,:));
                scenario_v = scenario_filtered; iter_v = iter_filtered; % without collision avoidance
    
                % execute sub controller for 1-veh scenario
                [u_v,y_pred_v,info_v] = sub_controller(scenario_v, iter_v);
                
                % prepare output data
                
                info_subcontroller_runtime(grp_idx,grp_member_idx) = toc(subcontroller_timer);  
                info_n_expanded(grp_idx,grp_member_idx) = info_v.tree.size();
                info_next_node_tmp = set_node(info_next_node_tmp,[grp_member_idx],info_v);
                info_shapes(grp_idx,grp_member_idx,:) = info_v.shapes(:);
    %             info_shapes_tmp{grp_member_idx,:} = info_v.shapes(:);
                info_vehicle_fullres_path(grp_idx,grp_member_idx) = path_between(info_v.tree_path(1),info_v.tree_path(2),info_v.tree,scenario_mpa);
                info_trim_indices(grp_idx,grp_member_idx) = info_v.trim_indices(1);
                y_pred_tmp{grp_idx,grp_member_idx} = y_pred_v{:};
                u_tmp(grp_idx,grp_member_idx) = u_v(1);
            end 
        end
%         info_subcontroller_runtime(grp_idx,:) = info_subcontroller_runtime_tmp;
%         info_shapes{grp_idx,:,:} = info_shapes_tmp;
        info_next_node(grp_idx,:,:) = info_next_node_tmp;
%         info_subcontroller_runtime(groups(grp_idx).members) = info_subcontroller_runtime_tmp;
    end
    
    % store data to 'info'
    for grp_idx = 1:length(groups)
        for grp_member_idx = 1:length(groups(grp_idx).members)
            vehicle_idx = groups(grp_idx).members(grp_member_idx);
            info.subcontroller_runtime(vehicle_idx) = info_subcontroller_runtime(grp_idx,grp_member_idx);
            info.n_expanded = sum(info_n_expanded,'all');
            info.next_node(vehicle_idx,:) = info_next_node(grp_idx,grp_member_idx,:);
            info.shapes(vehicle_idx,:) = info_shapes(grp_idx,grp_member_idx,:);
            info.vehicle_fullres_path(vehicle_idx) = info_vehicle_fullres_path(grp_idx,grp_member_idx);
            info.trim_indices(vehicle_idx) = info_trim_indices(grp_idx,grp_member_idx);
            y_pred{vehicle_idx,1} = y_pred_tmp{grp_idx,grp_member_idx};
            u(vehicle_idx) = u_tmp(grp_idx,grp_member_idx);
        end
    end
    
    
end
