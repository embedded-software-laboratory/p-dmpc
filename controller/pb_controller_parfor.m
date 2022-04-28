function [u, y_pred, info] = pb_controller_parl_backup(scenario, iter)
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
    topo_groups = [1 0 0 1;
                   0 1 1 0];
    groups = PB_predecessor_groups(topo_groups); 
    groups = struct;
    groups(1).members = [1,4];
    groups(1).coupledWithOtherGrp = {[],[]};
    groups(2).members = [2,3];
    groups(2).coupledWithOtherGrp = {[4],[1]};

    nGrp = length(groups); % number of groups
    y_pred = cell(nVeh,1);
    u = zeros(nVeh,1);

    % create temporary variables used in parfor
    y_pred_tmp = cell(nGrp,nVeh);
    u_tmp = zeros(nGrp,nVeh);

    info = struct;
    info_vehicle_fullres_path = {};
    info_trim_indices = (-1)*ones(nGrp,nVeh);
    info_subcontroller_runtime = zeros(nGrp,nVeh);
    info_shapes = cell(nGrp,nVeh,Hp);
    info_next_node = [];
    info_n_expanded = zeros(nGrp,nVeh);

    sub_controller = @(scenario, iter)...
        graph_search(scenario, iter);
    
    parfor grp_idx = 1:nGrp
        group = groups(grp_idx);

        % temporary veriables to store data
        nVehGrp = length(group.members); % number of vehicles in the current group
        info_next_node_tmp = node(-1, zeros(nVehGrp,1), zeros(nVehGrp,1), zeros(nVehGrp,1), zeros(nVehGrp,1), -1, -1);
        info_shapes_tmp = cell(nVehGrp,Hp);

        for grp_member_idx = 1:nVeh % nested for-loop variable nust be a constant value 
            if grp_member_idx <= nVehGrp
                subcontroller_timer = tic;
                vehicle_idx = group.members(grp_member_idx);
                
                % Filter out vehicles with lower or same priority.
                priority_filter = false(1,nVeh);
                priority_filter(group.members(1:grp_member_idx)) = true; % keep all with higher priority and self
                scenario_filtered = filter_scenario(scenario, priority_filter);
                iter_filtered = filter_iter(iter, priority_filter);
    
                self_index = sum(priority_filter(1:vehicle_idx)); % why 1:vehicle_idx?
                self_index = sum(priority_filter);  
                v2o_filter = true(1,scenario_filtered.nVeh);
                v2o_filter(self_index) = false;
    
                % add predicted trajecotries as obstacle
                [scenario_v, iter_v] = vehicles_as_obstacles(scenario_filtered, iter_filtered, v2o_filter, info_shapes_tmp(1:grp_member_idx-1,:));
%                 scenario_v = scenario_filtered; iter_v = iter_filtered; % without collision avoidance
%                 scenario_v.dynamic_obstacle_reachableSets = 
                % execute sub controller for 1-veh scenario
                [u_v,y_pred_v,info_v] = sub_controller(scenario_v, iter_v);
                
                % prepare output data
                
                info_subcontroller_runtime(grp_idx,grp_member_idx) = toc(subcontroller_timer);  
                info_n_expanded(grp_idx,grp_member_idx) = info_v.tree.size();
                info_next_node_tmp = set_node(info_next_node_tmp,[grp_member_idx],info_v);
                info_shapes_tmp(grp_member_idx,:) = info_v.shapes(:);
                info_shapes(grp_idx,grp_member_idx,:) = info_shapes_tmp(grp_member_idx,:);
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
    for grp_idx = 1:nGrp
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
