function [u, y_pred, info] = pb_controller_parl_ROS(scenario, iter)
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
    
    % get planning groups and their predecessors
    groups = PB_predecessor_groups(topo_groups);
    groups = struct;
    groups(1).members = [1,4];
    groups(1).coupled_with_same_grp = {[],[1]};
    groups(1).coupled_with_other_grps = {[],[]};
    groups(2).members = [2,3];
    groups(2).coupled_with_same_grp = {[],[2]};
    groups(2).coupled_with_other_grps = {[4],[1]};

    y_pred = cell(scenario.nVeh,1);
    u = zeros(scenario.nVeh,1);
    info = struct;
    info.vehicle_fullres_path = cell(scenario.nVeh,1);
    info.trim_indices = (-1)*ones(scenario.nVeh,1);
    info.subcontroller_runtime = zeros(scenario.nVeh,1);
    info.shapes = cell(scenario.nVeh,scenario.Hp);
    info.next_node = node(-1, zeros(scenario.nVeh,1), zeros(scenario.nVeh,1), zeros(scenario.nVeh,1), zeros(scenario.nVeh,1), -1, -1);
    info.n_expanded = 0;
    info.trims_Hp = zeros(scenario.nVeh,scenario.Hp+1); % trims planned in the next Hp time steps (including starting trim)
    
    sub_controller = @(scenario, iter)...
        graph_search(scenario, iter);
    
    for grp_idx = 1:length(groups)
        group = groups(grp_idx);
        for grp_member_idx = 1:length(group.members) 
            subcontroller_timer = tic;
            vehicle_idx = group.members(grp_member_idx);
%             disp(['Vechile: ',num2str(vehicle_idx)]);

            
            % Filter out vehicles with lower or same priority.
            priority_filter_same_grp = false(1,scenario.nVeh);
            priority_filter_same_grp(group.coupled_with_same_grp{grp_member_idx}) = true; % keep all with higher priority
            priority_filter_same_grp(vehicle_idx) = true; % keep self
            priority_filter_other_grps = false(1,scenario.nVeh);
            priority_filter_other_grps(group.coupled_with_other_grps{grp_member_idx}) = true; % keep all coupled vehicle with higher priority

            scenario_filtered_same_grp = filter_scenario(scenario, priority_filter_same_grp);
            iter_filtered_same_grp = filter_iter(iter, priority_filter_same_grp);
            iter_filtered_other_grps = filter_iter(iter, priority_filter_other_grps);

            self_index = sum(priority_filter_same_grp(1:vehicle_idx));        
            v2o_filter = true(1,scenario_filtered_same_grp.nVeh);
            v2o_filter(self_index) = false;

            % add predicted trajecotries of the coupled vehicles with higher priorities in the same group as obstacles
            [scenario_v, iter_v] = vehicles_as_obstacles(scenario_filtered_same_grp, iter_filtered_same_grp, v2o_filter, info.shapes(group.coupled_with_same_grp{grp_member_idx},:));

            % initial trims of coupled vehicles with higher priorities in the other groups
            initial_trims_other_grps = iter_filtered_other_grps.trim_indices;
            reachable_sets_other_grps = scenario.mpa.reachable_sets_conv(initial_trims_other_grps,:);
            % add reachable sets of the coupled vehicles with higher priorities in other groups as obsticles
            scenario_v.dynamic_obstacle_reachableSets = ...
                vehicles_as_obstacles_other_grps(scenario_v.dynamic_obstacle_reachableSets, iter_filtered_other_grps, reachable_sets_other_grps);

            % execute sub controller for 1-veh scenario
            [u_v,y_pred_v,info_v] = sub_controller(scenario_v, iter_v);
            
            % prepare output data
            info.subcontroller_runtime(vehicle_idx) = toc(subcontroller_timer);
            info.n_expanded = info.n_expanded + info_v.tree.size();
            info.next_node = set_node(info.next_node,[vehicle_idx],info_v);
            info.shapes(vehicle_idx,:) = info_v.shapes(:);
            info.vehicle_fullres_path(vehicle_idx) = path_between(info_v.tree_path(1),info_v.tree_path(2),info_v.tree,scenario.mpa);
            info.trim_indices(vehicle_idx) = info_v.trim_indices(1);
            info.trims_Hp(vehicle_idx,:) = info_v.trims_Hp;
            info.tree_path(vehicle_idx,:) = info_v.tree_path;
            info.trees{vehicle_idx} = info_v.tree; % store tree information
            y_pred{vehicle_idx,1} = y_pred_v{:};
            info.y_predicted{vehicle_idx,1} = y_pred_v{:}; % store the information of the predicted output
            u(vehicle_idx) = u_v(1);
            info.n_exhausted(vehicle_idx) = info_v.n_exhausted;
            
        end
    end
    
end
