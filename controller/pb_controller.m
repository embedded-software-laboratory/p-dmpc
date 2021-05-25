function [u, y_pred, info] = pb_controller(scenario, iter)

    assert(~isempty(scenario.coupling_adjacency));

    [isDAG, topo_groups] = kahn(scenario.coupling_adjacency);

    assert( isDAG, 'Coupling matrix is not a DAG' );

    % add 'self-connections'.
    scenario.coupling_adjacency = scenario.coupling_adjacency + eye(scenario.nVeh) > 0;

    y_pred = cell(scenario.nVeh,1);
    u = cell(1);
    info = cell(scenario.nVeh,1);
    
    sub_controller = @(scenario, iter)...
        graph_search(scenario, iter);
    
    groups = PB_predecessor_groups(topo_groups);
    for grp_idx = 1:length(groups)
        group = groups(grp_idx);
        for grp_member_idx = 1:length(group.members) 
            subcontroller_timer = tic;
            vehicle_idx = group.members(grp_member_idx);

            
            % Filter out vehicles with lower or same priority.
            priority_filter = false(1,scenario.nVeh);
            priority_filter(group.predecessors) = true; % keep all with higher priority
            priority_filter(vehicle_idx) = true; % keep self
            scenario_filtered = filter_scenario(scenario, priority_filter);
            iter_filtered = filter_iter(iter, priority_filter);

            self_index = sum(priority_filter(1:vehicle_idx));        
            v2o_filter = true(1,scenario_filtered.nVeh);
            v2o_filter(self_index) = false;

            % add predicted trajecotries as obstacle
            [scenario_v, iter_v] = vehicles_as_obstacles(scenario_filtered, iter_filtered, v2o_filter, info(priority_filter,1));
    
            % execute sub controller for 1-veh scenario
            [u_v,y_pred_v,info_v] = sub_controller(scenario_v, iter_v);
            
            info_v.subcontroller_runtime = toc(subcontroller_timer);
            
            y_pred{vehicle_idx,1} = y_pred_v{:};
            u{vehicle_idx,1} = u_v;
            info{vehicle_idx,1} = info_v;
        end
    end
    
end
