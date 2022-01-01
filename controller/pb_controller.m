function [u, y_pred, info, priority_list,veh_at_intersection] = pb_controller(scenario, iter, last_veh_at_intersection)
% PB_CONTROLLER    Plan trajectory for one time step using a priority-based controller.
%     Controller simulates multiple distributed controllers.

    assert( ~isempty(scenario.adjacency) )

    MEflag = false; 

%     obj = topo_priority(scenario);
%     groups = obj.priority();   
%     obj = random_priority(scenario);
%     groups = obj.priority();  

    obj = right_of_way_update(scenario,iter);
    [veh_at_intersection, groups] = obj.priority(last_veh_at_intersection); 
    
    members_list = horzcat(groups.members);
    nVeh = length(members_list); 
    priority_list = zeros(1,nVeh);
    prio = 1;
    for iVeh = members_list
        
        priority_list(iVeh) = prio;
        prio = prio + 1;
    end


    y_pred = cell(scenario.nVeh,1);
    u = zeros(scenario.nVeh,1);
    info = struct;
    info.vehicle_fullres_path = cell(scenario.nVeh,1);
    info.trim_indices = (-1)*ones(scenario.nVeh,1);
    info.subcontroller_runtime = zeros(scenario.nVeh,1);
    info.shapes = cell(scenario.nVeh,scenario.Hp);
    info.next_node = node(-1, zeros(scenario.nVeh,1), zeros(scenario.nVeh,1), zeros(scenario.nVeh,1), zeros(scenario.nVeh,1), -1, -1);
    info.n_expanded = 0;
    
    sub_controller = @(scenario, iter)...
        graph_search(scenario, iter);    

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
            [scenario_v, iter_v] = vehicles_as_obstacles(scenario_filtered, iter_filtered, v2o_filter, info.shapes(group.predecessors,:));

            % execute sub controller for 1-veh scenario
            [u_v,y_pred_v,info_v, MEflag] = sub_controller(scenario_v, iter_v);

            if MEflag 
                disp('No more nodes to expand...')
                break
            end
            %
            info.tree{vehicle_idx} = info_v.tree;
            info.tree_path(vehicle_idx,:) = info_v.tree_path;

            % prepare output data
            info.subcontroller_runtime(vehicle_idx) = toc(subcontroller_timer);
            info.n_expanded = info.n_expanded + info_v.tree.size();
            info.next_node = set_node(info.next_node,[vehicle_idx],info_v);
            info.shapes(vehicle_idx,:) = info_v.shapes(:);
            info.vehicle_fullres_path(vehicle_idx) = path_between(info_v.tree_path(1),info_v.tree_path(2),info_v.tree,scenario.mpa);
            info.trim_indices(vehicle_idx) = info_v.trim_indices;
            y_pred{vehicle_idx,1} = y_pred_v{:};
            u(vehicle_idx) = u_v(1);
        end

        if MEflag 
            break
        end

    end

    Lastgroups = groups;

    
    
    
end
