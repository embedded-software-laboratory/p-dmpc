function [u, y_pred, info] = pb_controller(scenario, iter)
% PB_CONTROLLER    Plan trajectory for one time step using a priority-based controller.
%     Controller simulates multiple distributed controllers.


switch scenario.priority_option
    case 'topo_priority'
        obj = topo_priority(scenario);
        groups = obj.priority(); 
        right_of_way = false;
        veh_at_intersection = [];
        edge_to_break = [];
    case 'right_of_way_priority'
        obj = right_of_way_priority(scenario,iter);
        right_of_way = true;
        [veh_at_intersection, groups, edge_to_break] = obj.priority();  
    case 'constant_priority'
        obj = constant_priority(scenario);
        groups = obj.priority(); 
        right_of_way = false;
        veh_at_intersection = [];
        edge_to_break = [];
    case 'random_priority' 
        obj = random_priority(scenario);
        groups = obj.priority(); 
        right_of_way = false;
        veh_at_intersection = [];
        edge_to_break = [];
    case 'FCA_priority'
        obj = FCA_priority(scenario,iter);
        [veh_at_intersection, groups] = obj.priority();
        right_of_way = false;
        edge_to_break = [];   
end

    
    % construct the priority list
    computation_levels = length(groups);
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
    info.priority_list = priority_list;
    info.veh_at_intersection = veh_at_intersection;
    info.computation_levels = computation_levels;
    info.edge_to_break = edge_to_break;
    
    % graph-search to select the optimal motion primitive
    sub_controller = @(scenario, iter)...
        graph_search(scenario, iter); 

    for grp_idx = 1:length(groups)
        group = groups(grp_idx);
        for grp_member_idx = 1:length(group.members) 
            subcontroller_timer = tic;
            vehicle_idx = group.members(grp_member_idx);
            % Filter out vehicles that are not adjacent
            veh_adjacent = find(scenario.adjacency(vehicle_idx,:,end));
            predecessors = intersect(group.predecessors,veh_adjacent);
%             predecessors = group.predecessors;

            % Filter out vehicles with lower or same priority.
            priority_filter = false(1,scenario.nVeh);
            priority_filter(predecessors) = true; % keep all with higher priority
            priority_filter(vehicle_idx) = true; % keep self
            scenario_filtered = filter_scenario(scenario, priority_filter);
            iter_filtered = filter_iter(iter, priority_filter);

            self_index = sum(priority_filter(1:vehicle_idx));        
            v2o_filter = true(1,scenario_filtered.nVeh);
            v2o_filter(self_index) = false;

            % add predicted trajecotries as obstacle
            [scenario_v, iter_v] = vehicles_as_obstacles(scenario_filtered, iter_filtered, v2o_filter, info.shapes(predecessors,:));

            
            % add adjacent vehicles with lower priorities as static obstacles
            if right_of_way
                adjacent_vehicle_lower_priority = setdiff(veh_adjacent,predecessors);

                for i = 1:length(adjacent_vehicle_lower_priority)
                    veh_index = adjacent_vehicle_lower_priority(i);
                    x0 = iter.x0(veh_index,1);
                    y0 = iter.x0(veh_index,2);
                    yaw0 = iter.x0(veh_index,3);
                    veh = Vehicle();
                    x_locals = [-1, -1,  1,  1] * (veh.Length/2+scenario.offset);
                    y_locals = [-1,  1,  1, -1] * (veh.Width/2+scenario.offset);
                    [x_globals,y_globals] = translate_global(yaw0, x0, y0, x_locals, y_locals);
                    scenario_v.obstacles{end+1} = [x_globals;y_globals];
                end
           end
            % execute sub controller for 1-veh scenario
            [u_v,y_pred_v,info_v] = sub_controller(scenario_v, iter_v);
            
            % prepare output data
            info.tree{vehicle_idx} = info_v.tree;
            info.tree_path(vehicle_idx,:) = info_v.tree_path;
            info.subcontroller_runtime(vehicle_idx) = toc(subcontroller_timer);
            info.n_expanded = info.n_expanded + info_v.tree.size();
            info.next_node = set_node(info.next_node,[vehicle_idx],info_v);
            info.shapes(vehicle_idx,:) = info_v.shapes(:);
            info.vehicle_fullres_path(vehicle_idx) = path_between(info_v.tree_path(1),info_v.tree_path(2),info_v.tree,scenario.mpa);
            info.trim_indices(vehicle_idx) = info_v.trim_indices;
            info.trims_Hp(vehicle_idx,:) = info_v.trims_Hp; % store the planned trims in the future Hp time steps
            info.trees{vehicle_idx} = info_v.tree; % store tree information
            info.y_predicted{vehicle_idx,1} = y_pred_v{:}; % store the information of the predicted output
            info.n_exhausted(vehicle_idx) = info_v.n_exhausted; % store the number of graph search exhaustion times
            y_pred{vehicle_idx,1} = y_pred_v{:};
            u(vehicle_idx) = u_v(1);
        end

    end
   
end
