function [info, scenario] = pb_controller(scenario, iter)
% PB_CONTROLLER    Plan trajectory for one time step using a priority-based controller.
%     Controller simulates multiple distributed controllers.

runtime_others_tic = tic;
right_of_way = false;
switch scenario.priority_option
    case 'topo_priority' 
        [groups, directed_adjacency, priority_list] = topo_priority().priority(scenario); 
        veh_at_intersection = [];
%         edge_to_break = [];
    case 'right_of_way_priority' 
        right_of_way = true;
        [veh_at_intersection, groups, ~, directed_adjacency,priority_list] = right_of_way_priority().priority(scenario,iter);  
    case 'constant_priority' 
        [groups, directed_adjacency, priority_list] = constant_priority().priority(scenario); 
        veh_at_intersection = [];
%         edge_to_break = [];
    case 'random_priority'  
        [groups, directed_adjacency, priority_list] = random_priority().priority(scenario); 
        veh_at_intersection = [];
%         edge_to_break = [];
    case 'FCA_priority' 
        [veh_at_intersection, groups, directed_adjacency, priority_list] = FCA_priority().priority(scenario,iter);
%         edge_to_break = [];   
    case 'mixed_traffic_priority'
        obj = mixed_traffic_priority(scenario);
        [groups, directed_adjacency] = obj.priority(); 
        right_of_way = false;
        veh_at_intersection = [];
        %edge_to_break = [];
end

    % visualize the coupling between vehicles
%     plot_coupling_lines(directed_adjacency, iter)

    % construct the priority list
    computation_levels = length(groups); 
    
    nVeh = scenario.nVeh; 
    Hp = scenario.Hp;

    % update properties of scenario
    scenario.directed_coupling = directed_adjacency;
    scenario.priority_list = priority_list;
    scenario.last_vehs_at_intersection = veh_at_intersection;
    
    % initialize variable to store control results
    info = ControllResultsInfo(nVeh, Hp, [scenario.vehicles.ID]);
    
    % graph-search to select the optimal motion primitive
    sub_controller = @(scenario, iter)...
        graph_search(scenario, iter); 

    directed_graph = digraph(directed_adjacency);
    [belonging_vector_total,~] = conncomp(directed_graph,'Type','weak'); % graph decomposition
    runtime_others = toc(runtime_others_tic); % subcontroller runtime except for runtime of graph search 
 

    for grp_idx = 1:length(groups)
        group = groups(grp_idx);
        for grp_member_idx = 1:length(group.members) 
            subcontroller_timer = tic;
            
            vehicle_idx = group.members(grp_member_idx);
            if ismember(vehicle_idx, info.vehs_fallback)
                % if the selected vehicle should take fallback
                info.subcontroller_runtime(vehicle_idx) = 0;
                continue
            end
            
            % Filter out vehicles that are not adjacent
            veh_adjacent = find(scenario.adjacency(vehicle_idx,:,end));
            predecessors = intersect(group.predecessors,veh_adjacent);

            % Filter out vehicles with lower or same priority.
            priority_filter = false(1,scenario.nVeh);
            priority_filter(predecessors) = true; % keep all with higher priority
            priority_filter(vehicle_idx) = true; % keep self
            scenario_filtered = filter_scenario(scenario, priority_filter);
            iter_filtered = filter_iter(iter, priority_filter);

            self_index = sum(priority_filter(1:vehicle_idx));        
            v2o_filter = true(1,scenario_filtered.nVeh);
            v2o_filter(self_index) = false;

            % add predicted trajecotries of vehicles with higher priority as dynamic obstacle
            [scenario_v, iter_v] = vehicles_as_dynamic_obstacles(scenario_filtered, iter_filtered, v2o_filter, info.shapes(predecessors,:));
            
            % add adjacent vehicles with lower priorities as static obstacles
            if right_of_way
                adjacent_vehicle_lower_priority = setdiff(veh_adjacent,predecessors);
                
                % only two strategies are supported if parallel computation is not used
                assert(strcmp(scenario_v.strategy_consider_veh_without_ROW,'2')==true || strcmp(scenario_v.strategy_consider_veh_without_ROW,'3')==true)
                scenario_v = consider_vehs_without_ROW(scenario_v, iter, adjacent_vehicle_lower_priority);
            end

            if scenario.k >= 128
                if vehicle_idx == 4
                    disp('')
                end
            end

            % execute sub controller for 1-veh scenario
            info_v = sub_controller(scenario_v, iter_v);
            
            if info_v.is_exhausted
                % if graph search is exhausted, this vehicles and all vehicles that have directed or
                % undirected couplings with this vehicle will take fallback 
                disp(['Graph search exhausted for vehicle ' num2str(scenario.vehicle_ids(vehicle_idx)) ', at time step: ' num2str(scenario.k) '.'])
                sub_graph_fallback = belonging_vector_total(vehicle_idx);
                info.vehs_fallback = [info.vehs_fallback, find(belonging_vector_total==sub_graph_fallback)];
                info.vehs_fallback = unique(info.vehs_fallback,'stable');
                info.is_exhausted(vehicle_idx) = true;
            else
                info = store_control_info(info, info_v, scenario);
            end
            info.subcontroller_runtime(vehicle_idx) = toc(subcontroller_timer);
        end

    end

    % total runtime of subcontroller
    info.subcontroller_runtime = info.subcontroller_runtime + runtime_others;

    % calculate the total runtime: only one vehicle in each computation level will be counted, this is the one with the maximum runtime 
    parl_groups_info = struct('vertices',1:nVeh,'num_CLs',computation_levels,'path_info',[]); % one group
    info = get_run_time_total_all_grps(info, parl_groups_info, groups);
end

