function [info, scenario] = pb_controller_mixed_traffic(scenario, iter)
% PB_CONTROLLER    Plan trajectory for one time step using a priority-based controller.
%     Controller simulates multiple distributed controllers.

    if strcmp(scenario.priority_option, 'mixed_traffic_priority')
        obj = mixed_traffic_priority(scenario);
        [groups, directed_adjacency] = obj.priority(); 
        right_of_way = false;
        veh_at_intersection = [];
    else
        disp("Wrong priority option for mixed traffic controller");
    end

    % construct the priority list
    computation_levels = length(groups);
    members_list = horzcat(groups.members);
    
    nVeh = length(members_list); 
    Hp = scenario.Hp;

    priority_list = zeros(1,nVeh);
    prio = 1;
    for iVeh = members_list 
        priority_list(iVeh) = prio;
        prio = prio + 1;
    end
    
    % update properties of scenario
    scenario.directed_coupling = directed_adjacency;
    scenario.priority_list = priority_list;
    scenario.last_vehs_at_intersection = veh_at_intersection;
    
    % initialize variable to store control results
    info = ControllResultsInfo(nVeh, Hp, [scenario.vehicles.ID]);

    % get coupling weigths
    [coupling_weights, coupling_info] = get_coupling_info_mixed_traffic(scenario, iter);
    
    % graph-search to select the optimal motion primitive
    sub_controller = @(scenario, iter)...
        graph_search(scenario, iter); 

    directed_graph = digraph(directed_adjacency);
    [belonging_vector_total,~] = conncomp(directed_graph,'Type','weak'); % graph decomposition

    for grp_idx = 1:length(groups)
        group = groups(grp_idx);
        for grp_member_idx = 1:length(group.members) 
            vehicle_idx = group.members(grp_member_idx);
            if ismember(vehicle_idx, info.vehs_fallback)
                % if the selected vehicle should take fallback
                info.subcontroller_runtime(vehicle_idx) = 0;
                continue
            end
            
            if scenario.vehicle_ids(vehicle_idx) ~= scenario.manual_vehicle_id && scenario.vehicle_ids(vehicle_idx) ~= scenario.second_manual_vehicle_id

                predecessors = group.predecessors;
                index_first_manual_vehicle = 0;
                index_second_manual_vehicle = 0;

                coupled_manual_vehicles = find(coupling_weights(vehicle_idx,:)~=0); % indices of the coupled manual vehicles

                for j = 1:length(predecessors)
                    if scenario.vehicle_ids(predecessors(j)) == scenario.manual_vehicle_id
                        % predecessor is first manual vehicle

                        if ismember(predecessors(j), coupled_manual_vehicles)
                            % manual vehicle is predecessor and coupled
                            index_first_manual_vehicle = j;
                        else
                            index_first_manual_vehicle = 0;
                        end
                    elseif scenario.vehicle_ids(predecessors(j)) == scenario.second_manual_vehicle_id
                        % predecessor is second manual vehicle
                    
                        if ismember(predecessors(j), coupled_manual_vehicles)
                            % manual vehicle is predecessor and coupled
                            index_second_manual_vehicle = j;
                        else
                            index_second_manual_vehicle = 0;
                        end
                    end
                end

                % only keep self
                filter_self = false(1,scenario.nVeh);
                filter_self(vehicle_idx) = true;
                scenario_v = filter_scenario(scenario, filter_self);
                iter_v = filter_iter(iter, filter_self);

                if index_first_manual_vehicle ~= 0
                    % TODO: function to get adjacent vehicles based on reachable set / RSS
                    % add reachable set of first manual vehicle as dynamic obstacle
                    index_first_manual_vehicle = predecessors(index_first_manual_vehicle);
                    reachable_sets_i = iter.reachable_sets(index_first_manual_vehicle,:);
                    % turn polyshape to plain array (repeat the first row to enclosed the shape)
                    reachable_sets_i_array = cellfun(@(c) {[c.Vertices(:,1)',c.Vertices(1,1)';c.Vertices(:,2)',c.Vertices(1,2)']}, reachable_sets_i); 
                    scenario_v.dynamic_obstacle_reachableSets(end+1,:) = reachable_sets_i_array;

                    subcontroller_timer = tic;
                    % execute sub controller for 1-veh scenario
                    info_v = sub_controller(scenario_v, iter_v);
                    info.subcontroller_runtime(vehicle_idx) = toc(subcontroller_timer);

                    if info_v.is_exhausted
                        % if graph search is exhausted, only autonomous vehicles take fallback to prevent collision with first manual vehicle 
                        disp(['Graph search exhausted for vehicle ' num2str(scenario.vehicle_ids(vehicle_idx)) ', at time step: ' num2str(scenario.k) '.'])
                        %sub_graph_fallback = belonging_vector_total(vehicle_idx);
                        %info.vehs_fallback = [info.vehs_fallback, find(belonging_vector_total==sub_graph_fallback)];
                        %info.vehs_fallback = unique(info.vehs_fallback,'stable');
                        %info.vehs_fallback = setdiff(info.vehs_fallback, index_first_manual_vehicle);
                        info.vehs_fallback = [info.vehs_fallback, vehicle_idx];
                        info.is_exhausted(vehicle_idx) = true;
                    else
                        info = store_control_info(info, info_v, scenario);
                    end
                end

                if index_second_manual_vehicle ~= 0
                    % TODO: function to get adjacent vehicles based on reachable set / RSS
                    % add reachable set of first manual vehicle as dynamic obstacle
                    index_second_manual_vehicle = predecessors(index_second_manual_vehicle);
                    reachable_sets_i = iter.reachable_sets(index_second_manual_vehicle,:);
                    % turn polyshape to plain array (repeat the first row to enclosed the shape)
                    reachable_sets_i_array = cellfun(@(c) {[c.Vertices(:,1)',c.Vertices(1,1)';c.Vertices(:,2)',c.Vertices(1,2)']}, reachable_sets_i); 
                    scenario_v.dynamic_obstacle_reachableSets(end+1,:) = reachable_sets_i_array;

                    subcontroller_timer = tic;
                    % execute sub controller for 1-veh scenario
                    info_v = sub_controller(scenario_v, iter_v);
                    info.subcontroller_runtime(vehicle_idx) = toc(subcontroller_timer);

                    if info_v.is_exhausted
                        % if graph search is exhausted, only autonomous vehicles take fallback to prevent collision with second manual vehicle 
                        disp(['Graph search exhausted for vehicle ' num2str(scenario.vehicle_ids(vehicle_idx)) ', at time step: ' num2str(scenario.k) '.'])
                        %sub_graph_fallback = belonging_vector_total(vehicle_idx);
                        %info.vehs_fallback = [info.vehs_fallback, find(belonging_vector_total==sub_graph_fallback)];
                        %info.vehs_fallback = unique(info.vehs_fallback,'stable');
                        %info.vehs_fallback = setdiff(info.vehs_fallback, index_second_manual_vehicle);
                        info.vehs_fallback = [info.vehs_fallback, vehicle_idx];
                        info.is_exhausted(vehicle_idx) = true;
                    else
                        info = store_control_info(info, info_v, scenario);
                    end
                end

                manual_indices = [index_first_manual_vehicle index_second_manual_vehicle];
                autonomous_vehicle_indices = setdiff(predecessors, manual_indices);
                
                % Filter out vehicles that are not adjacent
                veh_adjacent = find(scenario.adjacency(vehicle_idx,:,end));
                autonomous_vehicles_adjacent = intersect(autonomous_vehicle_indices,veh_adjacent);

                % Filter out vehicles with lower or same priority.
                priority_filter = false(1,scenario.nVeh);
                priority_filter(autonomous_vehicles_adjacent) = true; % keep all with higher priority
                priority_filter(vehicle_idx) = true; % keep self
                scenario_filtered = filter_scenario(scenario, priority_filter);
                iter_filtered = filter_iter(iter, priority_filter);

                self_index = sum(priority_filter(1:vehicle_idx));        
                v2o_filter = true(1,scenario_filtered.nVeh);
                v2o_filter(self_index) = false;

                % add predicted trajecotries of vehicles with higher priority as dynamic obstacle
                [scenario_v, iter_v] = vehicles_as_dynamic_obstacles(scenario_filtered, iter_filtered, v2o_filter, info.shapes(autonomous_vehicles_adjacent,:));

                subcontroller_timer = tic;
                % execute sub controller for 1-veh scenario
                info_v = sub_controller(scenario_v, iter_v);
                info.subcontroller_runtime(vehicle_idx) = toc(subcontroller_timer);

                if info_v.is_exhausted
                    % if graph search is exhausted, this vehicles and all autonomous vehicles that have couplings will take fallback 
                    disp(['Graph search exhausted for vehicle ' num2str(scenario.vehicle_ids(vehicle_idx)) ', at time step: ' num2str(scenario.k) '.'])
                    %sub_graph_fallback = belonging_vector_total(vehicle_idx);
                    %info.vehs_fallback = [info.vehs_fallback, find(belonging_vector_total==sub_graph_fallback)];
                    %info.vehs_fallback = unique(info.vehs_fallback,'stable');
                    info.vehs_fallback = [info.vehs_fallback, vehicle_idx];
                    info.is_exhausted(vehicle_idx) = true;
                else
                    info = store_control_info(info, info_v, scenario);
                end
            else
                % manually-controlled vehicles do not prevent collisions automatically
                 % only keep self
                filter_self = false(1,scenario.nVeh);
                filter_self(vehicle_idx) = true;
                scenario_v = filter_scenario(scenario, filter_self);
                iter_v = filter_iter(iter, filter_self);
                
                subcontroller_timer = tic;
                % execute sub controller for 1-veh scenario
                info_v = sub_controller(scenario_v, iter_v);
                info.subcontroller_runtime(vehicle_idx) = toc(subcontroller_timer);
    
                if info_v.is_exhausted
                    % if graph search is exhausted, this manual vehicle will take fallback
                    disp(['Graph search exhausted for vehicle ' num2str(scenario.vehicle_ids(vehicle_idx)) ', at time step: ' num2str(scenario.k) '.'])
                    %sub_graph_fallback = belonging_vector_total(vehicle_idx);
                    %info.vehs_fallback = [info.vehs_fallback, find(belonging_vector_total==sub_graph_fallback)];
                    %info.vehs_fallback = unique(info.vehs_fallback,'stable');
                    info.vehs_fallback = [info.vehs_fallback, vehicle_idx];
                    info.is_exhausted(vehicle_idx) = true;
                else
                    info = store_control_info(info, info_v, scenario);
                end
            end

            if ~ismember(vehicle_idx, info.vehs_fallback)
                % communicate data to other vehicles
                predicted_trims = info.predicted_trims(vehicle_idx,:); % including the current trim
                trim_current = predicted_trims(2);

                states_current = info.y_predicted{vehicle_idx}(1,:);
                x0 = states_current(indices().x);
                y0 = states_current(indices().y);

                if (scenario.manual_vehicle_id == scenario.vehicle_ids(vehicle_idx) && scenario.manual_mpa_initialized)
                    mpa = scenario.vehicles(vehicle_idx).vehicle_mpa;
                elseif (scenario.second_manual_vehicle_id == scenario.vehicle_ids(vehicle_idx) && scenario.second_manual_mpa_initialized)
                    mpa = scenario.vehicles(vehicle_idx).vehicle_mpa;
                else
                    mpa = scenario.mpa;
                end

                [predicted_lanelets,~,~] = get_predicted_lanelets(scenario.vehicles(vehicle_idx), trim_current, x0, y0, mpa, scenario.dt, scenario.name, scenario.options.isParl, scenario.vehicles(vehicle_idx).autoUpdatedPath);
                predicted_areas = info.shapes(vehicle_idx,:);

                % send message
                send_message(scenario.vehicles(vehicle_idx).communicate, scenario.k, predicted_trims, predicted_lanelets, predicted_areas);
            end
            
        end

    end

    % calculate the total runtime: only one vehicle in each computation level will be counted, this is the one with the maximum runtime 
    parl_groups_info = struct('vertices',1:nVeh,'num_CLs',computation_levels,'path_info',[]); % one group
    info = get_run_time_total_all_grps(info, parl_groups_info, groups);
end
