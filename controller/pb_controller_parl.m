function [info, scenario] = pb_controller_parl(scenario, iter)
% PB_CONTROLLER_PARL Plan trajectory for one time step using a
% priority-based controller. Vehicles inside one group plan in sequence and
% between groups plan in pararllel. Controller simulates multiple
% distributed controllers in a for-loop. 


switch scenario.priority_option
    case 'topo_priority'
        obj = topo_priority(scenario);
        groups = obj.priority(); 
        right_of_way = false;
        vehs_at_intersection = [];
        edge_to_break = [];
    case 'right_of_way_priority'
        [scenario,CL_based_hierarchy,lanelet_intersecting_areas] = right_of_way_priority().priority_parl(scenario, iter);
    case 'constant_priority'
        obj = constant_priority(scenario);
        groups = obj.priority(); 
        right_of_way = false;
        vehs_at_intersection = [];
        edge_to_break = [];
    case 'random_priority' 
        obj = random_priority(scenario);
        groups = obj.priority(); 
        right_of_way = false;
        vehs_at_intersection = [];
        edge_to_break = [];
    case 'FCA_priority'
        obj = FCA_priority(scenario,iter);
        [vehs_at_intersection, groups] = obj.priority();
        right_of_way = false;
        edge_to_break = [];   
    case 'mixed_traffic_priority'
        obj = mixed_traffic_priority(scenario);
        [groups, directed_adjacency] = obj.priority(); 
        right_of_way = false;
        veh_at_intersection = [];
        edge_to_break = [];
end

    nVeh = scenario.nVeh;
    Hp = scenario.Hp;

    if ~strcmp(scenario.priority_option, 'mixed_traffic_priority')
         % visualize the coupling between vehicles
        % plot_coupling_lines(coupling_weights, iter.x0, belonging_vector, 'ShowWeights', true)
        
        % initialize variable to store control results
        info = ControllResultsInfo(nVeh, Hp, [scenario.vehicles.ID]);

        % graph-search to select the optimal motion primitive
        sub_controller = @(scenario, iter)...
            graph_search(scenario, iter); 

        directed_graph = digraph(scenario.coupling_weights);
        [belonging_vector_total,~] = conncomp(directed_graph,'Type','weak'); % graph decomposition
        
        for level_j = 1:length(CL_based_hierarchy)
            vehs_level_i = CL_based_hierarchy(level_j).members; % vehicles of all groups in the same computation level
            
            for vehicle_idx = vehs_level_i
                if ismember(vehicle_idx, info.vehs_fallback)
                    % jump to next vehicle if the selected vehicle should take fallback
                    info.subcontroller_runtime(vehicle_idx) = 0;
                    continue
                end

                % only keep self
                filter_self = false(1,scenario.nVeh);
                filter_self(vehicle_idx) = true;
                scenario_v = filter_scenario(scenario, filter_self);
                iter_v = filter_iter(iter, filter_self);

                grp_idx = arrayfun(@(array) ismember(vehicle_idx,array.vertices), scenario_v.parl_groups_info);
                all_vehs_same_grp = scenario_v.parl_groups_info(grp_idx).vertices; % all vehicles in the same group

                all_coupled_vehs_with_ROW = find(scenario_v.coupling_weights(:,vehicle_idx)~=0)'; % all coupled vehicles with the right-of-way
                all_coupled_vehs_without_ROW = find(scenario_v.coupling_weights(vehicle_idx,:)~=0); % all coupled vehicles without the right-of-way
    
                coupling_vehs_same_grp_with_ROW = intersect(all_coupled_vehs_with_ROW, all_vehs_same_grp); % coupled vehicles with the right-of-way in the same group
                coupling_vehs_other_grps_with_ROW = setdiff(all_coupled_vehs_with_ROW, coupling_vehs_same_grp_with_ROW); % coupled vehicles with the right-of-way in other groups
                
                % Collisions with coupled vehicles with the right-of-way will be avoided by two ways depending on the time step at which their latest messages are sent:
                % 1. Their predicted occupied areas will be considered as dynamic obstacles if the latest messages come from the current time step. 
                % 2. Their reachable sets will be considered as dynamic obstacles if the latest messages come from past time step. 
                for i_HP = 1:length(all_coupled_vehs_with_ROW)
                    veh_with_ROW_i = all_coupled_vehs_with_ROW(i_HP);
                    
                    if ismember(veh_with_ROW_i,coupling_vehs_same_grp_with_ROW)
                        % if in the same group, read the current message and
                        % set the predicted occupied areas as dynamic obstacles  
                        latest_msg = read_message(scenario_v.vehicles.communicate, scenario_v.ros_subscribers{veh_with_ROW_i}, scenario_v.k);
                        predicted_areas_i = arrayfun(@(array) {[array.x';array.y']}, latest_msg.predicted_areas);
                        oldness_msg = scenario_v.k - latest_msg.time_step;
                        if oldness_msg ~= 0
                            % consider the oldness of the message: delete the
                            % first n entries and repeat the last entry for n times
                            predicted_areas_i = del_first_rpt_last(predicted_areas_i,oldness_msg);
                        end
                        scenario_v.dynamic_obstacle_area(end+1,:) = predicted_areas_i;
                    else
                        % if the selected vehicle is not in the same group, add their reachable sets as dynamic obstacles 
                        reachable_sets_i = iter.reachable_sets(veh_with_ROW_i,:);
                        % turn polyshape to plain array (repeat the first row to enclosed the shape)
                        reachable_sets_i_array = cellfun(@(c) {[c.Vertices(:,1)',c.Vertices(1,1)';c.Vertices(:,2)',c.Vertices(1,2)']}, reachable_sets_i); 
                        scenario_v.dynamic_obstacle_reachableSets(end+1,:) = reachable_sets_i_array;
                    end
                end

                % Set lanelet intersecting areas as static obstacles if vehicle without the right-of-way is not allowed to enter those area
                scenario_v.lanelet_intersecting_areas = lanelet_intersecting_areas{vehicle_idx};
                if isempty(scenario_v.lanelet_intersecting_areas)
                    scenario_v.lanelet_intersecting_areas = {}; % convert from 'double' to 'cell'
                end
                assert(iscell(scenario_v.lanelet_intersecting_areas));

                % consider coupled vehicles without the right-of-way
                scenario_v = consider_vehs_without_ROW(scenario_v, iter, all_coupled_vehs_without_ROW);

                if scenario.k>=278 || scenario.k==87
                    if scenario_v.vehicles.ID==5
                        disp('')
    %                     plot_obstacles(scenario_v)
    %                     pause(0.5)
                    end
                end

                subcontroller_timer = tic;
                % execute sub controller for 1-veh scenario
                info_v = sub_controller(scenario_v, iter_v);
                info.subcontroller_runtime(vehicle_idx) = toc(subcontroller_timer);

                if info_v.is_exhausted
                    % if graph search is exhausted, this vehicles and all vehicles that have directed or
                    % undirected couplings with this vehicle will take fallback 
                    disp(['Graph search exhausted for vehicle ' num2str(vehicle_idx) ', at time step: ' num2str(scenario.k) '.'])
                    sub_graph_fallback = belonging_vector_total(vehicle_idx);
                    info.vehs_fallback = [info.vehs_fallback, find(belonging_vector_total==sub_graph_fallback)];
                    info.vehs_fallback = unique(info.vehs_fallback,'stable');
                    info.is_exhausted(vehicle_idx) = true;
                else
                    info = store_control_info(info, info_v, scenario);
                end
                
                if scenario.k>=2
    %                 plot_obstacles(scenario_v)
    %                 pause(0.5)
    %                 plot_obstacles(info_v.shapes)
    %                 pause(0.5)
    %                 graphs_visualization(belonging_vector, coupling_weights, 'ShowWeights', true)
                end
            end
            
            % Communicate data to other vehicles
            % Trick: vehicles will wait for the last vehicle in the same computation level has planned 
            % to avoid receiving messages of the current time step from vehicles in the same computation level.         
            for vehicle_k = vehs_level_i
                if ismember(vehicle_k, info.vehs_fallback)
                    % if the selected vehicle should take fallback
                    continue
                end
                predicted_trims = info.predicted_trims(vehicle_k,:); % including the current trim
                trim_current = predicted_trims(2);

                states_current = info.y_predicted{vehicle_k}(1,:);
                x0 = states_current(indices().x);
                y0 = states_current(indices().y);

                [predicted_lanelets,~,~] = get_predicted_lanelets(scenario.vehicles(vehicle_k), trim_current, x0, y0, scenario.mpa, scenario.dt, scenario.options.isParl);
                predicted_areas_k = info.shapes(vehicle_k,:);

                % send message
                send_message(scenario.vehicles(vehicle_k).communicate, scenario.k, predicted_trims, predicted_lanelets, predicted_areas_k);
            end

        end

        % Calculate the total runtime of each group
        info = get_run_time_total_all_grps(info, scenario.parl_groups_info, CL_based_hierarchy);
    else
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
        
        % update properties of scenario
        scenario.directed_coupling = directed_adjacency;
        scenario.priority_list = priority_list;
        scenario.last_veh_at_intersection = veh_at_intersection;

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
            vehicle_idx = members_list(grp_idx);
            for i = 1:nVeh
               
                subcontroller_timer = tic;
        
                % Filter out vehicles that are not adjacent
                % TODO: function to get adjacent vehicles based on
                % reachable set / RSS
                %veh_adjacent = find(scenario.adjacency(vehicle_idx,:,end));
                %predecessors = intersect(group.predecessors,veh_adjacent);
                predecessors = group.predecessors;

                % only keep self
                filter_self = false(1,scenario.nVeh);
                filter_self(vehicle_idx) = true;
                scenario_v = filter_scenario(scenario, filter_self);
                iter_v = filter_iter(iter, filter_self);

                % only keep vehicle i
                filter = false(1,scenario.nVeh);
                filter(i) = true;
                scenario_filtered = filter_scenario(scenario, filter);
                iter_filtered = filter_iter(iter, filter);

                % Filter out vehicles with lower or same priority.
                %{
                priority_filter = false(1,scenario.nVeh);
                priority_filter(predecessors) = true; % keep all with higher priority
                priority_filter(vehicle_idx) = true; % keep self
                scenario_filtered = filter_scenario(scenario, priority_filter);
                iter_filtered = filter_iter(iter, priority_filter);

                self_index = sum(priority_filter(1:vehicle_idx));        
                v2o_filter = true(1,scenario_filtered.nVeh);
                v2o_filter(self_index) = false;
                %}
                
                if ~ismember(i,predecessors)
                    % if lower priority, read the current message and
                    % set the predicted occupied areas as dynamic obstacles  

                    %{
                    latest_msg = read_message(scenario_filtered.vehicles.communicate, scenario_filtered.ros_subscribers{i}, scenario_filtered.k);
                    predicted_areas_HP = arrayfun(@(array) {[array.x';array.y']}, latest_msg.predicted_areas);
                    oldness_msg = scenario_filtered.k - latest_msg.time_step;
                    if oldness_msg ~= 0
                        % consider the oldness of the message: delete the
                        % first n entries and repeat the last entry for n times
                        predicted_areas_HP = del_first_rpt_last(predicted_areas_HP,oldness_msg);
                    end
                    scenario_v.dynamic_obstacle_area(end+1,:) = predicted_areas_HP;
                    %}
                else
                    % if the selected vehicle has higher priority, add their reachable sets as dynamic obstacles 
                    reachable_sets_HP = iter.reachable_sets(i,:);
                    % turn polyshape to plain array
                    reachable_sets_HP_array = cellfun(@(c) {[c.Vertices(:,1)';c.Vertices(:,2)']}, reachable_sets_HP);
                    scenario_v.dynamic_obstacle_reachableSets(end+1,:) = reachable_sets_HP_array;
                end

                %{
                scenario_v = filter_scenario(scenario, ~v2o_filter);
                iter_v = filter_iter(iter, ~v2o_filter);
                %}

                %{
                if ~ismember(i,predecessors)
                    % if lower priority, read the current message and
                    % set the predicted occupied areas as dynamic obstacles  
                    latest_msg = read_message(scenario_v.vehicles.communicate, scenario_v.ros_subscribers{i}, scenario_v.k);
                    predicted_areas_HP = arrayfun(@(array) {[array.x';array.y']}, latest_msg.predicted_areas);
                    oldness_msg = scenario_v.k - latest_msg.time_step;
                    if oldness_msg ~= 0
                        % consider the oldness of the message: delete the
                        % first n entries and repeat the last entry for n times
                        predicted_areas_HP = del_first_rpt_last(predicted_areas_HP,oldness_msg);
                    end
                    scenario_v.dynamic_obstacle_area(end+1,:) = predicted_areas_HP;
                else
                    % if the selected vehicle has higher priority, add their reachable sets as dynamic obstacles 
                    reachable_sets_HP = iter.reachable_sets(i,:);
                    % turn polyshape to plain array
                    reachable_sets_HP_array = cellfun(@(c) {[c.Vertices(:,1)';c.Vertices(:,2)']}, reachable_sets_HP);
                    scenario_v.dynamic_obstacle_reachableSets(end+1,:) = reachable_sets_HP_array;
                end
                %}
                
                [u_v,y_pred_v,info_v] = sub_controller(scenario_v, iter_v);
                
                % prepare output data
                info.tree{i} = info_v.tree;
                info.tree_path(i,:) = info_v.tree_path;
                info.subcontroller_runtime(i) = toc(subcontroller_timer);
                info.n_expanded = info.n_expanded + info_v.tree.size();
                info.next_node = set_node(info.next_node,i,info_v);
                info.shapes(i,:) = info_v.shapes(:);
                info.vehicle_fullres_path(i) = path_between(info_v.tree_path(1),info_v.tree_path(2),info_v.tree,scenario);
                info.trim_indices(i) = info_v.trim_indices;
                info.predicted_trims(i,:) = info_v.predicted_trims; % store the planned trims in the future Hp time steps
                info.trees{i} = info_v.tree; % store tree information
                info.y_predicted{i,1} = y_pred_v{:}; % store the information of the predicted output
                y_pred{i,1} = y_pred_v{:};
                u(i) = u_v(1);
                
            end

            % Communicate data to other vehicles
            % Trick: vehicles will wait for the last vehicle in the same computation level has planned 
            % to avoid receiving messages of the current time step from vehicles in the same computation level.         
            for vehicle_k = 1:nVeh
                predicted_trims = info.predicted_trims(vehicle_k,:); % including the current trim
                
                trim_current = predicted_trims(2);

                states_current = info.y_predicted{vehicle_k}(1,:);
                x0 = states_current(indices().x);
                y0 = states_current(indices().y);

                if (scenario.manual_vehicle_id == scenario.vehicle_ids(vehicle_k) && scenario.manual_mpa_initialized)
                    mpa = scenario.vehicles(vehicle_k).vehicle_mpa;
                elseif (scenario.second_manual_vehicle_id == scenario.vehicle_ids(vehicle_k) && scenario.second_manual_mpa_initialized)
                    mpa = scenario.vehicles(vehicle_k).vehicle_mpa;
                else
                    mpa = scenario.mpa;
                end

                [predicted_lanelets,~,~] = get_predicted_lanelets(scenario.vehicles(vehicle_k), trim_current, x0, y0, mpa, scenario.dt);
                predicted_areas = info.shapes(vehicle_k,:);

                % send message
                send_message(scenario.vehicles(vehicle_k).communicate, scenario.k, predicted_trims, predicted_lanelets, predicted_areas);
            end

        end

        % calculate the total run time: only one vehicle in each computation level will be counted, this is the one with the maximum run time 
        subcontroller_run_time_total = 0;
        for level_i = 1:computation_levels
            vehs_in_level_i = groups(level_i).members;
            subcontroller_run_time_total = subcontroller_run_time_total + max(info.subcontroller_runtime(vehs_in_level_i));
        end

        info.subcontroller_run_time_total = subcontroller_run_time_total;
    end
end


