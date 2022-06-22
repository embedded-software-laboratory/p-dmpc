function [info, scenario] = pb_controller_parl(scenario, iter)
% PB_CONTROLLER_PARL Plan trajectory for one time step using a
% priority-based controller. Vehicles inside one group plan in sequence and
% between groups plan in pararllel. Controller simulates multiple
% distributed controllers in a for-loop. 

runtime_others_tic = tic;
right_of_way = false;
switch scenario.priority_option
    case 'topo_priority' 
        [groups, directed_adjacency, priority_list] = topo_priority().priority(scenario); 
        veh_at_intersection = [];
%         edge_to_break = [];
    case 'right_of_way_priority' 
        right_of_way = true;
        [scenario,CL_based_hierarchy,lanelet_intersecting_areas] = right_of_way_priority().priority_parl(scenario, iter);  
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
end

    nVeh = scenario.nVeh;
    Hp = scenario.Hp;

    % visualize the coupling between vehicles
    % plot_coupling_lines(coupling_weights, iter.x0, belonging_vector, 'ShowWeights', true)
    
    % initialize variable to store control results
    info = ControllResultsInfo(nVeh, Hp, [scenario.vehicles.ID]);

    % graph-search to select the optimal motion primitive
    sub_controller = @(scenario, iter)...
        graph_search(scenario, iter); 

    directed_graph = digraph(scenario.coupling_weights);
    [belonging_vector_total,~] = conncomp(directed_graph,'Type','weak'); % graph decomposition
    
    runtime_others = toc(runtime_others_tic); % subcontroller runtime except for runtime of graph search
    msg_send_time = zeros(nVeh,1);

    for level_j = 1:length(CL_based_hierarchy)
        vehs_level_i = CL_based_hierarchy(level_j).members; % vehicles of all groups in the same computation level
        
        for vehicle_idx = vehs_level_i
            if ismember(vehicle_idx, info.vehs_fallback)
                % jump to next vehicle if the selected vehicle should take fallback
                info.subcontroller_runtime(vehicle_idx) = 0;
                continue
            end
            subcontroller_timer = tic;

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

            
            % execute sub controller for 1-veh scenario
            info_v = sub_controller(scenario_v, iter_v);

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
            info.subcontroller_runtime(vehicle_idx) = toc(subcontroller_timer);

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
            msg_send_tic = tic;
            predicted_trims = info.predicted_trims(vehicle_k,:); % including the current trim
            trim_current = predicted_trims(2);

            states_current = info.y_predicted{vehicle_k}(1,:);
            x0 = states_current(indices().x);
            y0 = states_current(indices().y);

            [predicted_lanelets,~,~] = get_predicted_lanelets(scenario.vehicles(vehicle_k), trim_current, x0, y0, scenario.mpa, scenario.dt, scenario.options.isParl);
            predicted_areas_k = info.shapes(vehicle_k,:);

            % send message
            send_message(scenario.vehicles(vehicle_k).communicate, scenario.k, predicted_trims, predicted_lanelets, predicted_areas_k);
            msg_send_time(vehicle_k) = toc(msg_send_tic);
        end

    end

    % total runtime of subcontroller
    info.subcontroller_runtime = info.subcontroller_runtime + msg_send_time + runtime_others;
    % Calculate the total runtime of each group
    info = get_run_time_total_all_grps(info, scenario.parl_groups_info, CL_based_hierarchy);
end

