function [info, scenario] = pb_controller_parl(scenario, iter)
% PB_CONTROLLER_PARL Plan trajectory for one time step using a
% priority-based controller. Vehicles inside one group plan in sequence and
% between groups plan in pararllel. Controller simulates multiple
% distributed controllers in a for-loop. 

    runtime_others_tic = tic;
    [scenario,CL_based_hierarchy,lanelet_intersecting_areas] = priority_assignment_parl(scenario, iter);

    nVeh = scenario.nVeh;
    Hp = scenario.Hp;

    % visualize the coupling between vehicles
    % plot_coupling_lines(coupling_weights, iter.x0, belonging_vector, 'ShowWeights', true)
    
    % initialize variable to store control results
    info = ControllResultsInfo(nVeh, Hp, [scenario.vehicles.ID]);

    % graph-search to select the optimal motion primitive
    sub_controller = @(scenario, iter)...
        graph_search(scenario, iter); 

    directed_graph = digraph(scenario.directed_coupling);
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

            all_coupled_vehs_with_HP = find(scenario_v.directed_coupling(:,vehicle_idx)==1)'; % all coupled vehicles with higher priorities
            all_coupled_vehs_with_LP = find(scenario_v.directed_coupling(vehicle_idx,:)==1); % all coupled vehicles with lower priorities
 
            coupled_vehs_same_grp_with_HP = intersect(all_coupled_vehs_with_HP, all_vehs_same_grp); % coupled vehicles with higher priorities in the same group
            coupled_vehs_other_grps_with_HP = setdiff(all_coupled_vehs_with_HP, coupled_vehs_same_grp_with_HP); % coupled vehicles with higher priorities in other groups
            
            % Collisions with coupled vehicles with higher priorities will be avoided by two ways depending on the time step at which their latest messages are sent:
            % 1. Their predicted occupied areas will be considered as dynamic obstacles if the latest messages come from the current time step. 
            % 2. Their reachable sets will be considered as dynamic obstacles if the latest messages come from past time step. 
            for i_HP = 1:length(all_coupled_vehs_with_HP)
                veh_with_HP_i = all_coupled_vehs_with_HP(i_HP);
                
                if ismember(veh_with_HP_i,coupled_vehs_same_grp_with_HP)
                    % if in the same group, read the current message and
                    % set the predicted occupied areas as dynamic obstacles  
                    latest_msg = read_message(scenario_v.vehicles.communicate, scenario_v.ros_subscribers{veh_with_HP_i}, scenario_v.k);
                    predicted_areas_i = arrayfun(@(array) {[array.x(:)';array.y(:)']}, latest_msg.predicted_areas);
                    oldness_msg = scenario_v.k - latest_msg.time_step;
                    if oldness_msg ~= 0
                        % consider the oldness of the message: delete the
                        % first n entries and repeat the last entry for n times
                        predicted_areas_i = del_first_rpt_last(predicted_areas_i,oldness_msg);
                    end
                    scenario_v.dynamic_obstacle_area(end+1,:) = predicted_areas_i;
                else
                    % if the selected vehicle is not in the same group, add their reachable sets as dynamic obstacles 
                    reachable_sets_i = iter.reachable_sets(veh_with_HP_i,:);
                    % turn polyshape to plain array (repeat the first row to enclosed the shape)
                    reachable_sets_i_array = cellfun(@(c) {[c.Vertices(:,1)',c.Vertices(1,1)';c.Vertices(:,2)',c.Vertices(1,2)']}, reachable_sets_i); 
                    scenario_v.dynamic_obstacle_reachableSets(end+1,:) = reachable_sets_i_array;
                end
            end

            if ~strcmp(scenario.strategy_enter_intersecting_area,'1')
                % Set lanelet intersecting areas as static obstacles if vehicle with lower priorities is not allowed to enter those area
                scenario_v.lanelet_intersecting_areas = lanelet_intersecting_areas{vehicle_idx};
                if isempty(scenario_v.lanelet_intersecting_areas)
                    scenario_v.lanelet_intersecting_areas = {}; % convert from 'double' to 'cell'
                end
                assert(iscell(scenario_v.lanelet_intersecting_areas));
            end

            % consider coupled vehicles with lower priorities
            scenario_v = consider_vehs_with_LP(scenario_v, iter, all_coupled_vehs_with_LP);

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
                disp(['Graph search exhausted for vehicle ' num2str(scenario.vehicle_ids(vehicle_idx)) ', at time step: ' num2str(scenario.k) '.'])
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

            if ((scenario.vehicle_ids(vehicle_k) == scenario.manual_vehicle_id) && scenario.manual_mpa_initialized) ...
                || ((scenario.vehicle_ids(vehicle_k) == scenario.second_manual_vehicle_id) && scenario.second_manual_mpa_initialized)
                mpa = scenario.vehicles(vehicle_k).vehicle_mpa;
            else
                mpa = scenario.mpa;
            end

            [predicted_lanelets,~,~] = get_predicted_lanelets(scenario.vehicles(vehicle_k), trim_current, x0, y0, mpa, scenario.dt, scenario.options.isParl, scenario.name, scenario.vehicles(vehicle_k).autoUpdatedPath, scenario.vehicles(vehicle_k).last_trajectory_index);
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


