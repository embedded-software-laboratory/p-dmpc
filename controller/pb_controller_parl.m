function [info, scenario] = pb_controller_parl(scenario, iter)
% PB_CONTROLLER_PARL Plan trajectory for one time step using a
% priority-based controller. Vehicles inside one group plan in sequence and
% between groups plan in pararllel. Controller simulates multiple
% distributed controllers in a for-loop. 

    runtime_others_tic = tic;
    
    assign_priority_timer = tic;
    [scenario,iter,CL_based_hierarchy,lanelet_crossing_areas] = priority_assignment_parl(scenario, iter);
    scenario.timer.assign_priority = toc(assign_priority_timer);

    nVeh = scenario.options.amount;
    Hp = scenario.options.Hp;

    % visualize the coupling between vehicles
    % plot_coupling_lines(coupling_weights, iter.x0, belonging_vector, 'ShowWeights', true)
    
    % initialize variable to store control results
    info = ControllResultsInfo(nVeh, Hp, [scenario.vehicles.ID]);
    n_expended = zeros(nVeh,1);

    sub_controller = @scenario.sub_controller;

    directed_graph = digraph(scenario.directed_coupling);
    [belonging_vector_total,~] = conncomp(directed_graph,'Type','weak'); % graph decomposition
    
    scenario.num_couplings_between_grps = 0; % number of couplings between groups
    scenario.num_couplings_between_grps_ignored = 0; % ignored number of couplings between groups by using lanelet crossing lanelets
    for iCoupling = 1:length([scenario.coupling_info.veh_with_ROW])
        veh_ij = [scenario.coupling_info(iCoupling).veh_with_ROW,scenario.coupling_info(iCoupling).veh_without_ROW];
        is_same_grp = any(cellfun(@(c) all(ismember(veh_ij,c)),{scenario.parl_groups_info.vertices}));
        if ~is_same_grp
            scenario.num_couplings_between_grps = scenario.num_couplings_between_grps + 1;
            if scenario.coupling_info(iCoupling).is_ignored
                scenario.num_couplings_between_grps_ignored = scenario.num_couplings_between_grps_ignored + 1;
            end
        end
    end

    runtime_others = toc(runtime_others_tic); % subcontroller runtime except for runtime of graph search
    msg_send_time = zeros(1,nVeh);

    for level_j = 1:length(CL_based_hierarchy)
        vehs_level_i = CL_based_hierarchy(level_j).members; % vehicles of all groups in the same computation level
        
        for vehicle_idx = vehs_level_i
            if ismember(vehicle_idx, info.vehs_fallback)
                % jump to next vehicle if the selected vehicle should take fallback
                info.runtime_graph_search_each_veh(vehicle_idx) = 0;
                continue
            end
            subcontroller_timer = tic;

            % only keep self
            filter_self = false(1,scenario.options.amount);
            filter_self(vehicle_idx) = true;
            scenario_v = filter_scenario(scenario, filter_self);
            iter_v = filter_iter(iter, filter_self);

            grp_idx = arrayfun(@(array) ismember(vehicle_idx,array.vertices), scenario_v.parl_groups_info);
            all_vehs_same_grp = scenario_v.parl_groups_info(grp_idx).vertices; % all vehicles in the same group

            all_coupled_vehs_with_HP = find(scenario_v.directed_coupling_reduced(:,vehicle_idx)==1)'; % all coupled vehicles with higher priorities
            all_coupled_vehs_with_LP = find(scenario_v.directed_coupling_reduced(vehicle_idx,:)==1); % all coupled vehicles with lower priorities
 
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
                    % if the selected vehicle is not in the same group
                    if scenario.options.isDealPredictionInconsistency
                        % add their reachable sets as dynamic obstacles to deal
                        % with the prediction inconsistency
                        reachable_sets_i = iter.reachable_sets(veh_with_HP_i,:);
                        % turn polyshape to plain array (repeat the first row to enclosed the shape)
                        reachable_sets_i_array = cellfun(@(c) {[c.Vertices(:,1)',c.Vertices(1,1)';c.Vertices(:,2)',c.Vertices(1,2)']}, reachable_sets_i); 
                        scenario_v.dynamic_obstacle_reachableSets(end+1,:) = reachable_sets_i_array;
                    else
                        % otherwise add one-step delayed trajectories as dynamic obstacles
                        if scenario_v.k>1
                            % the old trajectories are available from the second time step onwards
                            old_msg = read_message(scenario_v.vehicles.communicate, scenario_v.ros_subscribers{veh_with_HP_i}, scenario_v.k-1);
                            predicted_areas_i = arrayfun(@(array) {[array.x(:)';array.y(:)']}, old_msg.predicted_areas);
                            oldness_msg = scenario_v.k - old_msg.time_step;
                            if oldness_msg ~= 0
                                % consider the oldness of the message: delete the
                                % first n entries and repeat the last entry for n times
                                predicted_areas_i = del_first_rpt_last(predicted_areas_i',oldness_msg);
                            end
                            scenario_v.dynamic_obstacle_area(end+1,:) = predicted_areas_i;
                        end
                    end

                end
            end

            if ~strcmp(scenario.options.strategy_enter_lanelet_crossing_area,'1')
                % Set lanelet intersecting areas as static obstacles if vehicle with lower priorities is not allowed to enter those area
                scenario_v.lanelet_crossing_areas = lanelet_crossing_areas{vehicle_idx};
                if isempty(scenario_v.lanelet_crossing_areas)
                    scenario_v.lanelet_crossing_areas = {}; % convert from 'double' to 'cell'
                end
                assert(iscell(scenario_v.lanelet_crossing_areas));
            end

            % consider coupled vehicles with lower priorities
            scenario_v = consider_vehs_with_LP(scenario_v, iter, vehicle_idx, all_coupled_vehs_with_LP);

            if scenario.k >= 1
                if vehicle_idx == 9 || vehicle_idx == 12
                    disp('')
                end
            end

            
            % execute sub controller for 1-veh scenario
%             tic
            info_v = sub_controller(scenario_v, iter_v);
%             toc
            if info_v.is_exhausted
                % if graph search is exhausted, this vehicles and all vehicles that have directed or
                % undirected couplings with this vehicle will take fallback 
                disp(['Graph search exhausted after expending node ' num2str(info_v.n_expanded) ' times for vehicle ' num2str(vehicle_idx) ', at time step: ' num2str(scenario.k) '.'])

                switch scenario.options.fallback_type
                    case 'localFallback'
                        sub_graph_fallback = belonging_vector_total(vehicle_idx);
                        info.vehs_fallback = [info.vehs_fallback, find(belonging_vector_total==sub_graph_fallback)];
                        info.vehs_fallback = unique(info.vehs_fallback,'stable');
                    case 'globalFallback'
                        % global fallback: all vehicles take fallback
                        info.vehs_fallback = 1:scenario.options.amount;
                    case 'noFallback'
                        % Fallback is disabled. Simulation will end.
                        info.vehs_fallback = 1:scenario.options.amount;
                    otherwise
                        warning("Please define one of the follows as fallback strategy: 'noFallback', 'localFallback', and 'globalFallback'.")
                end
                info.is_exhausted(vehicle_idx) = true;
            else
                info = store_control_info(info, info_v, scenario);
            end
            info.runtime_graph_search_each_veh(vehicle_idx) = toc(subcontroller_timer);
            n_expended(vehicle_idx) = info_v.tree.size();

            if scenario.k==inf
                plot_obstacles(scenario_v)
                plot_obstacles(info_v.shapes)
                graphs_visualization(scenario.belonging_vector, scenario.coupling_weights, 'ShowWeights', true)
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

            [predicted_lanelets,~,~] = get_predicted_lanelets(scenario, vehicle_k, trim_current, x0, y0);
            predicted_areas_k = info.shapes(vehicle_k,:);
            
            is_fallback = false;

            % send message
            send_message(scenario.vehicles(vehicle_k).communicate, scenario.k, predicted_trims, predicted_lanelets, predicted_areas_k, is_fallback);
            msg_send_time(vehicle_k) = toc(msg_send_tic);
        end

    end

    
    info.runtime_graph_search_each_veh = info.runtime_graph_search_each_veh + msg_send_time;
    % Calculate the total runtime of each group
    info = get_run_time_total_all_grps(info, scenario.parl_groups_info, CL_based_hierarchy, runtime_others);

    scenario.lanelet_crossing_areas = lanelet_crossing_areas;
end


