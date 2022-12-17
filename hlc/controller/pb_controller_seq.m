function [hlc] = pb_controller_seq(hlc)
% PB_CONTROLLER_PARL Plan trajectory for one time step using a
% priority-based controller. Vehicles inside one group plan in sequence and
% between groups plan in pararllel. Controller simulates multiple
% distributed controllers in a for-loop. 

    runtime_others_tic = tic;

    assign_priority_timer = tic;
    [hlc.scenario,hlc.iter,CL_based_hierarchy,lanelet_crossing_areas] = priority_assignment_parl(hlc.scenario, hlc.iter);
    hlc.scenario.timer.assign_priority = toc(assign_priority_timer);

    nVeh = hlc.scenario.options.amount;
    Hp = hlc.scenario.options.Hp;

    % initialize variable to store control results
    hlc.info = ControllResultsInfo(nVeh, Hp, [hlc.scenario.vehicles.ID]);
    n_expended = zeros(nVeh,1);

    sub_controller = hlc.sub_controller;

    directed_graph = digraph(hlc.scenario.directed_coupling);
    [belonging_vector_total,~] = conncomp(directed_graph,'Type','weak'); % graph decomposition
    
    hlc.scenario.num_couplings_between_grps = 0; % number of couplings between groups
    hlc.scenario.num_couplings_between_grps_ignored = 0; % ignored number of couplings between groups by using lanelet crossing lanelets
    for iCoupling = 1:length([hlc.scenario.coupling_info.veh_with_ROW])
        veh_ij = [hlc.scenario.coupling_info(iCoupling).veh_with_ROW,hlc.scenario.coupling_info(iCoupling).veh_without_ROW];
        is_same_grp = any(cellfun(@(c) all(ismember(veh_ij,c)),{hlc.scenario.parl_groups_info.vertices}));
        if ~is_same_grp
            hlc.scenario.num_couplings_between_grps = hlc.scenario.num_couplings_between_grps + 1;
            if hlc.scenario.coupling_info(iCoupling).is_ignored
                hlc.scenario.num_couplings_between_grps_ignored = hlc.scenario.num_couplings_between_grps_ignored + 1;
            end
        end
    end

    runtime_others = toc(runtime_others_tic); % subcontroller runtime except for runtime of graph search
    msg_send_time = zeros(1,nVeh);

    for level_j = 1:length(CL_based_hierarchy)
        vehs_level_i = CL_based_hierarchy(level_j).members; % vehicles of all groups in the same computation level
        
        for vehicle_idx = vehs_level_i
            if ismember(vehicle_idx, hlc.info.vehs_fallback)
                % jump to next vehicle if the selected vehicle should take fallback
                hlc.info.runtime_graph_search_each_veh(vehicle_idx) = 0;
                continue
            end
            subcontroller_timer = tic;

            % only keep self
            filter_self = false(1,hlc.scenario.options.amount);
            filter_self(vehicle_idx) = true;
            scenario_v = filter_scenario(hlc.scenario, filter_self);
            iter_v = filter_iter(hlc.iter, filter_self);

            grp_idx = arrayfun(@(array) ismember(vehicle_idx,array.vertices), scenario_v.parl_groups_info);
            all_vehs_same_grp = scenario_v.parl_groups_info(grp_idx).vertices; % all vehicles in the same group

            all_coupled_vehs_with_HP = find(scenario_v.directed_coupling_reduced(:,vehicle_idx)==1)'; % all coupled vehicles with higher priorities
            all_coupled_vehs_with_LP = find(scenario_v.directed_coupling_reduced(vehicle_idx,:)==1); % all coupled vehicles with lower priorities
 
            coupled_vehs_same_grp_with_HP = intersect(all_coupled_vehs_with_HP, all_vehs_same_grp); % coupled vehicles with higher priorities in the same group
            coupled_vehs_other_grps_with_HP = setdiff(all_coupled_vehs_with_HP, coupled_vehs_same_grp_with_HP); % coupled vehicles with higher priorities in other groups
            
            for i_HP = 1:length(all_coupled_vehs_with_HP)
                veh_with_HP_i = all_coupled_vehs_with_HP(i_HP);
                
                if ismember(veh_with_HP_i,coupled_vehs_same_grp_with_HP)
                    % if in the same group, read the current message and set the predicted occupied areas as dynamic obstacles  
                    latest_msg = read_message(scenario_v.vehicles.communicate, hlc.ros_subscribers{veh_with_HP_i}, scenario_v.k);
                    predicted_areas_i = arrayfun(@(array) {[array.x(:)';array.y(:)']}, latest_msg.predicted_areas);
                    oldness_msg = scenario_v.k - latest_msg.time_step;
                    if oldness_msg ~= 0
                        % consider the oldness of the message: delete the first n entries and repeat the last entry for n times
                        predicted_areas_i = del_first_rpt_last(predicted_areas_i,oldness_msg);
                    end
                    scenario_v.dynamic_obstacle_area(end+1,:) = predicted_areas_i;
                else
                    % if they are in different groups
                    if hlc.scenario.options.isDealPredictionInconsistency
                        % Collisions with coupled vehicles with higher priorities in different groups will be avoided by two ways depending on the time step at which 
                        % their latest messages are sent:
                        % 1. Their predicted occupied areas will be considered as dynamic obstacles if the latest messages come from the current time step. 
                        % 2. Their reachable sets will be considered as dynamic obstacles if the latest messages come from past time step. 
                        latest_msg = hlc.ros_subscribers{veh_with_HP_i}.LatestMessage;
                        if latest_msg.time_step == scenario_v.k
                            predicted_areas_i = arrayfun(@(array) {[array.x(:)';array.y(:)']}, latest_msg.predicted_areas);
                            scenario_v.dynamic_obstacle_area(end+1,:) = predicted_areas_i;
                        else
                            % Add their reachable sets as dynamic obstacles to deal with the prediction inconsistency
                            reachable_sets_i = hlc.iter.reachable_sets(veh_with_HP_i,:);
                            % turn polyshape to plain array (repeat the first row to enclosed the shape)
                            reachable_sets_i_array = cellfun(@(c) {[c.Vertices(:,1)',c.Vertices(1,1)';c.Vertices(:,2)',c.Vertices(1,2)']}, reachable_sets_i); 
                            scenario_v.dynamic_obstacle_reachableSets(end+1,:) = reachable_sets_i_array;
                        end
                    else
                        % otherwise add one-step delayed trajectories as dynamic obstacles
                        if scenario_v.k>1
                            % the old trajectories are available from the second time step onwards
                            old_msg = read_message(scenario_v.vehicles.communicate, hlc.ros_subscribers{veh_with_HP_i}, scenario_v.k-1);
                            predicted_areas_i = arrayfun(@(array) {[array.x(:)';array.y(:)']}, old_msg.predicted_areas);
                            oldness_msg = scenario_v.k - old_msg.time_step;
                            if oldness_msg ~= 0
                                % consider the oldness of the message: delete the first n entries and repeat the last entry for n times
                                predicted_areas_i = del_first_rpt_last(predicted_areas_i',oldness_msg);
                            end
                            scenario_v.dynamic_obstacle_area(end+1,:) = predicted_areas_i;
                        end
                    end

                end
            end

            if ~strcmp(hlc.scenario.options.strategy_enter_lanelet_crossing_area,'1')
                % Set lanelet intersecting areas as static obstacles if vehicle with lower priorities is not allowed to enter those area
                scenario_v.lanelet_crossing_areas = lanelet_crossing_areas{vehicle_idx};
                if isempty(scenario_v.lanelet_crossing_areas)
                    scenario_v.lanelet_crossing_areas = {}; % convert from 'double' to 'cell'
                end
                assert(iscell(scenario_v.lanelet_crossing_areas));
            end

            % consider coupled vehicles with lower priorities
            scenario_v = consider_vehs_with_LP(scenario_v, hlc.iter, vehicle_idx, all_coupled_vehs_with_LP);

            % execute sub controller for 1-veh scenario
            info_v = sub_controller(scenario_v, iter_v);
            if info_v.is_exhausted
                % if graph search is exhausted, this vehicles and all its weakly coupled vehicles will use their fallback trajectories
%                 disp(['Graph search exhausted after expending node ' num2str(info_v.n_expanded) ' times for vehicle ' num2str(vehicle_idx) ', at time step: ' num2str(scenario.k) '.'])
                switch hlc.scenario.options.fallback_type
                    case 'localFallback'
                        sub_graph_fallback = belonging_vector_total(vehicle_idx);
                        hlc.info.vehs_fallback = [hlc.info.vehs_fallback, find(belonging_vector_total==sub_graph_fallback)];
                        hlc.info.vehs_fallback = unique(hlc.info.vehs_fallback,'stable');
                    case 'globalFallback'
                        % global fallback: all vehicles take fallback
                        hlc.info.vehs_fallback = 1:hlc.scenario.options.amount;
                    case 'noFallback'
                        % Fallback is disabled. Simulation will end.
                        hlc.info.vehs_fallback = 1:hlc.scenario.options.amount;
                    otherwise
                        warning("Please define one of the follows as fallback strategy: 'noFallback', 'localFallback', and 'globalFallback'.")
                end
                hlc.info.is_exhausted(vehicle_idx) = true;
            else
                hlc.info = store_control_info(hlc.info, info_v, hlc.scenario);
            end
            hlc.info.runtime_graph_search_each_veh(vehicle_idx) = toc(subcontroller_timer);
            n_expended(vehicle_idx) = info_v.tree.size();

            if hlc.scenario.k==inf
                plot_obstacles(scenario_v)
                plot_obstacles(info_v.shapes)
                graphs_visualization(hlc.scenario.belonging_vector, hlc.scenario.coupling_weights, 'ShowWeights', true)
            end
        end
        
        % Communicate data to other vehicles
        % Trick: vehicles will wait for the last vehicle in the same computation level has planned 
        % to avoid receiving messages of the current time step from vehicles in the same computation level.         
        for vehicle_k = vehs_level_i
            if ismember(vehicle_k, hlc.info.vehs_fallback)
                % if the selected vehicle should take fallback
                continue
            end
            msg_send_tic = tic;
            predicted_trims = hlc.info.predicted_trims(vehicle_k,:); % including the current trim
            trim_current = predicted_trims(2);

            states_current = hlc.info.y_predicted{vehicle_k}(1,:);
            x0 = states_current(indices().x);
            y0 = states_current(indices().y);

            [predicted_lanelets,~,~] = get_predicted_lanelets(hlc.scenario, vehicle_k, trim_current, x0, y0);
            predicted_areas_k = hlc.info.shapes(vehicle_k,:);
            
            is_fallback = false;

            % send message
            send_message(hlc.scenario.vehicles(vehicle_k).communicate, hlc.scenario.k, predicted_trims, predicted_lanelets, predicted_areas_k, is_fallback);
            msg_send_time(vehicle_k) = toc(msg_send_tic);
        end
    end
    
    hlc.info.runtime_graph_search_each_veh = hlc.info.runtime_graph_search_each_veh + msg_send_time;
    % Calculate the total runtime of each group
    hlc.info = get_run_time_total_all_grps(hlc.info, hlc.scenario.parl_groups_info, CL_based_hierarchy, runtime_others);

    hlc.scenario.lanelet_crossing_areas = lanelet_crossing_areas;
end


