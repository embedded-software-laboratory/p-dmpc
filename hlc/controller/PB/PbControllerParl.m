
classdef PbControllerParl < HLCInterface
    methods
        function obj = PbControllerParl()
            obj = obj@HLCInterface();
        end
    end
    methods (Access = protected)
        function controller(obj)
            % PB_CONTROLLER_PARL Plan trajectory for one time step using a
            % priority-based controller. Vehicles inside one group plan in sequence and
            % between groups plan in pararllel. Controller simulates multiple
            % distributed controllers in a for-loop.

            runtime_others_tic = tic;

            assign_priority_timer = tic;
            [obj.scenario,obj.iter,CL_based_hierarchy,lanelet_crossing_areas] = priority_assignment_parl(obj.scenario, obj.iter);
            obj.iter.timer.assign_priority = toc(assign_priority_timer);

            nVeh = obj.scenario.options.amount;
            Hp = obj.scenario.options.Hp;

            % initialize variable to store control results
            obj.info = ControllResultsInfo(nVeh, Hp, [obj.scenario.vehicles.ID]);
            n_expended = zeros(nVeh,1);

            directed_graph = digraph(obj.iter.directed_coupling);
            [obj.belonging_vector_total,~] = conncomp(directed_graph,'Type','weak'); % graph decomposition

            obj.iter.num_couplings_between_grps = 0; % number of couplings between groups
            obj.iter.num_couplings_between_grps_ignored = 0; % ignored number of couplings between groups by using lanelet crossing lanelets
            for iCoupling = 1:length([obj.iter.coupling_info.veh_with_ROW])
                veh_ij = [obj.iter.coupling_info(iCoupling).veh_with_ROW,obj.iter.coupling_info(iCoupling).veh_without_ROW];
                is_same_grp = any(cellfun(@(c) all(ismember(veh_ij,c)),{obj.iter.parl_groups_info.vertices}));
                if ~is_same_grp
                    obj.iter.num_couplings_between_grps = obj.iter.num_couplings_between_grps + 1;
                    if obj.iter.coupling_info(iCoupling).is_ignored
                        obj.iter.num_couplings_between_grps_ignored = obj.iter.num_couplings_between_grps_ignored + 1;
                    end
                end
            end

            runtime_others = toc(runtime_others_tic); % subcontroller runtime except for runtime of graph search
            msg_send_time = 0;

            vehicle_idx = obj.indices_in_vehicle_list(1);

            subcontroller_timer = tic;

            % only keep self
            filter_self = false(1,obj.scenario.options.amount);
            filter_self(vehicle_idx) = true;
            iter_v = filter_iter(obj.iter, filter_self);

            grp_idx = arrayfun(@(array) ismember(vehicle_idx,array.vertices), iter_v.parl_groups_info);
            all_vehs_same_grp = iter_v.parl_groups_info(grp_idx).vertices; % all vehicles in the same group

            all_coupled_vehs_with_HP = find(iter_v.directed_coupling_reduced(:,vehicle_idx)==1)'; % all coupled vehicles with higher priorities
            all_coupled_vehs_with_LP = find(iter_v.directed_coupling_reduced(vehicle_idx,:)==1); % all coupled vehicles with lower priorities

            coupled_vehs_same_grp_with_HP = intersect(all_coupled_vehs_with_HP, all_vehs_same_grp); % coupled vehicles with higher priorities in the same group
            coupled_vehs_other_grps_with_HP = setdiff(all_coupled_vehs_with_HP, coupled_vehs_same_grp_with_HP); % coupled vehicles with higher priorities in other groups

            for veh_with_HP_i = all_coupled_vehs_with_HP

                if ismember(veh_with_HP_i,coupled_vehs_same_grp_with_HP)
                    % if in the same group, read the current message and set the predicted occupied areas as dynamic obstacles
                    latest_msg = read_message(obj.scenario.vehicles(vehicle_idx).communicate.predictions, obj.ros_subscribers.predictions{veh_with_HP_i}, obj.k);
%                     obj.info.vehs_fallback = union(obj.info.vehs_fallback, latest_msg.vehs_fallback);
%                     if ismember(vehicle_k, obj.info.vehs_fallback)
%                         % if the selected vehicle should take fallback
%                         break
%                     end
                    predicted_areas_i = arrayfun(@(array) {[array.x(:)';array.y(:)']}, latest_msg.predicted_areas);
                    oldness_msg = obj.k - latest_msg.time_step;
                    if oldness_msg ~= 0
                        % consider the oldness of the message: delete the first n entries and repeat the last entry for n times
                        predicted_areas_i = del_first_rpt_last(predicted_areas_i,oldness_msg);
                    end
                    iter_v.dynamic_obstacle_area(end+1,:) = predicted_areas_i;

                else
                    % if they are in different groups
                    if obj.scenario.options.isDealPredictionInconsistency
                        % Collisions with coupled vehicles with higher priorities in different groups will be avoided by two ways depending on the time step at which
                        % their latest messages are sent:
                        % 1. Their predicted occupied areas will be considered as dynamic obstacles if the latest messages come from the current time step.
                        % 2. Their reachable sets will be considered as dynamic obstacles if the latest messages come from past time step.
                        latest_msg = obj.ros_subscribers.predictions{veh_with_HP_i}.LatestMessage;
                        if latest_msg.time_step == obj.k
                            predicted_areas_i = arrayfun(@(array) {[array.x(:)';array.y(:)']}, latest_msg.predicted_areas);
                            iter_v.dynamic_obstacle_area(end+1,:) = predicted_areas_i;
                        else
                            % Add their reachable sets as dynamic obstacles to deal with the prediction inconsistency
                            reachable_sets_i = obj.iter.reachable_sets{veh_with_HP_i,:};
                            % turn polyshape to plain array (repeat the first row to enclosed the shape)
                            reachable_sets_i_array = cellfun(@(c) {[c.Vertices(:,1)',c.Vertices(1,1)';c.Vertices(:,2)',c.Vertices(1,2)']}, reachable_sets_i);
                            iter_v.dynamic_obstacle_reachableSets(end+1,:) = reachable_sets_i_array;
                        end
                    else
                        % otherwise add one-step delayed trajectories as dynamic obstacles
                        if obj.k>1
                            % the old trajectories are available from the second time step onwards
                            old_msg = read_message(obj.scenario.vehicles(vehicle_idx).communicate.predictions, obj.ros_subscribers.predictions{veh_with_HP_i}, obj.k-1);
                            predicted_areas_i = arrayfun(@(array) {[array.x(:)';array.y(:)']}, old_msg.predicted_areas);
                            oldness_msg = obj.k - old_msg.time_step;
                            if oldness_msg ~= 0
                                % consider the oldness of the message: delete the first n entries and repeat the last entry for n times
                                predicted_areas_i = del_first_rpt_last(predicted_areas_i',oldness_msg);
                            end
                            iter_v.dynamic_obstacle_area(end+1,:) = predicted_areas_i;
                        end
                    end

                end
            end

            if ~strcmp(obj.scenario.options.strategy_enter_lanelet_crossing_area,'1')
                % Set lanelet intersecting areas as static obstacles if vehicle with lower priorities is not allowed to enter those area
                iter_v.lanelet_crossing_areas = lanelet_crossing_areas{vehicle_idx};
                if isempty(iter_v.lanelet_crossing_areas)
                    iter_v.lanelet_crossing_areas = {}; % convert from 'double' to 'cell'
                end
                assert(iscell(iter_v.lanelet_crossing_areas));
            end

            % consider coupled vehicles with lower priorities
            iter_v = consider_vehs_with_LP(obj.scenario, iter_v, vehicle_idx, all_coupled_vehs_with_LP, obj.ros_subscribers);

            %% Plan for vehicle vehicle_idx
            % execute sub controller for 1-veh scenario
            info_v = obj.sub_controller(obj.scenario, iter_v);
            if info_v.is_exhausted
                % if graph search is exhausted, this vehicles and all its weakly coupled vehicles will use their fallback trajectories
                %                 disp(['Graph search exhausted after expending node ' num2str(info_v.n_expanded) ' times for vehicle ' num2str(vehicle_idx) ', at time step: ' num2str(scenario.k) '.'])
                switch obj.scenario.options.fallback_type
                    case 'localFallback'
                        sub_graph_fallback = obj.belonging_vector_total(vehicle_idx);
                        obj.info.vehs_fallback = [obj.info.vehs_fallback, find(obj.belonging_vector_total==sub_graph_fallback)];
                        obj.info.vehs_fallback = unique(obj.info.vehs_fallback,'stable');
                    case 'globalFallback'
                        % global fallback: all vehicles take fallback
                        obj.info.vehs_fallback = 1:obj.scenario.options.amount;
                    case 'noFallback'
                        % Fallback is disabled. Simulation will end.
                        obj.info.vehs_fallback = 1:obj.scenario.options.amount;
                    otherwise
                        warning("Please define one of the follows as fallback strategy: 'noFallback', 'localFallback', and 'globalFallback'.")
                end
                obj.info.is_exhausted(vehicle_idx) = true;
            else
                obj.info = store_control_info(obj.info, info_v, obj.scenario);
            end
            obj.info.runtime_graph_search_each_veh(vehicle_idx) = toc(subcontroller_timer);
            n_expended(vehicle_idx) = info_v.tree.size();

            if obj.iter.k==inf
                plot_obstacles(obj.scenario)
                plot_obstacles(info_v.shapes)
                graphs_visualization(obj.iter.belonging_vector, obj.scenario.coupling_weights, 'ShowWeights', true)
            end
            %% Send own data to other vehicles

            if ~ismember(vehicle_idx, obj.info.vehs_fallback)
                % if the selected vehicle should take fallback

                msg_send_tic = tic;
                predicted_areas_k = obj.info.shapes(vehicle_idx,:);

                % send message
                obj.scenario.vehicles(vehicle_idx).communicate.predictions.send_message(obj.k, predicted_areas_k, obj.info.vehs_fallback);
                msg_send_time = toc(msg_send_tic);

            else
                msg_send_tic = tic;
                obj.scenario.vehicles(vehicle_idx).communicate.predictions.send_message(obj.k, {}, obj.info.vehs_fallback);
                msg_send_time = toc(msg_send_tic);
            end

            obj.info.runtime_graph_search_each_veh(vehicle_idx) = obj.info.runtime_graph_search_each_veh(vehicle_idx) + msg_send_time;
            obj.info.runtime_graph_search_max = obj.info.runtime_graph_search_each_veh(vehicle_idx);
            obj.info.runtime_subcontroller_each_veh(vehicle_idx) = obj.info.runtime_graph_search_each_veh(vehicle_idx) + runtime_others;
            obj.info.runtime_subcontroller_max = obj.info.runtime_graph_search_max + runtime_others;
            obj.info.computation_levels = length(CL_based_hierarchy);
            obj.scenario.lanelet_crossing_areas = lanelet_crossing_areas;
        end
    end
end
