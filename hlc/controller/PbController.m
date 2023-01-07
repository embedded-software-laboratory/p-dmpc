classdef PbController < HLCInterface
    methods
        function obj = PbController()
            obj = obj@HLCInterface();
        end
    end
    methods (Access = protected)
        function controller(obj)
            % PB_CONTROLLER    Plan trajectory for one time step using a priority-based controller.
            %     Controller simulates multiple distributed controllers.

            runtime_others_tic = tic;
            [veh_at_intersection, groups, directed_adjacency, priority_list] = priority_assignment(obj.scenario,obj.iter);

            % visualize the coupling between vehicles
            %     plot_coupling_lines(directed_adjacency, iter)

            % construct the priority list
            computation_levels = length(groups);

            nVeh = obj.scenario.options.amount;
            Hp = obj.scenario.options.Hp;

            % update properties of scenario
            obj.iter.directed_coupling = directed_adjacency;
            obj.iter = priority_list;
            obj.iter = veh_at_intersection;

            % initialize variable to store control results
            obj.info = ControllResultsInfo(nVeh, Hp, [obj.scenario.vehicles.ID]);

            directed_graph = digraph(directed_adjacency);
            [belonging_vector_total,~] = conncomp(directed_graph,'Type','weak'); % graph decomposition
            runtime_others = toc(runtime_others_tic); % subcontroller runtime except for runtime of graph search


            for grp_idx = 1:length(groups)
                group = groups(grp_idx);
                for grp_member_idx = 1:length(group.members)
                    subcontroller_timer = tic;

                    vehicle_idx = group.members(grp_member_idx);
                    if ismember(vehicle_idx, obj.info.vehs_fallback)
                        % if the selected vehicle should take fallback
                        obj.info.runtime_graph_search_each_veh(vehicle_idx) = 0;
                        continue
                    end

                    % Filter out vehicles that are not adjacent
                    veh_adjacent = find(obj.iter.adjacency(vehicle_idx,:,end));
                    veh_adjacent = setdiff(veh_adjacent,vehicle_idx); % exclude self
                    predecessors = intersect(group.predecessors,veh_adjacent);

                    % Filter out vehicles with lower or same priority.
                    priority_filter = false(1,obj.scenario.options.amount);
                    priority_filter(predecessors) = true; % keep all with higher priority
                    priority_filter(vehicle_idx) = true; % keep self
                    iter_filtered = filter_iter(obj.iter, priority_filter);

                    self_index = sum(priority_filter(1:vehicle_idx));
                    v2o_filter = true(1,iter_filtered.amount);
                    v2o_filter(self_index) = false;

                    % add predicted trajecotries of vehicles with higher priority as dynamic obstacle
                    [iter_v] = vehicles_as_dynamic_obstacles(iter_filtered, v2o_filter, obj.info.shapes(predecessors,:));

                    % add adjacent vehicles with lower priorities as static obstacles
                    adjacent_vehicle_lower_priority = setdiff(veh_adjacent,predecessors);
                    % only two strategies are supported if parallel computation is not used
                    assert(any(strcmp(scenario.options.strategy_consider_veh_without_ROW,{'1','2','3'})))
                    obj.iter = consider_vehs_with_LP(scenario_v, obj.iter, vehicle_idx, adjacent_vehicle_lower_priority);

                    % execute sub controller for 1-veh scenario
                    info_v = obj.sub_controller(obj.scenario, iter_v);

                    if info_v.is_exhausted
                        % if graph search is exhausted, this vehicles and all vehicles that have directed or
                        % undirected couplings with this vehicle will take fallback
                        disp(['Graph search exhausted for vehicle ' num2str(vehicle_idx) ', at time step: ' num2str(obj.iter.k) '.'])
                        sub_graph_fallback = belonging_vector_total(vehicle_idx);
                        obj.info.vehs_fallback = [obj.info.vehs_fallback, find(belonging_vector_total==sub_graph_fallback)];
                        obj.info.vehs_fallback = unique(obj.info.vehs_fallback,'stable');
                        obj.info.is_exhausted(vehicle_idx) = true;
                    else
                        obj.info = store_control_obj.info(obj.info, info_v, obj.scenario);
                    end
                    obj.info.runtime_graph_search_each_veh(vehicle_idx) = toc(subcontroller_timer);
                end

            end

            % calculate the total runtime: only one vehicle in each computation level will be counted, this is the one with the maximum runtime
            parl_groups_info = struct('vertices',1:nVeh,'num_CLs',computation_levels,'path_info',[]); % one group
            obj.info = get_run_time_total_all_grps(obj.info, parl_groups_info, groups, runtime_others);
        end
    end
end
