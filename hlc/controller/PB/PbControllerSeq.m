classdef PbControllerSeq < PbControllerInterface
    methods
        function obj = PbControllerSeq()
            obj = obj@PbControllerInterface();
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
            msg_send_time = zeros(1,nVeh);

            for level_j = 1:length(CL_based_hierarchy)
                vehs_level_i = CL_based_hierarchy(level_j).members; % vehicles of all groups in the same computation level

                for vehicle_idx = vehs_level_i
                    if ismember(vehicle_idx, obj.info.vehs_fallback)
                        % jump to next vehicle if the selected vehicle should take fallback
                        obj.info.runtime_graph_search_each_veh(vehicle_idx) = 0;
                        continue
                    end
                    
                    % plan for vehicle_idx
                    obj.plan_single_vehicle(vehicle_idx);
                end

                % Communicate data to other vehicles
                for vehicle_k = vehs_level_i
                    if ismember(vehicle_k, obj.info.vehs_fallback)
                        % if the selected vehicle should take fallback
                        continue
                    end
                    % send message
                    msg_send_tic = tic;
                    predicted_areas_k = obj.info.shapes(vehicle_k,:);
                    obj.scenario.vehicles(vehicle_k).communicate.predictions.send_message(obj.k, predicted_areas_k, obj.info.vehs_fallback);
                    msg_send_time(vehicle_k) = toc(msg_send_tic);
                end
            end

            obj.info.runtime_graph_search_each_veh = obj.info.runtime_graph_search_each_veh + msg_send_time;
            % Calculate the total runtime of each group
            obj.info = get_run_time_total_all_grps(obj.info, obj.iter.parl_groups_info, CL_based_hierarchy, runtime_others);

            obj.scenario.lanelet_crossing_areas = lanelet_crossing_areas;
        end
    end
end
