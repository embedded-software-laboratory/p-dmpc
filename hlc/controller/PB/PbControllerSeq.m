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

            runtime_others = obj.init_step();

            msg_send_time = zeros(1,obj.amount);
            runtime_planning = zeros(1, obj.amount);

            for level_j = 1:length(obj.CL_based_hierarchy)
                vehs_level_i = obj.CL_based_hierarchy(level_j).members; % vehicles of all groups in the same computation level

                for vehicle_idx = vehs_level_i
                    if ismember(vehicle_idx, obj.info.vehs_fallback)
                        % jump to next vehicle if the selected vehicle should take fallback
                        obj.info.runtime_graph_search_each_veh(vehicle_idx) = 0;
                        continue
                    end
                    
                    % plan for vehicle_idx
                    runtime_planning(vehicle_idx) = obj.plan_single_vehicle(vehicle_idx);
                end

                % Communicate data to other vehicles
                for vehicle_k = vehs_level_i
                    msg_send_time(vehicle_k) = obj.publish_predicitons(vehicle_k);
                end
            end

            % Calculate the total runtime of each group
            obj.info = get_run_time_total_all_grps(obj.info, ...
                obj.iter.parl_groups_info, obj.CL_based_hierarchy, ...
                msg_send_time, runtime_others, runtime_planning);

            obj.iter.lanelet_crossing_areas = obj.lanelet_crossing_areas;
        end
    end
end
