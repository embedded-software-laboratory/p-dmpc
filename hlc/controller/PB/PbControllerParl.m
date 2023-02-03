classdef PbControllerParl < PbControllerInterface
    methods
        function obj = PbControllerParl()
            obj = obj@PbControllerInterface();
        end
    end
    methods (Access = protected)
        function controller(obj)
            % PB_CONTROLLER_PARL Plan trajectory for one time step using a
            % priority-based controller. Vehicles inside one group plan in sequence and
            % between groups plan in pararllel. Controller simulates multiple
            % distributed controllers in a for-loop.

            vehicle_idx = obj.indices_in_vehicle_list(1);

            % plan for vehicle_idx
            obj.plan_single_vehicle(vehicle_idx);

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
