classdef PrioritizedParallelController < PrioritizedController

    methods

        function obj = PrioritizedParallelController(scenario, plant)
            obj = obj@PrioritizedController(scenario, plant);
        end

    end

    methods (Access = protected)

        function controller(obj)
            % PB_CONTROLLER_PARL Plan trajectory for one time step using a
            % priority-based controller. Vehicles inside one group plan in sequence and
            % between groups plan in pararllel. Controller simulates multiple
            % distributed controllers in a for-loop.

            % initialize variable to store control results
            obj.info = ControlResultsInfo( ...
                obj.scenario.options.amount, ...
                obj.scenario.options.Hp, ...
                obj.plant.all_vehicle_ids ...
            );

            vehicle_idx = obj.plant.indices_in_vehicle_list(1);

            % plan for vehicle_idx
            obj.timing_per_vehicle(vehicle_idx).start('plan_single_vehicle', obj.k);
            obj.plan_single_vehicle(vehicle_idx);
            obj.timing_per_vehicle(vehicle_idx).stop('plan_single_vehicle', obj.k);

            %% Send own data to other vehicles
            msg_send_tic = tic;
            obj.publish_predictions(vehicle_idx);
            msg_send_time = toc(msg_send_tic);

            obj.info.runtime_graph_search_max = obj.info.runtime_graph_search_each_veh(vehicle_idx);
            obj.info.runtime_subcontroller_each_veh(vehicle_idx) = msg_send_time + runtime_planning;
            obj.info.runtime_subcontroller_each_veh(vehicle_idx) = obj.info.runtime_subcontroller_each_veh(vehicle_idx);
            obj.info.runtime_subcontroller_max = obj.info.runtime_subcontroller_each_veh(vehicle_idx);
            obj.info.computation_levels = length(obj.CL_based_hierarchy);
        end

    end

end
