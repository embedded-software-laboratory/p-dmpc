classdef PrioritizedSequentialController < PrioritizedController

    methods

        function obj = PrioritizedSequentialController(options, plant)
            obj = obj@PrioritizedController(options, plant);
        end

    end

    methods (Access = protected)

        function controller(obj)
            % PB_CONTROLLER_PARL Plan trajectory for one time step using a
            % priority-based controller. Vehicles inside one group plan in sequence and
            % between groups plan in parallel. Controller simulates multiple
            % distributed controllers in a for-loop.

            % initialize variable to store control results
            obj.info = ControlResultsInfo( ...
                obj.options.amount, ...
                obj.options.Hp, ...
                obj.plant.all_vehicle_ids ...
            );

            for level_j = 1:length(obj.CL_based_hierarchy)
                vehs_level_i = obj.CL_based_hierarchy(level_j).members; % vehicles of all groups in the same computation level

                for vehicle_idx = vehs_level_i
                    % plan for vehicle_idx
                    obj.timing_per_vehicle(vehicle_idx).start('plan_single_vehicle', obj.k);
                    obj.plan_single_vehicle(vehicle_idx);
                    obj.timing_per_vehicle(vehicle_idx).stop('plan_single_vehicle', obj.k);
                    obj.timing_per_vehicle(vehicle_idx).start('publish_predictions', obj.k);
                    obj.publish_predictions(vehicle_idx);
                    obj.timing_per_vehicle(vehicle_idx).stop('publish_predictions', obj.k);
                end

            end

        end

    end

end
