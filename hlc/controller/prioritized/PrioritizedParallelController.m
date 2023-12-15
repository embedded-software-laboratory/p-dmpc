classdef PrioritizedParallelController < PrioritizedController

    methods

        function obj = PrioritizedParallelController(options, plant, ros2_node)
            obj = obj@PrioritizedController(options, plant, ros2_node);
        end

    end

    methods (Access = protected)

        function controller(obj)
            % PB_CONTROLLER_PARL Plan trajectory for one time step using a
            % prioritized controller. Vehicles inside one group plan in sequence and
            % between groups plan in pararllel. Controller simulates multiple
            % distributed controllers in a for-loop.

            % initialize variable to store control results
            obj.info = ControlResultsInfo( ...
                obj.options.amount, ...
                obj.options.Hp, ...
                obj.plant.all_vehicle_indices ...
            );

            vehicle_idx = obj.plant.vehicle_indices_controlled(1);

            % plan for vehicle_idx
            obj.timing_per_vehicle(vehicle_idx).start('plan_single_vehicle', obj.k);
            obj.plan_single_vehicle(vehicle_idx);
            obj.timing_per_vehicle(vehicle_idx).stop('plan_single_vehicle', obj.k);

            %% Send own data to other vehicles
            obj.timing_per_vehicle(vehicle_idx).start('publish_predictions', obj.k);
            obj.publish_predictions(vehicle_idx);
            obj.timing_per_vehicle(vehicle_idx).stop('publish_predictions', obj.k);
        end

    end

end
