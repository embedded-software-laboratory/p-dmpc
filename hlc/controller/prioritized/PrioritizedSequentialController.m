classdef PrioritizedSequentialController < PrioritizedController

    methods

        function obj = PrioritizedSequentialController(options, plant, ros2_node)
            obj = obj@PrioritizedController(options, plant, ros2_node);
        end

    end

    methods (Access = protected)

        function controller(obj)
            % PB_CONTROLLER_PARL Plan trajectory for one time step using a
            % prioritized controller. Vehicles inside one group plan in sequence and
            % between groups plan in parallel. Controller simulates multiple
            % distributed controllers in a for-loop.

            % initialize variable to store control results
            obj.info = ControlResultsInfo( ...
                obj.options.amount, ...
                obj.options.Hp, ...
                obj.plant.all_vehicle_ids ...
            );

            level_matrix = kahn(obj.iter.directed_coupling_sequential);

            for i_level = 1:size(level_matrix, 1)

                vehicles_in_level = find(level_matrix(i_level, :));

                for i_vehicle = vehicles_in_level
                    % plan for i_vehicle
                    obj.timing_per_vehicle(i_vehicle).start('plan_single_vehicle', obj.k);
                    obj.plan_single_vehicle(i_vehicle);
                    obj.timing_per_vehicle(i_vehicle).stop('plan_single_vehicle', obj.k);
                    obj.timing_per_vehicle(i_vehicle).start('publish_predictions', obj.k);
                    obj.publish_predictions(i_vehicle);
                    obj.timing_per_vehicle(i_vehicle).stop('publish_predictions', obj.k);
                end

            end

        end

    end

end
