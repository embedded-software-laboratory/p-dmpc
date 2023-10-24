classdef PrioritizedSequentialController < PrioritizedController

    methods

        function obj = PrioritizedSequentialController(scenario, plant)
            obj = obj@PrioritizedController(scenario, plant);
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
                obj.scenario.options.amount, ...
                obj.scenario.options.Hp, ...
                obj.plant.all_vehicle_ids ...
            );

            msg_send_time = zeros(1, obj.plant.amount);
            runtime_planning = zeros(1, obj.plant.amount);

            for level_j = 1:length(obj.CL_based_hierarchy)
                vehs_level_i = obj.CL_based_hierarchy(level_j).members; % vehicles of all groups in the same computation level

                for vehicle_idx = vehs_level_i
                    % plan for vehicle_idx
                    planning_timer = tic;
                    obj.plan_single_vehicle(vehicle_idx);
                    runtime_planning(vehicle_idx) = toc(planning_timer);

                    % communicate data to other vehicles
                    msg_send_tic = tic;
                    obj.publish_predictions(vehicle_idx);
                    msg_send_time(vehicle_idx) = toc(msg_send_tic);

                    % pause that MATLAB can execute the callback function of
                    % the sent prediction message
                    pause(1e-2)
                end

            end

            % Calculate the total runtime of each group
            obj.info = get_run_time_total_all_grps(obj.info, ...
                obj.iter.parl_groups_info, obj.CL_based_hierarchy, ...
                msg_send_time, runtime_planning);

        end

    end

end
