classdef FcaPrioritizer < Prioritizer
    % FCAPRIORITIZER  Instance of interface_priority used for dynamic priority
    % assignment. Vehicles with more potential collisions on their future
    % reference trajectory are assigned higher priorities.

    methods

        function obj = FcaPrioritizer()
        end

        function [directed_coupling] = prioritize(~, scenario, iter)
            adjacency = iter.adjacency;

            %% assign priorities to vehicles based on future collision assessment

            nVeh = size(adjacency, 1);
            Hp = size(iter.reference_trajectory_points, 2);
            collisions = zeros(1, nVeh);

            veh = Vehicle();
            x_locals = [-1, -1, 1, 1] * (veh.Length / 2 + scenario.options.offset);
            y_locals = [-1, 1, 1, -1] * (veh.Width / 2 + scenario.options.offset);

            for nveh = 1:nVeh - 1
                % position of nveh
                nveh_x = iter.reference_trajectory_points(nveh, :, 1);
                nveh_y = iter.reference_trajectory_points(nveh, :, 2);
                refPath_n = [nveh_x; nveh_y]';
                nveh_yaw = calculate_yaw(refPath_n);
                % check adjacent vehicles
                veh_adjacent = find(adjacency(nveh, :));

                %check the vehicles whose index is larger than the current vehicle, no repeated check
                veh_adjacent = veh_adjacent(veh_adjacent > nveh);

                for istep = 1:Hp
                    % shape of nveh
                    [x_globals_n, y_globals_n] = translate_global(nveh_yaw(istep), nveh_x(istep), nveh_y(istep), x_locals, y_locals);
                    shape_n = [x_globals_n; y_globals_n];

                    % check collistion between vehicles and static obstacles
                    if ~isempty(iter.obstacles)

                        for i = 1:numel(iter.obstacles)

                            if intersect_sat(shape_n, iter.obstacles{i})
                                collisions(nveh) = collisions(nveh) + 1;
                            end

                        end

                    end

                    % check collistion between vehicles and dynamic obstacles
                    if ~isempty(iter.dynamic_obstacle_area)

                        for i = 1:size(iter.dynamic_obstacle_area, 1)

                            if intersect_sat(shape_n, iter.dynamic_obstacle_area{i, istep})
                                collisions(nveh) = collisions(nveh) + 1;
                            end

                        end

                    end

                    % check collistion between two vehicles
                    for iveh = veh_adjacent
                        % position of iveh
                        iveh_x = iter.reference_trajectory_points(iveh, :, 1);
                        iveh_y = iter.reference_trajectory_points(iveh, :, 2);
                        refPath_i = [iveh_x; iveh_y]';
                        iveh_yaw = calculate_yaw(refPath_i);

                        % shape of iveh
                        [x_globals_i, y_globals_i] = translate_global(iveh_yaw(istep), iveh_x(istep), iveh_y(istep), x_locals, y_locals);
                        shape_i = [x_globals_i; y_globals_i];

                        % check if there is collision between nveh and iveh
                        if intersect_sat(shape_n, shape_i)
                            collisions(nveh) = collisions(nveh) + 1;
                            collisions(iveh) = collisions(iveh) + 1;
                        end

                    end

                end

            end

            [~, FCAPrio] = sort(collisions, 'descend'); % ordered vehicle index w.r.t. priority
            %disp(['collisions: ',num2str(collisions)])
            %disp(['priority_index: ',num2str(FCAPrio)])

            directed_coupling = adjacency;

            for iVeh = 1:nVeh

                for jVeh = 1:nVeh

                    if directed_coupling(iVeh, jVeh) && (FCAPrio(iVeh) > FCAPrio(jVeh))
                        directed_coupling(iVeh, jVeh) = 0;
                    end

                end

            end

            [isDAG, ~] = kahn(directed_coupling);
            assert(isDAG, 'Coupling matrix is not a DAG');
        end

    end

end