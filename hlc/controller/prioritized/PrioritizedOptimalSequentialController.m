classdef PrioritizedOptimalSequentialController < PrioritizedSequentialController

    methods

        function obj = PrioritizedOptimalSequentialController()
        end

    end

    methods (Access = protected)

        function controller(obj)
            % CONTROLLER Plan trajectory for one time step using a
            % prioritized controller. Vehicles inside one group plan in sequence and
            % between groups plan in parallel. Controller simulates multiple
            % distributed controllers in a for-loop.

            unique_priorities = Prioritizer.unique_priorities( ...
                obj.merged_graph("adjacency") ...
            );
            n_priorities = size(unique_priorities, 2);

            for hlc = obj.hlcs
                hlc.prepare_iterations();
            end

            for priority_permutation = 1:n_priorities

                for hlc = obj.hlcs
                    hlc.prepare_permutation( ...
                        unique_priorities(:, priority_permutation), ...
                        priority_permutation ...
                    );
                end

                level_matrix = kahn(obj.merged_graph("directed_coupling_sequential"));

                for i_level = 1:size(level_matrix, 1)
                    vehicles_in_level = find(level_matrix(i_level, :));

                    for i_vehicle = vehicles_in_level
                        obj.hlcs(i_vehicle).solve_permutation();
                    end

                end

            end

        end

        function store_control_info(obj)

            for hlc = obj.hlcs
                hlc.compute_solution_cost();
                hlc.send_solution_cost();
            end

            for hlc = obj.hlcs
                hlc.receive_solution_cost();
                hlc.choose_solution();
                store_control_info@HighLevelController(hlc);
            end

        end

    end

end
