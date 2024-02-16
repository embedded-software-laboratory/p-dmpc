classdef PrioritizedExplorativeSequentialController < PrioritizedSequentialController

    methods

        function obj = PrioritizedExplorativeSequentialController()
        end

    end

    methods (Access = protected)

        function controller(obj)
            % CONTROLLER Plan trajectory for one time step using a
            % prioritized controller. Vehicles inside one group plan in sequence and
            % between groups plan in parallel. Controller simulates multiple
            % distributed controllers in a for-loop.

            for hlc = obj.hlcs
                hlc.prepare_iterations();
            end

            n_computation_levels = obj.hlcs(1).get_n_computation_levels();

            for hlc = obj.hlcs
                % guarantee the number of levels is consistent among the hlcs
                assert(n_computation_levels == hlc.get_n_computation_levels());
            end

            for desired_computation_level = 1:n_computation_levels

                for hlc = obj.hlcs
                    % calculate priority information for permutation
                    % where vehicle is on the desired_computation_level
                    hlc.prepare_permutation(desired_computation_level);
                    hlc.solve_permutation();
                end

            end

            for hlc = obj.hlcs
                hlc.compute_solution_cost();
                hlc.send_solution_cost();
            end

            for hlc = obj.hlcs
                hlc.receive_solution_cost();
                hlc.choose_solution();
            end

        end

    end

end
