classdef RandomPrioritizer < Prioritizer
    % RANDOMPRIORITIZER  Instance of interface_priority used for dynamic priority
    % assignment, randomly assign priority to vehicles

    properties (Access = private)
    end

    methods

        function obj = RandomPrioritizer()
            obj.is_assign_unique_priority = false; % whether to asign unique priority
        end

        function [directed_coupling] = prioritize(~, iter, time_step, ~, ~)
            adjacency = iter.adjacency;

            priority_rand_stream = RandStream("mt19937ar", "Seed", time_step);
            nVeh = size(adjacency, 1);
            current_priorities = randperm(priority_rand_stream, nVeh, nVeh);

            directed_coupling = Prioritizer.directed_coupling_from_priorities(adjacency, current_priorities);

        end

    end

end
