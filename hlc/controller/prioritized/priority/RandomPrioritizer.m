classdef RandomPrioritizer < Prioritizer
    % RANDOMPRIORITIZER  Instance of interface_priority used for dynamic priority
    % assignment, randomly assign priority to vehicles

    properties (Access = private)
    end

    methods

        function obj = RandomPrioritizer()
            obj.is_assign_unique_priority = false; % whether to asign unique priority
        end

        function [directed_coupling] = prioritize(~, time_step, ~, iter)
            adjacency = iter.adjacency;

            priority_rand_stream = RandStream("mt19937ar", "Seed", time_step);
            directed_coupling = adjacency;
            nVeh = size(adjacency, 1);
            current_priorities = randperm(priority_rand_stream, nVeh, nVeh);

            for iVeh = 1:nVeh

                for jVeh = 1:nVeh

                    if directed_coupling(iVeh, jVeh) && (current_priorities(iVeh) > current_priorities(jVeh))
                        directed_coupling(iVeh, jVeh) = 0;
                    end

                end

            end

        end

    end

end
