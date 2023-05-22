classdef RandomPrioritizer < Prioritizer
    % RANDOMPRIORITIZER  Instance of interface_priority used for dynamic priority
    % assignment, randomly assign priority to vehicles

    properties (Access = private)
    end

    methods

        function obj = RandomPrioritizer()
            obj.is_assign_unique_priority = false; % whether to asign unique priority
        end

        function [directed_coupling] = prioritize(~, ~, iter)
            adjacency = iter.adjacency;

            priority_rand_stream = RandStream("mt19937ar", "Seed", iter.k);
            directed_coupling = adjacency;
            nVeh = size(adjacency, 1);
            RandPrio = randperm(priority_rand_stream, nVeh, nVeh);

            for iVeh = 1:nVeh

                for jVeh = 1:nVeh

                    if directed_coupling(iVeh, jVeh) && (RandPrio(iVeh) > RandPrio(jVeh))
                        directed_coupling(iVeh, jVeh) = 0;
                    end

                end

            end

            [isDAG, ~] = kahn(directed_coupling);
            assert(isDAG, 'Coupling matrix is not a DAG');
        end

    end

end
