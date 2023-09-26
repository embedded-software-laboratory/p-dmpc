classdef OptimalPrioritizer < Prioritizer
    % constant_priority  Instance of interface_priority used for priority
    % assignment, fixed priority according to vehicle ids

    properties (Access = private)
    end

    methods

        function obj = OptimalPrioritizer()
        end

        function [directed_coupling] = prioritize(~, ~, iter)
            adjacency = iter.adjacency;

            directed_coupling = adjacency;
            nVeh = size(adjacency, 1);
            all_prios = perms(1:nVeh);
            OptimalPrio = all_prios(iter.permutation, :);

            for iVeh = 1:nVeh

                for jVeh = 1:nVeh

                    if directed_coupling(iVeh, jVeh) && (OptimalPrio(iVeh) > OptimalPrio(jVeh))
                        directed_coupling(iVeh, jVeh) = 0;
                    end

                end

            end

            [isDAG, ~] = kahn(directed_coupling);
            assert(isDAG, 'Coupling matrix is not a DAG');
        end

    end

end
