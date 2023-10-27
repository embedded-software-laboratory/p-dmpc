classdef ConstantPrioritizer < Prioritizer
    % constant_priority  Instance of interface_priority used for priority
    % assignment, fixed priority according to vehicle ids

    properties (Access = private)
    end

    methods

        function obj = ConstantPrioritizer()
        end

        function [directed_coupling] = prioritize(~, ~, ~, iter)
            adjacency = iter.adjacency;

            directed_coupling = adjacency;
            nVeh = size(adjacency, 1);

            if iter.priority_permutation == 0
                % standard permutation
                current_priorities = 1:nVeh;
            else
                % specific permutation
                all_priorities = perms(1:nVeh);
                current_priorities = all_priorities(iter.priority_permutation, :);
            end

            for iVeh = 1:nVeh

                for jVeh = 1:nVeh

                    if directed_coupling(iVeh, jVeh) && (current_priorities(iVeh) > current_priorities(jVeh))
                        directed_coupling(iVeh, jVeh) = 0;
                    end

                end

            end

            [isDAG, ~] = kahn(directed_coupling);
            assert(isDAG, 'Coupling matrix is not a DAG');
        end

    end

end
