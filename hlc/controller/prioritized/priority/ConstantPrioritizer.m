classdef ConstantPrioritizer < Prioritizer
    % constant_priority  Instance of interface_priority used for priority
    % assignment, fixed priority according to vehicle ids

    properties (GetAccess = public, SetAccess = private)
        all_priorities % n_vehicles x n_unique_priority_permutations
        n_priorities
    end

    methods

        function obj = ConstantPrioritizer()
        end

        function [directed_coupling] = prioritize(obj, iter, ~, ~, ~)
            adjacency = iter.adjacency;

            directed_coupling = adjacency;
            nVeh = size(adjacency, 1);

            if iter.priority_permutation == 0
                % standard permutation
                current_priorities = 1:nVeh;
            else
                % specific permutation
                current_priorities = obj.all_priorities(:, iter.priority_permutation);
            end

            for iVeh = 1:nVeh

                for jVeh = 1:nVeh

                    if directed_coupling(iVeh, jVeh) && (current_priorities(iVeh) > current_priorities(jVeh))
                        directed_coupling(iVeh, jVeh) = 0;
                    end

                end

            end

        end

        function compute_unique_priorities(obj, adjacency)
            obj.all_priorities = obj.unique_priorities(adjacency);
            obj.n_priorities = size(obj.all_priorities, 2);
        end

        function priorities = unique_priorities(~, adjacency)
            n_vehicles = size(adjacency, 1);
            priorities = zeros(n_vehicles, factorial(n_vehicles));
            directed_coupling_base = triu(adjacency, 1);
            dag_coupling_base = digraph(directed_coupling_base);
            n_edges = numedges(dag_coupling_base);
            n_permutations = 2^n_edges;
            i_priority = 1;

            for i_permutation = 1:n_permutations
                dag_coupling = dag_coupling_base;
                flips = dec2bin(i_permutation - 1, n_edges) == '1';

                for i_flip = find(flips)
                    dag_coupling = flipedge(dag_coupling, i_flip);
                end

                if isdag(dag_coupling)
                    priorities(:, i_priority) = toposort(dag_coupling);
                    i_priority = i_priority + 1;
                else
                    priorities(:, end) = [];
                end

            end

        end

    end

end
