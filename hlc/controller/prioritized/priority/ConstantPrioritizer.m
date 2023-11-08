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
            nVeh = size(adjacency, 1);

            if iter.priority_permutation == 0
                % standard permutation
                current_priorities = 1:nVeh;
            else
                % specific permutation
                current_priorities = obj.all_priorities(:, iter.priority_permutation);
            end

            directed_coupling = Prioritizer.directed_coupling_from_priorities(adjacency, current_priorities);

        end

        function compute_unique_priorities(obj, adjacency)
            obj.all_priorities = obj.unique_priorities(adjacency);
            obj.n_priorities = size(obj.all_priorities, 2);
        end

        function priorities = unique_priorities(~, adjacency)
            n_vehicles = size(adjacency, 1);
            priorities = zeros(n_vehicles, 0);
            directed_coupling_base = triu(adjacency, 1);
            dag_coupling_base = digraph(directed_coupling_base);
            n_edges = numedges(dag_coupling_base);
            n_permutations = 2^n_edges;

            for i_permutation = 1:n_permutations
                dag_coupling = dag_coupling_base;
                flips = dec2bin(i_permutation - 1, n_edges) == '1';

                dag_coupling = flipedge(dag_coupling, find(flips));

                if isdag(dag_coupling)
                    topological_order = toposort(dag_coupling);
                    priority = zeros(1, n_vehicles);
                    priority(topological_order) = 1:n_vehicles;
                    priorities(:, end + 1) = priority; %#ok<AGROW>
                end

            end

        end

    end

end
