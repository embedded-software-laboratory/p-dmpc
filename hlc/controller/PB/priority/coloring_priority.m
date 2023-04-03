classdef coloring_priority < interface_priority

    properties (Access = private)
    end

    methods

        function obj = coloring_priority()
        end

        function [level, coupling_directed, priority_list] = priority(obj, iter)
            % TODO all in this function/no dependencies (?)
            % apply topological sorting algorithm with coloring
            [topo_valid, topo_matrix] = topological_coloring(iter.adjacency);
            assert(topo_valid, 'No valid topological coloring possible.');

            % determine the order to reduce incoming edges
            order = order_topo(topo_matrix, iter.adjacency);

            % reorder matrix according to new level order
            topo_matrix(:, :) = topo_matrix(order, :);

            level = computation_level_members(topo_matrix);

            % Assign priority according to computation level
            coupling_directed = obj.direct_coupling(iter.adjacency, topo_matrix);
            priority_list = obj.get_priority(coupling_directed);
        end

    end

end
