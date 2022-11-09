classdef coloring_priority < interface_priority

    properties (Access=private)
    end
    methods
        function obj = coloring_priority()
        end

        function [groups, coupling_directed, priority_list] = priority(obj,scenario)
            % TODO all in this function/no dependencies (?)
            % apply topological sorting algorithm with coloring
            [topo_valid, topo_matrix] = topological_coloring(scenario.adjacency(:,:,end));
            assert(topo_valid,'No valid topological coloring possible.');
            
            % determine the order to reduce incoming edges 
            order = order_topo(topo_matrix,scenario.adjacency(:,:,end));
            
            % reorder matrix according to new level order
            topo_matrix(:,:) = topo_matrix(order,:);

            groups = PB_predecessor_groups(topo_matrix);
    
            % Assign priority according to computation level
            priority_list = obj.get_priority(groups);
    
            coupling_directed = obj.direct_coupling(scenario, topo_matrix);
        end
    end
end
