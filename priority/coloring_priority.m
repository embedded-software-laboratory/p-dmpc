classdef coloring_priority < interface_priority
   methods(Static)
        function [groups, coupling_directed, priority_list] = priority(scenario)
            % TODO put `topological_sorting_coloring` function here, remove dependencies from other functions
            [isDAG, topo_groups] = topological_sorting_coloring(scenario.adjacency(:,:,end));

            assert( isDAG, 'Coupling matrix is not a DAG' );
            groups = PB_predecessor_groups(topo_groups);

            % Assign priority according to computation level
            priority_list = get_priority(groups);

            coupling_directed = direct_coupling(scenario, topo_groups);
        end
    end
end
