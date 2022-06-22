classdef  topo_priority < interface_priority
% topo_priority  Instance of interface_priority used for priority assignment.
    
    properties (Access=private)
        isDAG
        topo_groups
        
    end
    
    methods 
        function obj = topo_priority()
            obj.is_assign_unique_priority = false; % whether to asign unique priority            
        end
        
        function [groups, coupling_directed, priority_list] = priority(obj,scenario)
            
            if scenario.assignPrios || isempty(scenario.directed_coupling)
                [obj.isDAG, obj.topo_groups] = topological_sorting_coloring(scenario.adjacency(:,:,end));
            else
                [obj.isDAG, obj.topo_groups] = kahn(scenario.directed_coupling(:,:,end));
            end

            assert( obj.isDAG, 'Coupling matrix is not a DAG' );
            groups = PB_predecessor_groups(obj.topo_groups);

            % Assign prrority according to computation level
            % Vehicles with higher priorities plan trajectory before vehicles
            % with lower priorities            
            priority_list = obj.get_priority(groups,obj.is_assign_unique_priority);

            coupling_undirected = scenario.adjacency(:,:,end);
            % determine directed adjacency
            coupling_directed = coupling_undirected;
            [rows, cols] = find(coupling_undirected~=0);
            for k = 1:length(rows)
                v_i = rows(k); 
                v_j = cols(k);
                level_i = find(obj.topo_groups(:,v_i)==1);
                level_j = find(obj.topo_groups(:,v_j)==1);
                % edge comes from vertex in the front level, ends in vertex in
                % back level
                if level_i > level_j
                    coupling_directed(v_i,v_j) = 0;
                end
            end            
        end
        
    end
    


end
