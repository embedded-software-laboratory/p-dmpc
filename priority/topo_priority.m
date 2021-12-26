classdef  topo_priority < interface_priority
% random_priority  Instance of experiment interface used for simulation in matlab.
    
    properties (Access=private)
        isDAG
        topo_groups
        
    end
    
    methods 
        function obj = topo_priority(scenario)
            obj.scenario = scenario;
            
            if obj.scenario.assignPrios || isempty(obj.scenario.directed_coupling)
                [obj.isDAG, obj.topo_groups] = topological_sorting_coloring(obj.scenario.adjacency(:,:,end));
            else
                [obj.isDAG, obj.topo_groups] = kahn(obj.scenario.directed_coupling(:,:,end));
            end
            
        end
        
        function groups = priority(obj)
            assert( obj.isDAG, 'Coupling matrix is not a DAG' );
            groups = PB_predecessor_groups(obj.topo_groups);
            
        end
        
    end
    


end
