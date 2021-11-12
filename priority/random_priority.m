classdef  random_priority < interface_priority
% random_priority  Instance of experiment interface used for simulation in matlab.
    
    properties (Access=protected)
        
        ran_priority
    end
    
    methods 
        function obj = random_priority(scenario)
            obj.scenario = scenario;
        end
        
        function ran_priority = random(obj)
            
            if obj.scenario.assignPrios || isempty(obj.scenario.directed_coupling)
                [isDAG, topo_groups] = topological_sorting_coloring(obj.scenario.adjacency);
            else
                [isDAG, topo_groups] = kahn(obj.scenario.directed_coupling);
            end
            
            assert( isDAG, 'Coupling matrix is not a DAG' );
            ran_priority = PB_predecessor_groups(topo_groups);
            
        end
        
    end
    


end