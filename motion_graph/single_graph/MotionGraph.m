classdef MotionGraph
% MotionGraph Represents a maneuver automaton    
    properties
        % trims - A struct array of the specified trim_inputs
        trims
        % maneuvers - Cell/struct matrix (nTrims x nTrims)
        %   dx
        %   dy
        %   dyaw
        %   area
        maneuvers
        transition_matrix   % Matrix      (nTrims x nTrims) (not necessary ??)
    end
    
    methods
        
        function obj = MotionGraph(model, trim_inputs, trim_adjacency, offset, dt)
            % Constructor
            % trim_adjacency is a matrix of size (nTrims x nTrims), 
            % read as: rows are start trims and columns are end trims
            
            obj.transition_matrix = trim_adjacency;
                       
            n_trims = length(trim_inputs);
            
            % trim struct array
            % BicycleModel
            if model.nu == 2    
                obj.trims = struct('steering',0,'velocity',0); 
                
            % BicycleModelConstSpeed
            elseif model.nu == 1
                obj.trims = struct('steering',0);
            end 
            
            for i = 1:n_trims
                obj.trims(i) = generate_trim(model, trim_inputs(i,:));
            end
            
            
            % maneuver cell/struct matrix            
            for i = 1:n_trims
                for j = 1:n_trims
                    if obj.transition_matrix(i,j)
                        obj.maneuvers{i,j} = generate_maneuver(model, obj.trims(i), obj.trims(j), offset, dt);
                    end
                end
            end
        end
        
        
        
    end
end
