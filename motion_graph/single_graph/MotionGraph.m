classdef MotionGraph
    % Class for MotionGraph
    
    % trim
    %   u
    % maneuver
    %   dx
    %   dy
    %   dyaw
    %   area
    
    properties
        trims               % struct array
        maneuvers           % Cell/struct matrix (nTrims x nTrims)
        transition_matrix   % Matrix      (nTrims x nTrims) (not necessary ??)
    end
    
    methods
        
        function obj = MotionGraph(model, trim_inputs, trim_adjacency, dt)
            % Constructor
            
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
                        obj.maneuvers{i,j} = generate_maneuver(model, obj.trims(i), obj.trims(j), dt);
                    end
                end
            end
        end
        
        
        
    end
end
