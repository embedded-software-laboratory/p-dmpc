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
        distance_to_equilibrium % Distance in graph from current state to equilibrium state (nTrims x 1)
    end
    
    methods
        
        function obj = MotionGraph(model, trim_inputs, trim_adjacency, offset, dt)
            % Constructor
            % trim_inputs is a matrix of size (nTrims x nu)
            % trim_adjacency is a matrix of size (nTrims x nTrims), 
            %   read as: rows are start trims and columns are end trims
            
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

            % compute distance to equilibrium state
            eq_states = find(trim_inputs(:,2)==0);
            obj.distance_to_equilibrium = zeros(n_trims,1);
            for iTrim = 1:n_trims
                dist_to_eq_state = 0;
                visited_states = iTrim;
                [obj.distance_to_equilibrium(iTrim),~] = obj.comp_dist_to_eq_state(iTrim, dist_to_eq_state, visited_states, eq_states);
            end
        end
        
        function [dist_to_eq_state, visited_states] = comp_dist_to_eq_state(obj, state, dist_to_eq_state, visited_states,eq_states)
            if ismember(state, eq_states)
                % return
            else
                dist_to_eq_state = dist_to_eq_state + 1;
                min_dist = Inf;
                for s = find(obj.transition_matrix(state,:))
                    if (ismember(s, visited_states))
                        next_dist = Inf;
                    else
                        next_visited_states = [visited_states, s];
                        [next_dist,~] = obj.comp_dist_to_eq_state(s,dist_to_eq_state,next_visited_states,eq_states);
                    end
                    if next_dist < min_dist
                        min_dist = next_dist;
                    end
                end
                dist_to_eq_state = min_dist;
            end
        end
        
        
    end
end
