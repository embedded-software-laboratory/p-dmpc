classdef MotionPrimitiveAutomaton
    properties
        maneuvers % cell(n_trims, n_trims)
        trims % A struct array of the specified trim_inputs
        transition_matrix_single % Matrix (nTrims x nTrims)
        trim_tuple               % Matrix with trim indices ((nTrims1*nTrims2*...) x nVehicles)
        transition_matrix        % binary Matrix (if maneuverTuple exist according to trims) (nTrimTuples x nTrimTuples)
        distance_to_equilibrium % Distance in graph from current state to equilibrium state (nTrims x 1)
    end
    
    methods
        function obj = MotionPrimitiveAutomaton(model, trim_set, offset, dt, nveh)
            % Constructor
            % trim_inputs is a matrix of size (nTrims x nu)
            % trim_adjacency is a matrix of size (nTrims x nTrims), 
            %   read as: rows are start trims and columns are end trims

            [trim_inputs, trim_adjacency] = choose_trims(trim_set);
            n_trims = length(trim_inputs);
            
            obj.transition_matrix_single = trim_adjacency;
            
            % trim struct array
            % BicycleModel
            if model.nu == 2
                obj.trims = struct('steering',0,'speed',0); 
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
                    if obj.transition_matrix_single(i,j)
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

            % compute trim tuple (vertices)
            trim_index_list = cell(nveh,1);
            [trim_index_list{:}] = deal(1:n_trims);
            obj.trim_tuple = cartprod(trim_index_list{:});
            
            % compute maneuver matrix for trimProduct
            obj.transition_matrix = compute_product_maneuver_matrix(obj,nveh);
            
        end
    
        function max_speed = get_max_speed(obj)
            % returns maximum speed per vehicle (nVeh x 1)
            max_speed = max([obj.trims(:).speed]);
        end

        
        function [dist_to_eq_state, visited_states] = comp_dist_to_eq_state(obj, state, dist_to_eq_state, visited_states,eq_states)
            if ismember(state, eq_states)
                % return
            else
                dist_to_eq_state = dist_to_eq_state + 1;
                min_dist = Inf;
                for s = find(obj.transition_matrix_single(state,:))
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

        function maneuver_matrix = compute_product_maneuver_matrix(obj,nveh)
            % initialize product maneuver_matrix
            maneuver_matrix = obj.transition_matrix_single;
            % compute tensor product iteratively
            for i = 2 : nveh
                maneuver_matrix = kron(maneuver_matrix,obj.transition_matrix_single);
            end
        end

    end
end



function [trim_inputs, trim_adjacency] = choose_trims(trim_set)
switch trim_set
case 1
    %% Two speeds (0 and 0.75 m/s), five steering
    steering = (-2:2) * pi/12;
    ntrims = numel(steering)+1;
    trim_inputs = zeros(ntrims,2);
    trim_inputs(2:end,1) = steering;
    trim_inputs(2:end,2) = 0.75;%10 / 3.6;
    trim_adjacency = eye(ntrims);
    % equilibrium is reachable from all states and can reach all states
    trim_adjacency(1, :) = 1;
    trim_adjacency(:, 1) = 1;
    % other states are connected by one hop
    trim_adjacency(2:end-1,3:end) = trim_adjacency(2:end-1,3:end) ...
        + eye(ntrims-2);
    trim_adjacency(3:end,2:end-1) = trim_adjacency(3:end,2:end-1) ...
        + eye(ntrims-2);
case 2
    %% Six speeds (0 to 1.5 m/s), one steering (0 deg)
    speeds = (0:0.3:1.5) / 3.6;
    ntrims = numel(speeds);
    trim_inputs = zeros(ntrims,2);
    trim_inputs(:,2) = speeds;
    trim_adjacency = eye(ntrims);
    trim_adjacency(1:end-1,2:end) = trim_adjacency(1:end-1,2:end) ...
        + eye(ntrims-1);
    trim_adjacency(2:end,1:end-1) = trim_adjacency(2:end,1:end-1) ...
        + eye(ntrims-1);
end
end