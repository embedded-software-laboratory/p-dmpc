classdef MotionPrimitiveAutomaton
% MOTIONPRIMITVEAUTOMATON   MotionPrimitiveAutomaton 
   
    properties
        maneuvers % cell(n_trims, n_trims)
        trims % A struct array of the specified trim_inputs
        transition_matrix_single % Matrix (nTrims x nTrims x horizon_length)
        trim_tuple               % Matrix with trim indices ((nTrims1*nTrims2*...) x nVehicles)
        transition_matrix        % binary Matrix (if maneuverTuple exist according to trims) (nTrimTuples x nTrimTuples x horizon_length)
        distance_to_equilibrium  % Distance in graph from current state to equilibrium state (nTrims x 1)
        recursive_feasibility
        local_reachable_sets           % Reachable sets of each trim (possibly non-convex); 
                                            % It's local because the position and yaw angle of vehicles are not considered
        local_reachable_sets_conv;     % Convexified reachable sets of each trim
    end
    
    methods
        function obj = MotionPrimitiveAutomaton(model, trim_set, offset, dt, nveh, N, nTicks, recursive_feasibility)
            % Constructor
            % trim_inputs is a matrix of size (nTrims x nu)
            % trim_adjacency is a matrix of size (nTrims x nTrims), 
            %   read as: rows are start trims and columns are end trims
            % N is the horizon length

            % path of the MPA
            [file_path,~,~] = fileparts(mfilename('fullpath'));
            folder_target = [file_path,filesep,'library'];
            mpa_instance_name = ['MPA_','trims',num2str(trim_set),'_Hp',num2str(N),'.mat'];
            mpa_full_path = [folder_target,filesep,mpa_instance_name];

            % if the needed MPA is alread exist in the library, simply load
            % it, otherwise it will be calculated and saved to the library.
            if isfile(mpa_full_path)
                load(mpa_full_path,"mpa");
                obj = mpa;
                return
            end

            obj.recursive_feasibility = recursive_feasibility;
                        
            [trim_inputs, trim_adjacency] = choose_trims(trim_set);
            n_trims = length(trim_inputs);
            
            obj.transition_matrix_single = zeros([size(trim_adjacency),N]);
            obj.transition_matrix_single(:,:,:) = repmat(trim_adjacency,1,1,N);
            
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
                    if obj.transition_matrix_single(i,j,1)
                        obj.maneuvers{i,j} = generate_maneuver(model, obj.trims(i), obj.trims(j), offset, dt, nTicks);
                    end
                end
            end

            % compute distance to equilibrium state
            eq_states = find(trim_inputs(:,2)==0);            
            adj_trims = graph(obj.transition_matrix_single(:,:,1));
            obj.distance_to_equilibrium = distances(adj_trims,eq_states);

            % compute trim tuple (vertices)
            trim_index_list = cell(nveh,1);
            [trim_index_list{:}] = deal(1:n_trims);
            obj.trim_tuple = cartprod(trim_index_list{:});
            
            if recursive_feasibility
                obj.transition_matrix_single = compute_time_varying_transition_matrix(obj);
            end

            % compute maneuver matrix for trimProduct
            obj.transition_matrix = compute_product_maneuver_matrix(obj,nveh,N);
            
            % variables to store reachable sets in different time steps
            obj.local_reachable_sets = cell(n_trims,N);
            obj.local_reachable_sets_conv = cell(n_trims,N);
                
            % compute the reachable sets of each trim
            
            [obj.local_reachable_sets, obj.local_reachable_sets_conv] = reachability_analysis_offline(obj,N);
            
            save_mpa(obj,mpa_full_path); % save mpa to library
            
        end
    
        function max_speed = get_max_speed(obj, cur_trim_id)
            % returns maximum speed, averaged over the timestep (nSamples x 1)
            % is not general, but works for current MPAs
            N = size(obj.transition_matrix,3);
            max_speed = zeros(N,1);
            max_speed_last = obj.trims(cur_trim_id).speed;
            for k = 1:N
                successor_trim_ids = find(obj.transition_matrix_single(cur_trim_id, :, k));
                [max_speed_next, cur_trim_id] = max([obj.trims(successor_trim_ids).speed]);
                max_speed(k) = (max_speed_next + max_speed_last)/2; % assumes linear change
                max_speed_last = max_speed_next;
            end
        end

        
        function transition_matrix_single = compute_time_varying_transition_matrix(obj)
            N = size(obj.transition_matrix_single,3);
            transition_matrix_single = obj.transition_matrix_single;
            for k = 1:N
                % Columns are end trims. Forbid trims whose distance 
                % to an equilbrium state is too high
                k_to_go = N-k;
                transition_matrix_single(:,obj.distance_to_equilibrium>k_to_go,k) = 0;
            end
        end


        function maneuver_matrix = compute_product_maneuver_matrix(obj,nveh,N)
            nTrimTuples = size(obj.trim_tuple,1);
            maneuver_matrix = zeros(nTrimTuples,nTrimTuples,N);
            % Assumes Hp=Hu
            for k = 1:N
                transition_matrix_slice = obj.transition_matrix_single(:,:,k);
                % compute tensor product iteratively
                for i = 2 : nveh
                    transition_matrix_slice = kron(transition_matrix_slice,obj.transition_matrix_single(:,:,k));
                end
                maneuver_matrix(:,:,k) = transition_matrix_slice;
            end
        end

        function save_mpa(obj,mpa_full_path)
            % Save MPA to library
            mpa = obj;
            save(mpa_full_path,'mpa');
        end

        function emergency_braking_distance = get_emergency_braking_distance(obj, cur_trim_id, time_step)
            % returns the emergency braking distance starting from the current trim
            emergency_braking_distance = 0;
            speed_cur = obj.trims(cur_trim_id).speed;

            % compute the shortest path from the current trim to the equilibrium trim
            equilibrium_trim = find([obj.trims.speed]==0);
            assert(length(equilibrium_trim)==1) % if there are multiple equilibrium states, this function should be then adapted
            graph_trims = graph(obj.transition_matrix_single(:,:,1));
            shortest_path_to_equilibrium = shortestpath(graph_trims,cur_trim_id,equilibrium_trim); % shortest path between current trim and equilibrium trim

            for iTrim=shortest_path_to_equilibrium(2:end)
                speed_next = obj.trims(iTrim).speed;
                speed_mean = (speed_cur+speed_next)/2; % assume linear change
                emergency_braking_distance = emergency_braking_distance + speed_mean*time_step;
                speed_cur = speed_next; % update the current speed for the next iteration
            end
        end
        
        function shortest_time_to_arrive = get_the_shortest_time_to_arrive(obj, cur_trim_id, distance_destination, time_step)
            % Returns the shortest time to arive a given distance starting from the current trim
            % Noted that this is only a lower bound time because the
            % steering angle is not considered, namely we assume the
            % vehicle drives straight to arrive the goal destination.
            shortest_time_to_arrive = 0;
            distance_remained = distance_destination;
            distance_acceleration = 0; % acceleration distance
            % compute the shortest path from the current trim to the
            % trim(s) with maximum speed
            max_speed = max([obj.trims.speed]);
            max_speed_trims = find([obj.trims.speed]==max_speed); % find all the trims with the maximum speed

            graph_trims = graph(obj.transition_matrix_single(:,:,1));
            shortest_distances_to_max_speed = distances(graph_trims,cur_trim_id,max_speed_trims); % shortest path between two single nodes
            % find the one which has the minimal distance to the trims with the maximum speed
            [min_distance,idx] = min(shortest_distances_to_max_speed); 
            if min_distance==0 % if the current trim has already the maximum speed, no acceleration is needed
                shortest_time_to_arrive = distance_remained/max_speed;
            else % acceleration to maximum speed
                max_speed_trim = max_speed_trims(idx);
                shortest_path_to_max_speed = shortestpath(graph_trims,cur_trim_id,max_speed_trim); % shortest path between two single nodes
                for i=1:length(shortest_path_to_max_speed)
                    trim_current = shortest_path_to_max_speed(i);
                    
                    speed_cur = obj.trims(trim_current).speed;
                    if i+1<=length(shortest_path_to_max_speed)
                        trim_next = shortest_path_to_max_speed(i+1);
                        speed_next = obj.trims(trim_next).speed;
                    else
                        speed_next = max_speed;
                    end
                    mean_speed = (speed_cur+speed_next)/2;
                    distance_acceleration = distance_acceleration + mean_speed*time_step;
                    if distance_acceleration > distance_destination % if the vehicle arrives the detination when accelerating
                        shortest_time_to_arrive = shortest_time_to_arrive + distance_remained/mean_speed; % time accumulates
                        break
                    else
                        shortest_time_to_arrive = shortest_time_to_arrive + time_step; % time accumulates
                        distance_remained = distance_destination - distance_acceleration;
                    end
                end

                % if the detination is still not arrived after acceleration, calculate the remaining time using the maximum speed
                if distance_remained>0
                    shortest_time_to_arrive = shortest_time_to_arrive + distance_remained/max_speed;
                end

            end
        end


        

    end
end