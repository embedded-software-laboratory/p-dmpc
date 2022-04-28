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
        reachable_sets           % Store all the reachable sets of each trim (possibly non-convex)
        reachable_sets_conv;     % Convexified reachable sets of each trim
    end
    
    methods
        function obj = MotionPrimitiveAutomaton(model, trim_set, offset, dt, nveh, N, nTicks, recursive_feasibility)
            % Constructor
            % trim_inputs is a matrix of size (nTrims x nu)
            % trim_adjacency is a matrix of size (nTrims x nTrims), 
            %   read as: rows are start trims and columns are end trims
            % N is the horizon length

            % path of the MPA
            [file_path,name,ext] = fileparts(mfilename('fullpath'));
            folder_target = [file_path,filesep,'library'];
            mpa_instance_name = ['MPA_','trims',num2str(trim_set),'_Hp',num2str(N),'.mat'];
            mpa_full_path = [folder_target,filesep,mpa_instance_name];

            % if the needed MPA is alread exist in the library, simply load it
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
            obj.reachable_sets = cell(n_trims,N);
            obj.reachable_sets_conv = cell(n_trims,N);
                
            % compute the reachable sets of each trim
            
            [obj.reachable_sets, obj.reachable_sets_conv] = reachability_analysis_offline(obj,N);
            
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

    end
end