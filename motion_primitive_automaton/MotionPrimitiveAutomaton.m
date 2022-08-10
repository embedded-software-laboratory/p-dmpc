classdef MotionPrimitiveAutomaton
% MOTIONPRIMITVEAUTOMATON   MotionPrimitiveAutomaton 
   
    properties
        maneuvers                   % cell(n_trims, n_trims)
        trims                       % A struct array of the specified trim_inputs
        transition_matrix_single    % Matrix (nTrims x nTrims x horizon_length)
        trim_tuple                  % Matrix with trim indices ((nTrims1*nTrims2*...) x nVehicles)
        transition_matrix           % binary Matrix (if maneuverTuple exist according to trims) (nTrimTuples x nTrimTuples x horizon_length)
        transition_matrix_mean_speed% nTrims-by-nTrims matrix, each entry is the mean speed of the two connected trims
        distance_to_equilibrium     % Distance in graph from current state to equilibrium state (nTrims x 1)
        recursive_feasibility
        local_reachable_sets        % local reachable sets of each trim (possibly non-convex)
        local_reachable_sets_conv;  % Convexified local reachable sets of each trim
        local_center_trajectory     % local trajcetory of the center point
        local_reachable_sets_CP     % local reachable sets of the center point
        emengency_braking_maneuvers % cell(n_trims, 1), local occupied area of emergency braking maneuver
        shortest_paths_to_max_speed     % cell(n_trims, 1), the shortest path in the trim graph from the current trim to the trim with maximum speed
        shortest_paths_to_equilibrium   % cell(n_trims, 1), the shortest path in the trim graph from the current trim to the trim with zero speed
        special_trims                   % struct, store which trim corresponds to turn most left/right and go straight; if multiple trims, choose the one with the smallest speed
        special_maneuvers               % cell(n_trims, 1), special maneuvers, such as most-left trun, most-left trun twice, most-right turn, most-right turn twice
    end
    
    methods
        function obj = MotionPrimitiveAutomaton(model, trim_ID, offset, dt, nveh, N, nTicks, recursive_feasibility, is_allow_non_convex, options)
            % Constructor
            % trim_inputs is a matrix of size (nTrims x nu)
            % trim_adjacency is a matrix of size (nTrims x nTrims), 
            %   read as: rows are start trims and columns are end trims
            % N is the horizon length

            % path of the MPA
            [file_path,~,~] = fileparts(mfilename('fullpath'));
            folder_target = [file_path,filesep,'library'];
            if ~isfolder(folder_target)
                % create target folder if not exist
                mkdir(folder_target)
            end

            % for example: MPA_trims12_Hp6, MPA_trims12_Hp6_parl_non-convex
            mpa_instance_name = FileNameConstructor.get_mpa_name(trim_ID,N,dt,options.isParl,is_allow_non_convex);

            mpa_full_path = fullfile(folder_target,mpa_instance_name);

            % if the needed MPA is alread exist in the library, simply load
            % it, otherwise it will be calculated and saved to the library.
            % Note: if MPA properties are changed, then reload all MPAs!
            if isfile(mpa_full_path)
                load(mpa_full_path,"mpa");
                obj = mpa;
                disp('Offline MPA was found and loaded.')
                return
            end

            obj.recursive_feasibility = recursive_feasibility;
                        
            [trim_inputs, trim_adjacency] = choose_trims(trim_ID);
            n_trims = length(trim_inputs);
            
            obj.transition_matrix_single = zeros([size(trim_adjacency),N]);
            obj.transition_matrix_single(:,:,:) = repmat(trim_adjacency,1,1,N);

            obj.transition_matrix_mean_speed = zeros(size(trim_adjacency));

            for iTrim = 1:length(trim_inputs)
                for jTrim = 1:length(trim_inputs)
                    if trim_adjacency(iTrim,jTrim)~=0
                        obj.transition_matrix_mean_speed(iTrim,jTrim) = (trim_inputs(iTrim,2) + trim_inputs(jTrim,2)) / 2;
                    end
                end
            end
            
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
            
            % get special trims
            obj.special_trims = struct('go_straight',[],'turn_left_most',[],'turn_right_most',[],'equilibrium',[]);
            obj.special_trims.equilibrium = find([obj.trims.steering]==0 & [obj.trims.speed]==0);
            steering_zero = 0; % go straight
            steering_max = max([obj.trims.steering]); % turn left 
            steering_min = min([obj.trims.steering]); % turn right
            special_steerings = [steering_zero,steering_max,steering_min];

            for iTurn = 1:length(special_steerings)
                special_steering = special_steerings(iTurn);
                find_target_steering_trims = find([obj.trims.steering]==special_steering);
                if iTurn==1
                    % delete equilibrium of go-straight trims
                    find_target_steering_trims = setdiff(find_target_steering_trims,obj.special_trims.equilibrium);
                end
                if length(find_target_steering_trims)>1
                    % if multiple trims have the maximum steering angle, choose the one with the smallest speed
                    [~,idx_special_steering_min_speed] = min([obj.trims(find_target_steering_trims).speed]);
                else
                    idx_special_steering_min_speed = 1;
                end
                switch iTurn
                    case 1
                        obj.special_trims.go_straight = find_target_steering_trims(idx_special_steering_min_speed);
                    case 2
                        obj.special_trims.turn_left_most = find_target_steering_trims(idx_special_steering_min_speed);
                    case 3
                        obj.special_trims.turn_right_most = find_target_steering_trims(idx_special_steering_min_speed);
                end
            end

            % maneuver cell/struct matrix
            for i = 1:n_trims
                for j = 1:n_trims
                    if obj.transition_matrix_single(i,j,1)
                        obj.maneuvers{i,j} = generate_maneuver(model, obj.trims(i), obj.trims(j), offset, dt, nTicks, is_allow_non_convex);
                    end
                end
                obj.shortest_paths_to_max_speed{i,1} = get_shortest_path_to_max_speed(obj,i);
                obj.shortest_paths_to_equilibrium{i,1} = get_shortest_path_to_equilibrium(obj,i);
                obj.emengency_braking_maneuvers{i,1} = get_emergency_braking_maneuver(obj,i);
            end

            obj.special_maneuvers = generate_special_maneuvers(obj);

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
                
            % For parallel computation, reachability analysis are used
            if options.isParl
                is_calculate_reachable_sets_of_CP = false; % whether to calculate center point's reachable sets
                [obj.local_reachable_sets, obj.local_reachable_sets_conv, obj.local_center_trajectory, obj.local_reachable_sets_CP] = ...
                    reachability_analysis_offline_DP(obj,N,is_calculate_reachable_sets_of_CP);
            end
            
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

        function [reachable_sets_local, reachable_sets_conv_local, center_trajectory, reachable_sets_CP] = reachability_analysis_offline(obj, Hp, is_calculate_reachable_sets_of_CP)
            % Calculate local reachable sets starting from a certain trim,
            % which can be used for online reachability analysis
            % 
            % INPUT:
            %   obj: motion primitive automaton calss
            %   
            %   Hp: prediction horizon
            %
            %   is_calculate_reachable_sets_of_CP: whether to calculate center point's reachable sets
            % 
            % OUTPUT:
            %   reachable_sets_local: cell [n_trims x Hp]. The union of local reachable
            %   sets  
            %     
            %   reachable_sets_conv_local: cell [n_trims x Hp]. The convexified union
            %   of local reachable sets 
            
            offline_computation_start = tic;
            threshold_Hp = 6;
            if Hp > threshold_Hp
                disp(['Computing reachable sets now...' newline ...
                    'Since the prediction horizon is ' num2str(Hp) ' (more than ' num2str(threshold_Hp) '), it may take several minutes.'])
            else
                disp('Computing local offline reachable sets now...')
            end
            disp('Note this only needs to be done once as later they will be saved to motion-primitive-automaton library.')
        
            n_trims = numel(obj.trims);
            reachable_sets_local = repmat({polyshape},n_trims,Hp); % cell array with empty polyshapes
            reachable_sets_conv_local = cell(n_trims,Hp);
            center_trajectory = cell(n_trims,Hp);
            reachable_sets_CP = repmat({polyshape},n_trims,Hp);
        
            % transform maneuver area to polyshape which is required when using
            % MATLAB function `union`
            for i=1:n_trims
                child_trims = find(obj.transition_matrix_single(i,:,1));
                for idx=1:length(child_trims)
                    j = child_trims(idx);
                    obj.maneuvers{i,j}.areaPoly = polyshape(obj.maneuvers{i,j}.area(1,:),obj.maneuvers{i,j}.area(2,:),'Simplify',false);
                end
            end

            trimsInfo = struct;

            % display progress
            textprogressbar('Computing local offline reachable sets: ');
            progress = 0;
            textprogressbar(progress);

            for i=1:n_trims
                for t=1:Hp
                    if t==1 % root trim
                        trimsInfo(i,t).parentTrims = i;
                    else % The child trims become parent trims of the next time step
                        trimsInfo(i,t).parentTrims = trimsInfo(i,t-1).childTrims;
                    end
                    
                    % variable to store all child trims of the parent trims
                    trimsInfo(i,t).childTrims = [];
                    trimsInfo(i,t).childNum = [];
                    % find child trims of the parent trims
                    for i_Trim=1:length(trimsInfo(i,t).parentTrims)
                        find_child = find(obj.transition_matrix_single(trimsInfo(i,t).parentTrims(i_Trim),:,t));
                        trimsInfo(i,t).childTrims = [trimsInfo(i,t).childTrims find_child];
                        trimsInfo(i,t).childNum = [trimsInfo(i,t).childNum length(find_child)];
                    end
                                        
                    % loop through all parent trims
                    for j=1:length(trimsInfo(i,t).parentTrims)
                        if t==1
                            x0 = 0;
                            y0 = 0;
                            yaw0 = 0;
                        else
                            x0 = trimsInfo(i,t-1).maneuvers{j}.xs(end);
                            y0 = trimsInfo(i,t-1).maneuvers{j}.ys(end);
                            yaw0 = trimsInfo(i,t-1).maneuvers{j}.yaws(end);
                        end

                        % loop through all child trims
                        for k=1:trimsInfo(i,t).childNum(j)
                            trim_start = trimsInfo(i,t).parentTrims(j);
                            child_ordinal = sum(trimsInfo(i,t).childNum(1:j-1)) + k;
                            trim_end = trimsInfo(i,t).childTrims(child_ordinal);
            
                            % tranlates the local coordinates to global coordinates
                            [trimsInfo(i,t).maneuvers{child_ordinal}.xs, trimsInfo(i,t).maneuvers{child_ordinal}.ys] = ...
                                translate_global(yaw0,x0,y0,obj.maneuvers{trim_start,trim_end}.xs,obj.maneuvers{trim_start,trim_end}.ys);
                            trimsInfo(i,t).maneuvers{child_ordinal}.yaws = yaw0 + obj.maneuvers{trim_start,trim_end}.yaws;

                            % occupied area of the translated maneuvers
                            [area_x, area_y] = ...
                                translate_global(yaw0,x0,y0,obj.maneuvers{trim_start,trim_end}.area(1,:),obj.maneuvers{trim_start,trim_end}.area(2,:));
                            trimsInfo(i,t).maneuvers{child_ordinal}.area = [area_x;area_y];
                            trimsInfo(i,t).maneuvers{child_ordinal}.areaPoly = polyshape(area_x,area_y,'Simplify',false);
                        end
                    end
                    
                    % MATLAB function `union()` can union multiple polygons
                    % at the same time, but the number should not be very large
                    % as it will run significantly slow.
                    size_union = 15;
                    n_union_times = ceil(length(trimsInfo(i,t).maneuvers)/size_union);
                    for p = 1:n_union_times
                        if p == n_union_times
                            areaPolys = cellfun(@(c) [c.areaPoly], trimsInfo(i,t).maneuvers((p-1)*size_union+1:end));
                        else
                            areaPolys = cellfun(@(c) [c.areaPoly], trimsInfo(i,t).maneuvers((p-1)*size_union+1:p*size_union));
                        end
                        reachable_sets_local{i,t} = union([areaPolys,reachable_sets_local{i,t}]);
                    end
                    reachable_sets_conv_local{i,t} = convhull(reachable_sets_local{i,t}); % convexify

                    if is_calculate_reachable_sets_of_CP
                        % calculate trajectory and convexified reachable sets of the center point
                        center_trajectory{i,t} = cellfun(@(c) [c.xs;c.ys], trimsInfo(i,t).maneuvers,'UniformOutput',false);
                        center_tra_xy = [center_trajectory{i,t}{:}];
                        idx_center_tra_conv= convhull(center_tra_xy(1,:),center_tra_xy(2,:));
                        reachable_sets_CP{i,t} = polyshape(center_tra_xy(1,idx_center_tra_conv),center_tra_xy(2,idx_center_tra_conv));
                    end

                    % display progress
                    progress = ((i-1)*Hp + t)/(n_trims*Hp)*100; % percentage 
                    textprogressbar(progress);
                end
            end
            duration_computation = toc(offline_computation_start);
            textprogressbar('done');
            disp(['Finished in ' num2str(duration_computation) ' seconds.'])
        end

        function save_mpa(obj,mpa_full_path)
            % Save MPA to library
            mpa = obj;
            save(mpa_full_path,'mpa');
        end



        function [reachable_sets_local, reachable_sets_conv_local, center_trajectory, reachable_sets_CP] = reachability_analysis_offline_DP(obj, Hp, is_calculate_reachable_sets_of_CP)
            % Calculate local reachable sets starting from a certain trim using dynamic programming,
            % which can be used for online reachability analysis
            % 
            % INPUT:
            %   obj: motion primitive automaton calss
            %   
            %   Hp: prediction horizon
            %
            %   is_calculate_reachable_sets_of_CP: whether to calculate center point's reachable sets
            % 
            % OUTPUT:
            %   reachable_sets_local: cell [n_trims x Hp]. The union of local reachable
            %   sets  
            %     
            %   reachable_sets_conv_local: cell [n_trims x Hp]. The convexified union
            %   of local reachable sets 
            
            offline_computation_start = tic;
            threshold_Hp = 10;
            if Hp > threshold_Hp
                disp(['Computing reachable sets now...' newline ...
                    'Since the prediction horizon is ' num2str(Hp) ' (more than ' num2str(threshold_Hp) '), it may take several minutes.'])
            else
                disp('Computing local offline reachable sets now...')
            end
            disp('Note this only needs to be done once as later they will be saved to motion-primitive-automaton library.')

            Hp_half = ceil(Hp/2); % for time step greater then half of the prediction horizon, dynamic programming is used to save computation time
        
            n_trims = numel(obj.trims);
            reachable_sets_local = repmat({polyshape},n_trims,Hp); % cell array with empty polyshapes
            reachable_sets_local_HpHalf = repmat({polyshape},n_trims,1); % reachable sets of the time step "Hp_half" assuming that "Hp_half" is the last time step (so that the last trim must end with the equilibrium trim)
            reachable_sets_conv_local = cell(n_trims,Hp);
            reachable_sets_conv_local_HpHalf = cell(n_trims,1);
            center_trajectory = cell(n_trims,Hp);
            reachable_sets_CP = repmat({polyshape},n_trims,Hp);
        
            % transform maneuver area to polyshape which is required when using
            % MATLAB function `union`
            for i=1:n_trims
                child_trims = find(obj.transition_matrix_single(i,:,1));
                for idx=1:length(child_trims)
                    j = child_trims(idx);
                    obj.maneuvers{i,j}.areaPoly = polyshape(obj.maneuvers{i,j}.area(1,:),obj.maneuvers{i,j}.area(2,:),'Simplify',false);
                end
            end

            trimsInfo = struct;
            trimsInfoHpHalf = struct;

            % display progress
            textprogressbar('Computing local offline reachable sets: ');
            progress = 0;
            textprogressbar(progress);

            for i=1:n_trims
                for t=1:Hp_half
                    if t==1 % root trim
                        trimsInfo(i,t).parentTrims = i;
                    else % The child trims become parent trims of the next time step
                        trimsInfo(i,t).parentTrims = trimsInfo(i,t-1).childTrims;
                    end
                    
                    % variable to store all child trims of the parent trims
                    trimsInfo(i,t).childTrims = [];
                    trimsInfo(i,t).childNum = [];
                    % find child trims of the parent trims
                    for i_Trim=1:length(trimsInfo(i,t).parentTrims)
                        find_child = find(obj.transition_matrix_single(trimsInfo(i,t).parentTrims(i_Trim),:,t));
                        trimsInfo(i,t).childTrims = [trimsInfo(i,t).childTrims find_child];
                        trimsInfo(i,t).childNum = [trimsInfo(i,t).childNum length(find_child)];
                    end
                                        
                    % loop through all parent trims
                    for j=1:length(trimsInfo(i,t).parentTrims)
                        if t==1
                            x0 = 0;
                            y0 = 0;
                            yaw0 = 0;
                        else
                            x0 = trimsInfo(i,t-1).maneuvers{j}.xs(end);
                            y0 = trimsInfo(i,t-1).maneuvers{j}.ys(end);
                            yaw0 = trimsInfo(i,t-1).maneuvers{j}.yaws(end);
                        end

                        % loop through all child trims
                        for k=1:trimsInfo(i,t).childNum(j)
                            trim_start = trimsInfo(i,t).parentTrims(j);
                            child_ordinal = sum(trimsInfo(i,t).childNum(1:j-1)) + k;
                            trim_end = trimsInfo(i,t).childTrims(child_ordinal);
            
                            % tranlates the local coordinates to global coordinates
                            [trimsInfo(i,t).maneuvers{child_ordinal}.xs, trimsInfo(i,t).maneuvers{child_ordinal}.ys] = ...
                                translate_global(yaw0,x0,y0,obj.maneuvers{trim_start,trim_end}.xs,obj.maneuvers{trim_start,trim_end}.ys);
                            trimsInfo(i,t).maneuvers{child_ordinal}.yaws = yaw0 + obj.maneuvers{trim_start,trim_end}.yaws;

                            % occupied area of the translated maneuvers
                            [area_x, area_y] = ...
                                translate_global(yaw0,x0,y0,obj.maneuvers{trim_start,trim_end}.area(1,:),obj.maneuvers{trim_start,trim_end}.area(2,:));
                            trimsInfo(i,t).maneuvers{child_ordinal}.area = [area_x;area_y];
                            trimsInfo(i,t).maneuvers{child_ordinal}.areaPoly = polyshape(area_x,area_y,'Simplify',false);
                        end

                    end
                    
                    % MATLAB function `union()` can union multiple polygons
                    % at the same time, but the number should not be very large
                    % as it will run significantly slow.
                    size_union = 15;
                    n_union_times = ceil(length(trimsInfo(i,t).maneuvers)/size_union);
                    for p = 1:n_union_times
                        if p == n_union_times
                            areaPolys_tmp = cellfun(@(c) [c.areaPoly], trimsInfo(i,t).maneuvers((p-1)*size_union+1:end));
                        else
                            areaPolys_tmp = cellfun(@(c) [c.areaPoly], trimsInfo(i,t).maneuvers((p-1)*size_union+1:p*size_union));
                        end
                        reachable_sets_local{i,t} = union([areaPolys_tmp,reachable_sets_local{i,t}]);
                    end
                    reachable_sets_conv_local{i,t} = convhull(reachable_sets_local{i,t}); % convexify

                    if is_calculate_reachable_sets_of_CP
                        % calculate trajectory and convexified reachable sets of the center point
                        center_trajectory{i,t} = cellfun(@(c) [c.xs;c.ys], trimsInfo(i,t).maneuvers,'UniformOutput',false);
                        center_tra_xy = [center_trajectory{i,t}{:}];
                        idx_center_tra_conv= convhull(center_tra_xy(1,:),center_tra_xy(2,:));
                        reachable_sets_CP{i,t} = polyshape(center_tra_xy(1,idx_center_tra_conv),center_tra_xy(2,idx_center_tra_conv));
                    end

                    % display progress
                    progress = ((i-1)*Hp + t)/(n_trims*Hp)*100/2; % percentage. Divide by 2 since the left half will be calculated using dynamic programming later
                    textprogressbar(progress);
                end
            end

            % use dynamic programming to finished the remaining half
            % pridition horizon
            
            % assume that Hp_half is the final time step and thus
            % the child trim must end with the equilibrium trim
            for ii = 1:n_trims
                tt = Hp_half;
                
                % parent trims are unchanged, only child trims are reducted
                trimsInfoHpHalf(ii,1).parentTrims = trimsInfo(ii,tt).parentTrims;
                % variable to store all child trims of the parent trims
                trimsInfoHpHalf(ii,1).childTrims = [];
                trimsInfoHpHalf(ii,1).childNum = [];
                % find child trims of the parent trims
                for i_Trim=1:length(trimsInfoHpHalf(ii,1).parentTrims)
                    find_child = find(obj.transition_matrix_single(trimsInfoHpHalf(ii,1).parentTrims(i_Trim),:,end));
                    trimsInfoHpHalf(ii,1).childTrims = [trimsInfoHpHalf(ii,1).childTrims find_child];
                    trimsInfoHpHalf(ii,1).childNum = [trimsInfoHpHalf(ii,1).childNum length(find_child)];
                end

                % loop through all parent trims
                for j=1:length(trimsInfoHpHalf(ii,1).parentTrims)
                    if tt==1
                        x0 = 0;
                        y0 = 0;
                        yaw0 = 0;
                    else
                        x0 = trimsInfo(ii,tt-1).maneuvers{j}.xs(end);
                        y0 = trimsInfo(ii,tt-1).maneuvers{j}.ys(end);
                        yaw0 = trimsInfo(ii,tt-1).maneuvers{j}.yaws(end);
                    end
                    % loop through all child trims
                    for k=1:trimsInfoHpHalf(ii,1).childNum(j)
                        trim_start = trimsInfoHpHalf(ii,1).parentTrims(j);
                        child_ordinal = sum(trimsInfoHpHalf(ii,1).childNum(1:j-1)) + k;
                        trim_end = trimsInfoHpHalf(ii,1).childTrims(child_ordinal);
        
                        % tranlates the local coordinates to global coordinates
                        [trimsInfoHpHalf(ii,1).maneuvers{child_ordinal}.xs, trimsInfoHpHalf(ii,1).maneuvers{child_ordinal}.ys] = ...
                            translate_global(yaw0,x0,y0,obj.maneuvers{trim_start,trim_end}.xs,obj.maneuvers{trim_start,trim_end}.ys);
                        trimsInfoHpHalf(ii,1).maneuvers{child_ordinal}.yaws = yaw0 + obj.maneuvers{trim_start,trim_end}.yaws;

                        % occupied area of the translated maneuvers
                        [area_x, area_y] = ...
                            translate_global(yaw0,x0,y0,obj.maneuvers{trim_start,trim_end}.area(1,:),obj.maneuvers{trim_start,trim_end}.area(2,:));
                        trimsInfoHpHalf(ii,1).maneuvers{child_ordinal}.area = [area_x;area_y];
                        trimsInfoHpHalf(ii,1).maneuvers{child_ordinal}.areaPoly = polyshape(area_x,area_y,'Simplify',false);
                    end
                end

                n_union_times = ceil(length(trimsInfoHpHalf(ii,1).maneuvers)/size_union);
                for p = 1:n_union_times
                    if p == n_union_times
                        areaPolys_tmp = cellfun(@(c) [c.areaPoly], trimsInfoHpHalf(ii,1).maneuvers((p-1)*size_union+1:end));
                    else
                        areaPolys_tmp = cellfun(@(c) [c.areaPoly], trimsInfoHpHalf(ii,1).maneuvers((p-1)*size_union+1:p*size_union));
                    end
                    reachable_sets_local_HpHalf{ii,1} = union([areaPolys_tmp,reachable_sets_local_HpHalf{ii,1}]);
                end
                reachable_sets_conv_local_HpHalf{ii,1} = convhull(reachable_sets_local_HpHalf{ii,1}); % convexify
            end

            for i=1:n_trims
                for t=Hp_half+1:Hp
                    trimInfo = trimsInfo(i,t-Hp_half);
                    areaPolys = repmat(polyshape,1,length(trimInfo.maneuvers));
                    for j=1:length(trimInfo.maneuvers)
                        parentTrim = trimInfo.childTrims(j);
                        if t==Hp
                            [area_x,area_y] = translate_global(trimInfo.maneuvers{j}.yaws(end),trimInfo.maneuvers{j}.xs(end),trimInfo.maneuvers{j}.ys(end), ...
                                reachable_sets_local_HpHalf{parentTrim,1}.Vertices(:,1)',reachable_sets_local_HpHalf{parentTrim,1}.Vertices(:,2)');
                        else
                            [area_x,area_y] = translate_global(trimInfo.maneuvers{j}.yaws(end),trimInfo.maneuvers{j}.xs(end),trimInfo.maneuvers{j}.ys(end), ...
                                reachable_sets_local{parentTrim,Hp_half}.Vertices(:,1)',reachable_sets_local{parentTrim,Hp_half}.Vertices(:,2)');
                        end

                        areaPolys(j) = polyshape(area_x,area_y,'Simplify',false);
                    end
                    % union polyshapes
                    n_union_times = ceil(length(areaPolys)/size_union);
                    for p = 1:n_union_times
                        if p == n_union_times
                            areaPolys_tmp = areaPolys((p-1)*size_union+1:end);
                        else
                            areaPolys_tmp = areaPolys((p-1)*size_union+1:p*size_union);
                        end
                        reachable_sets_local{i,t} = union([areaPolys_tmp,reachable_sets_local{i,t}]);
                    end
                    reachable_sets_conv_local{i,t} = convhull(reachable_sets_local{i,t}); % convexify

                    % display progress
                    progress = 50 + ((i-1)*Hp + t)/(n_trims*Hp)*100/2; % percentage 
                    textprogressbar(progress);
                end
            end

            duration_computation = toc(offline_computation_start);
            textprogressbar('done');
            disp(['Finished in ' num2str(duration_computation) ' seconds.'])
        
        end


        function emergency_braking_distance = get_emergency_braking_distance(obj, cur_trim_id, time_step)
            % returns the emergency braking distance starting from the current trim
            emergency_braking_distance = 0;
            speed_cur = obj.trims(cur_trim_id).speed;

            shortest_path_to_equilibrium = get_shortest_path_to_equilibrium(obj, cur_trim_id);

            for iTrim=shortest_path_to_equilibrium(2:end)
                speed_next = obj.trims(iTrim).speed;
                speed_mean = (speed_cur+speed_next)/2; % assume linear change
                emergency_braking_distance = emergency_braking_distance + speed_mean*time_step;
                speed_cur = speed_next; % update the current speed for the next iteration
            end
        end

        function shortest_path_to_equilibrium = get_shortest_path_to_equilibrium(obj, cur_trim_id)
            % compute the shortest path from the current trim to the equilibrium trim
            equilibrium_trim = find([obj.trims.speed]==0);
            assert(length(equilibrium_trim)==1) % if there are multiple equilibrium states, this function should be then adapted
            graph_trims = graph(obj.transition_matrix_single(:,:,1));
            shortest_path_to_equilibrium = shortestpath(graph_trims,cur_trim_id,equilibrium_trim); % shortest path between current trim and equilibrium trim
        end
        
        function shortest_time_to_arrive = get_the_shortest_time_to_arrive(obj, current_trim, distance_destination, time_step)
            % Returns the shortest time to drive a given distance starting
            % from the current trim.
            % Note that this is only a lower bound time because the
            % steering angle is not considered, namely we assume the
            % vehicle drives straight to arrive the goal destination.
            shortest_time_to_arrive = 0;
            distance_remained = distance_destination;
            distance_acceleration = 0; % acceleration distance
            % compute the shortest path from the current trim to the trim(s) with maximum speed
            shortest_path_to_max_speed = obj.shortest_paths_to_max_speed{current_trim};
            max_speed = obj.trims(shortest_path_to_max_speed(end)).speed;

            % find the one which has the minimal distance to the trims with the maximum speed
            if length(shortest_path_to_max_speed)==1 % if the current trim has already the maximum speed, no acceleration is needed
                shortest_time_to_arrive = distance_remained/max_speed;
            else % acceleration to maximum speed
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

        function braking_time = get_emergency_braking_time(obj, current_trim, braking_distance, time_step)
            % Returns the shortest time to perform emergency braking maneuver starting from the current trim
            % Note that this is only a lower bound time because the
            % steering angle is not considered, namely we assume the
            % vehicle drives straight to arrive the goal destination.
            braking_time = 0;
            distance_remained = braking_distance;
            distance_decceleration = 0; % acceleration distance
            % compute the shortest path from the current trim to the trim(s) with minimum speed
            min_speed = 0;
            min_speed_trims = find([obj.trims.speed]==min_speed); % find all the trims with the minimum speed
    
            graph_trims = graph(obj.transition_matrix_single(:,:,1));
            shortest_distances_to_min_speed = distances(graph_trims,current_trim,min_speed_trims); % shortest path between two single nodes
            % find the one which has the minimal distance to the trims with the minimum speed
            [min_distance,idx] = min(shortest_distances_to_min_speed); 
            if min_distance==0 % if the current trim has already the minimum speed, no decceleration is needed
                braking_time = distance_remained/min_speed;
            else % decceleration to minimum speed
                min_speed_trim = min_speed_trims(idx);
                shortest_path_to_min_speed = shortestpath(graph_trims,current_trim,min_speed_trim); % shortest path between two single nodes
                for i=1:length(shortest_path_to_min_speed)
                    trim_current = shortest_path_to_min_speed(i);
                    
                    speed_cur = obj.trims(trim_current).speed;
                    if i+1<=length(shortest_path_to_min_speed)
                        trim_next = shortest_path_to_min_speed(i+1);
                        speed_next = obj.trims(trim_next).speed;
                    else
                        speed_next = min_speed;
                    end
                    mean_speed = (speed_cur+speed_next)/2;
                    distance_decceleration = distance_decceleration + mean_speed*time_step;
                    if distance_decceleration > braking_distance % if the vehicle arrives the destination when deccelerating
                        braking_time = braking_time + distance_remained/mean_speed; % time accumulates
                        break
                    else
                        braking_time = braking_time + time_step; % time accumulates
                        distance_remained = braking_distance - distance_decceleration;
                    end
                end
            end
        end

        function [time_to_catch, waiting_time, distance_traveled_leader_total, distance_traveled_follower_total] = get_the_shortest_time_to_catch(obj, trim_leader, trim_follower, distance, time_step)
            % Calculatet the shortest time to achieve a collision by
            % letting the leader take an emergency braking and the follower
            % take an full acceleration.
            % The total traveled distances of the leader and the follower are also returned.
            distance_remaining = distance;
            time_to_catch = 0;
            waiting_time = 0; % time that the leader stops and waits for the follower
            distance_traveled_leader_total = 0;
            distance_traveled_follower_total = 0;
            speed_leader = obj.trims(trim_leader).speed;
            speed_follower = obj.trims(trim_follower).speed;

            % get the shortest path from the current trim to the equilibrium trim
            shortest_path_to_equilibrium = obj.shortest_paths_to_equilibrium{trim_leader};

            % get the shortest path to the trims with the maximum speed
            shortest_path_to_max_speed = obj.shortest_paths_to_max_speed{trim_follower};
            trim_max_speed = shortest_path_to_max_speed(end);
            max_speed = obj.trims(trim_max_speed).speed;

            count_trim = 2; % start from the second trim in the path since the first one is the current trim

            distance_traveled = @(v0,a,t) v0*t + 1/2*a*t^2; % calculate distance based on the initial speed and constant acceleration
            is_catch = false;
            while ~is_catch
                if speed_leader ~= 0
                    % leader continue to decelerate
                    trim_leader_next = shortest_path_to_equilibrium(count_trim);
                    speed_leader_next = obj.trims(trim_leader_next).speed;
                    a_leader = (speed_leader_next - speed_leader)/time_step; % acceleration (negative value)
                    % assert(a_leader<=0)
                    % traveled distance of the current time step (assume linear acceleration) 
                    distance_traveled_leader_tmp = distance_traveled(speed_leader, a_leader, time_step);
                else
                    speed_leader_next = 0;
                    a_leader = 0;
                    distance_traveled_leader_tmp = 0;
                end

                if speed_follower ~= max_speed
                    % follower continue to accelerate
                    trim_follower_next = shortest_path_to_max_speed(count_trim);
                    speed_follower_next = obj.trims(trim_follower_next).speed;
                    a_follower = (speed_follower_next - speed_follower)/time_step; % acceleration (positive value)
                    % assert(a_follower>=0)
                    distance_traveled_follower_tmp = distance_traveled(speed_follower, a_follower, time_step); 
                else
                    speed_follower_next = max_speed;
                    a_follower = 0;
                    distance_traveled_follower_tmp = distance_traveled(speed_follower, a_follower, time_step); 
                end

                % remaining distance for the follower to catch the leader
                distance_left_tmp = distance_remaining - distance_traveled_follower_tmp + distance_traveled_leader_tmp;
                if distance_left_tmp <= 0
                    % the follower catchs the leader at this time step
                    is_catch = true;
                    % solve the quadratic equation to get the accumulating time of this time step
                    r = roots([1/2*(a_follower-a_leader), speed_follower-speed_leader, -distance_remaining]); 
                    assert(isreal(r)==true)
                    time_accumulate = max(r); % choose the positive one
                    % update the traveled distance of the current time step 
                    distance_traveled_leader_tmp = speed_leader + 0.5*a_leader*time_accumulate^2;
                    distance_traveled_follower_tmp = distance_traveled_leader_tmp + distance_remaining;

                else
                    % accumulating time equals to the time step
                    time_accumulate = time_step;
                    % update the remaining distance
                    distance_remaining = distance_left_tmp;
                end

                % update the STAC
                time_to_catch = time_to_catch + time_accumulate;
                % update the total traveled distance
                distance_traveled_leader_total = distance_traveled_leader_total + distance_traveled_leader_tmp;
                distance_traveled_follower_total = distance_traveled_follower_total + distance_traveled_follower_tmp;
                % update speeds
                speed_leader = speed_leader_next; % update the current speed for the next iteration
                speed_follower = speed_follower_next; % update the current speed for the next iteration
                % update the waiting time
                if speed_leader==0 && speed_leader_next==0
                    waiting_time = waiting_time + time_accumulate;
                end
                % update counter
                count_trim = count_trim + 1;
            end
        end

        function emengency_braking_maneuver = get_emergency_braking_maneuver(obj,trim_current)
            % Returns the occupied area of emergency braking maneuver for
            % the given trim
            shortest_path_to_equilibrium = obj.shortest_paths_to_equilibrium{trim_current};
            if length(shortest_path_to_equilibrium)==1
                % vehicle already at the equilibrium trim
                trim_next_LP = trim_current;
            else
                trim_next_LP = shortest_path_to_equilibrium(2);
            end
            
            % local shape to global shape
            emengency_braking_maneuver.area = obj.maneuvers{trim_current,trim_next_LP}.area;
            emengency_braking_maneuver.area_without_offset = obj.maneuvers{trim_current,trim_next_LP}.area_without_offset;
            emengency_braking_maneuver.area_large_offset = obj.maneuvers{trim_current,trim_next_LP}.area_large_offset;
        end

        function shortest_path_to_max_speed = get_shortest_path_to_max_speed(obj,trim_current)
            % Returns the shortest path in the trim graph from the current trim to the trim with maximum speed
            % compute the shortest path from the current trim to the trim(s) with maximum speed
            max_speed = max([obj.trims.speed]);
            max_speed_trims = find([obj.trims.speed]==max_speed); % find all the trims with the maximum speed

            graph_weighted = graph(1./obj.transition_matrix_mean_speed);
            
            shortest_distances_to_max_speed = distances(graph_weighted,trim_current,max_speed_trims); % shortest path between two single nodes
            % find the one which has the minimal distance to the trims with the maximum speed
            [~,idx] = min(shortest_distances_to_max_speed); 
            max_speed_trim = max_speed_trims(idx);
            shortest_path_to_max_speed = shortestpath(graph_weighted,trim_current,max_speed_trim); % shortest path between two single nodes
        end

        function special_maneuvers = generate_special_maneuvers(obj)
            % generate special maneuvers starting from the equilibrium trim, such as most-left trun, most-left trun twice, most-right turn, most-right turn twice
            special_maneuvers = struct('go_straight',[],'go_straight_twice',[],'most_left',[],'most_left_twice',[],'most_right',[],'most_right_twice',[]);

            % go straight
            m_straight_1 = obj.maneuvers{obj.special_trims.equilibrium,obj.special_trims.go_straight};
            m_straight_2 = obj.maneuvers{obj.special_trims.go_straight,obj.special_trims.go_straight};
            special_maneuvers.go_straight = m_straight_1.area_without_offset;

            % position
            s_straight_2 = sin(m_straight_1.dyaw);
            c_straight_2 = cos(m_straight_1.dyaw);
            x_straight_2 = c_straight_2*m_straight_2.area_without_offset(1,:) - s_straight_2*m_straight_2.area_without_offset(2,:) + m_straight_1.dx;
            y_straight_2 = s_straight_2*m_straight_2.area_without_offset(1,:) + c_straight_2*m_straight_2.area_without_offset(2,:) + m_straight_1.dy;
            special_maneuvers.go_straight_twice = [x_straight_2;y_straight_2];

            % combine, use a column [nan;nan] to sperate two areas
            special_maneuvers.go_straight_twice = [special_maneuvers.go_straight,[nan;nan],special_maneuvers.go_straight_twice];

            % most left turn
            m_left_1 = obj.maneuvers{obj.special_trims.equilibrium,obj.special_trims.turn_left_most};
            m_left_2 = obj.maneuvers{obj.special_trims.turn_left_most,obj.special_trims.turn_left_most};
            special_maneuvers.most_left = m_left_1.area_without_offset;

            % position
            s_left_2 = sin(m_left_1.dyaw);
            c_left_2 = cos(m_left_1.dyaw);
            x_left_2 = c_left_2*m_left_2.area_without_offset(1,:) - s_left_2*m_left_2.area_without_offset(2,:) + m_left_1.dx;
            y_left_2 = s_left_2*m_left_2.area_without_offset(1,:) + c_left_2*m_left_2.area_without_offset(2,:) + m_left_1.dy;
            special_maneuvers.most_left_twice = [x_left_2;y_left_2];

            % combine, use a column [nan;nan] to sperate two areas
            special_maneuvers.most_left_twice = [special_maneuvers.most_left,[nan;nan],special_maneuvers.most_left_twice];

            m_right_1 = obj.maneuvers{obj.special_trims.equilibrium,obj.special_trims.turn_right_most};
            m_right_2 = obj.maneuvers{obj.special_trims.turn_right_most,obj.special_trims.turn_right_most};
            special_maneuvers.most_right = m_right_1.area_without_offset;

            % position
            s_right_2 = sin(m_right_1.dyaw);
            c_right_2 = cos(m_right_1.dyaw);
            x_right_2 = c_right_2*m_right_2.area_without_offset(1,:) - s_right_2*m_right_2.area_without_offset(2,:) + m_right_1.dx;
            y_right_2 = s_right_2*m_right_2.area_without_offset(1,:) + c_right_2*m_right_2.area_without_offset(2,:) + m_right_1.dy;
            special_maneuvers.most_right_twice = [x_right_2;y_right_2];

            % combine, use a column [nan;nan] to sperate two areas
            special_maneuvers.most_right_twice = [special_maneuvers.most_right,[nan;nan],special_maneuvers.most_right_twice];

        end
    end
end

