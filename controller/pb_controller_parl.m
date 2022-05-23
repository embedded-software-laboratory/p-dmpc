function [u, y_pred, info] = pb_controller_parl(scenario, iter)
% PB_CONTROLLER    Plan trajectory for one time step using a priority-based controller.
%     Controller simulates multiple distributed controllers.


switch scenario.priority_option
    case 'topo_priority'
        obj = topo_priority(scenario);
        groups = obj.priority(); 
        right_of_way = false;
        veh_at_intersection = [];
        edge_to_break = [];
    case 'right_of_way_priority'
        obj = right_of_way_priority(scenario, iter);
        right_of_way = true;
        [CL_based_hierarchy, parl_groups_infos, edge_to_break, coupling_weights, belonging_vector] = obj.priority_parl();
        veh_at_intersection = [];
    case 'constant_priority'
        obj = constant_priority(scenario);
        groups = obj.priority(); 
        right_of_way = false;
        veh_at_intersection = [];
        edge_to_break = [];
    case 'random_priority' 
        obj = random_priority(scenario);
        groups = obj.priority(); 
        right_of_way = false;
        veh_at_intersection = [];
        edge_to_break = [];
    case 'FCA_priority'
        obj = FCA_priority(scenario,iter);
        [veh_at_intersection, groups] = obj.priority();
        right_of_way = false;
        edge_to_break = [];   
end


    nVeh = scenario.nVeh;
    Hp = scenario.Hp;

    n_grps = length(parl_groups_infos); % number of parallel groups

    % update the coupling weights matrix in case some edges are inverted to berak circles in the graph 
    scenario.coupling_weights = coupling_weights;

    % construct the priority list
    computation_levels = length(CL_based_hierarchy);
    members_list = horzcat(CL_based_hierarchy.members);
    priority_list = zeros(1,nVeh);
    prio = 1;
    for iVeh = members_list 
        priority_list(iVeh) = prio;
        prio = prio + 1;
    end
    
    y_pred = cell(nVeh,1);
    u = zeros(nVeh,1);
    
    info = struct;
    info.vehicle_fullres_path = cell(nVeh,1);
    info.trim_indices = (-1)*ones(nVeh,1);
    info.subcontroller_runtime = zeros(nVeh,1);
    info.shapes = cell(nVeh,Hp);
    info.next_node = node(-1, zeros(nVeh,1), zeros(nVeh,1), zeros(nVeh,1), zeros(nVeh,1), -1, -1);
    info.n_expanded = 0;
    info.priority_list = priority_list;
    info.veh_at_intersection = veh_at_intersection;
    info.computation_levels = computation_levels;
    info.edge_to_break = edge_to_break;
    
    % graph-search to select the optimal motion primitive
    sub_controller = @(scenario, iter)...
        graph_search(scenario, iter); 

    for level_i = 1:length(CL_based_hierarchy)
        vehs_level_i = CL_based_hierarchy(level_i).members; % vehicles of all groups in the same computation level
        predecessors = CL_based_hierarchy(level_i).predecessors; % vehicles of all groups in the previous computation level
        
        for idx_i = 1:length(vehs_level_i)
            vehicle_i = vehs_level_i(idx_i);

            % only keep self
            filter_self = false(1,scenario.nVeh);
            filter_self(vehicle_i) = true;
            scenario_v = filter_scenario(scenario, filter_self);
            iter_v = filter_iter(iter, filter_self);

            grp_idx = arrayfun(@(array) ismember(vehicle_i,array.vertices), parl_groups_infos);
            all_vehs_same_grp = parl_groups_infos(grp_idx).vertices; % all vehicles in the same group

            all_coupling_vehs_HP = find(scenario_v.coupling_weights(:,vehicle_i)~=0)'; % all coupling vehicles with higher priorities
            all_coupling_vehs_LP = find(scenario_v.coupling_weights(vehicle_i,:)~=0); % all coupling vehicles with lower priorities
 
            coupling_vehs_same_grp_HP = intersect(all_coupling_vehs_HP, all_vehs_same_grp); % coupling vehicles with higher priorities in the same group
            coupling_vehs_other_grps_HP = setdiff(all_coupling_vehs_HP, coupling_vehs_same_grp_HP); % coupling vehicles with higher priorities in other groups
            
            % Collision with coupling vehicles with higher priorities will be avoided by two ways depending on the time step at which their latest messages are sent:
            % 1. Their predicted occupied areas will be considered as dynamic obstacles if the latest messages come from the current time step. 
            % 2. Their reachable sets will be considered as dynamic obstacles if the latest messages come from past time step. 
            for i_HP = 1:length(all_coupling_vehs_HP)
                veh_HP = all_coupling_vehs_HP(i_HP);
                
                latest_msg_HP = scenario_v.ros_subscribers{veh_HP}.LatestMessage;
    
                if latest_msg_HP.time_step == scenario_v.k
                    % if the latest message comes from the current timestep
                    predicted_areas_HP = arrayfun(@(array) {[array.x';array.y']}, latest_msg_HP.predicted_areas);
                    scenario_v.dynamic_obstacle_area(end+1,:) = predicted_areas_HP;
                else
                    % if the latest message comes not from the current timestep 
                    status = false; % status indicates whether a new message is received
                    if ismember(veh_HP,coupling_vehs_same_grp_HP)
                        % if the selected vehicle is in the same group, define a timeout and wait for its message
                        timeout = 0.5;
                        [msg_received, status, ~] = receive(scenario_v.ros_subscribers{veh_HP}, timeout);
                        if status
                            predicted_areas_HP = arrayfun(@(array) {[array.x';array.y']}, msg_received.predicted_areas);
                            scenario_v.dynamic_obstacle_area(end+1,:) = predicted_areas_HP;
                        end
                    end

                    if ~ismember(veh_HP,coupling_vehs_same_grp_HP) || ~status
                        % if the selected vehicle is not in the same group or is in the same group but no new message is received in a certain time 
                        % interval, add the reachable sets as dynamic obstacles 
                        reachable_sets_HP = iter.reachable_sets(veh_HP,:);
                        % turn polyshape to plain array
                        reachable_sets_HP_array = cellfun(@(c) {[c.Vertices(:,1)';c.Vertices(:,2)']}, reachable_sets_HP);
                        scenario_v.dynamic_obstacle_reachableSets(end+1,:) = reachable_sets_HP_array;
                    end
                end
            end
%         
%                     % convexify the reachable sets since the separate axis theorem works only for convex shapes
%                     points_idx = cellfun(@(cell) {convhull(cell.Vertices,'Simplify',true)}, reachable_sets_HP);
%                     reachable_sets_conv_HP = cellfun(@(reachable_set, idx) {[reachable_set.Vertices(idx,1)';reachable_set.Vertices(idx,2)']}, reachable_sets_HP, points_idx);

            % Collision with coupling vehicles with lower priorities will be avoided by considered them as static obstacles 
            if right_of_way
                for i_LP = 1:length(all_coupling_vehs_LP)
                    veh_LP = all_coupling_vehs_LP(i_LP);
                    
                    veh = Vehicle();
                    % calculate the current occupied area 
                    [x_globals,y_globals] = translate_global(...
                        iter.x0(veh_LP, indices().heading),...              % yaw angle
                        iter.x0(veh_LP, indices().x),...                    % x-position
                        iter.x0(veh_LP, indices().y),...                    % y-position
                        [-1,-1, 1, 1]*(veh.Length/2+scenario_v.offset),...  % x-coordinates of the local occupied area 
                        [-1, 1, 1,-1]*(veh.Width/2+scenario_v.offset));     % y-coordinates of the local occupied area 
                    
                    % add as static obstacles
                    scenario_v.obstacles(end+1) = {[x_globals;y_globals]};
                end
            end
            
            subcontroller_timer = tic;

%             if scenario.k>=8 && vehicle_i==14
%                 graphs_visualization(belonging_vector,coupling_weights,'ShowWeights',true)
%                 disp('debug')
%             end

            % execute sub controller for 1-veh scenario
            [u_v,y_pred_v,info_v] = sub_controller(scenario_v, iter_v);
            

            % prepare output data
            info.tree{vehicle_i} = info_v.tree;
            info.tree_path(vehicle_i,:) = info_v.tree_path;
            info.subcontroller_runtime(vehicle_i) = toc(subcontroller_timer);
            info.n_expanded = info.n_expanded + info_v.tree.size();
            info.next_node = set_node(info.next_node, vehicle_i, info_v);
            info.shapes(vehicle_i,:) = info_v.shapes(:);
            info.vehicle_fullres_path(vehicle_i) = path_between(info_v.tree_path(1), info_v.tree_path(2), info_v.tree, scenario.mpa);
            info.trim_indices(vehicle_i) = info_v.trim_indices;
            info.predicted_trims(vehicle_i,:) = info_v.predicted_trims; % store the planned trims in the future Hp time steps
            info.trees{vehicle_i} = info_v.tree; % store tree information
            info.y_predicted{vehicle_i,1} = y_pred_v{:}; % store the information of the predicted output
            info.parl_groups_infos = parl_groups_infos;
            y_pred{vehicle_i,1} = y_pred_v{:};
            u(vehicle_i) = u_v(1);
        end
        
        % Communicate data to other vehicles
        % Trick: vehicles will wait for the last vehicle in the same computation level has planned 
        % to avoid receiving messages of the current time step from vehicles in the same computation level. 
        for idx_k = 1:length(vehs_level_i)
            vehicle_k = vehs_level_i(idx_k);

            states_current = info.y_predicted{vehicle_k}(1,:);

            predicted_trims = info.predicted_trims(vehicle_k,:); % including the current trim
            trim_current = predicted_trims(2);

            % get reference speed and path points
            predicted_vRef = get_max_speed(scenario.mpa, trim_current);
            
            % Find equidistant points on the reference trajectory.
            reference = sampleReferenceTrajectory(...
                Hp, ...                                                 % number of prediction steps
                scenario.vehicles(vehicle_k).referenceTrajectory, ...   % total reference path
                states_current(indices().x), ...                    % vehicle position x
                states_current(indices().y), ...                    % vehicle position y
                predicted_vRef*scenario.dt...                         % distance traveled in one timestep
            );

            ref_points_index = reshape(reference.ReferenceIndex,Hp,1);
            predicted_lanelets = get_predicted_lanelets(scenario.vehicles(vehicle_k), ref_points_index, scenario.road_raw_data.lanelet);
            
            predicted_areas = info.shapes(vehicle_k,:);

            % send message
            scenario.vehicles(vehicle_k).communicate.send_message(scenario.k, predicted_trims, predicted_lanelets, predicted_areas);
        end

    end

    subcontroller_time_all_grps = zeros(1,n_grps); % subcontroller time of each group
    % calculate the subcontroller time of each parallel group
    for grp_i = 1:n_grps
        vehs_in_grp_i = parl_groups_infos(grp_i).vertices;
        for level_j = 1:length(CL_based_hierarchy)
            vehs_in_level_j = CL_based_hierarchy(level_j).members;
            find_in_same_level = ismember(vehs_in_grp_i,vehs_in_level_j);
            vehs_in_same_level = vehs_in_grp_i(find_in_same_level);

            % take the maximum control time among vehicles in the same group and in the same computation level
            if ~isempty(vehs_in_same_level)
                subcontroller_time_all_grps(grp_i) = subcontroller_time_all_grps(grp_i) + max(info.subcontroller_runtime(vehs_in_same_level));
            end
        end
    end

    % the subcontroller time of the whole system in each time step depends on the maximum subcontroller time used by each parallel group
    max_subcontroller_time = max(subcontroller_time_all_grps);
    info.subcontroller_runtime_all_grps = subcontroller_time_all_grps;
end


%% backup
%             grp_idx = arrayfun(@(array) ismember(vehicle_i,array.vertices), parl_groups_infos);
%             all_vehs_same_grp = parl_groups_infos(grp_idx).vertices; % all vehicles in the same group
% 
%             % coupling vehicles with higher priorities in the same group
%             coupling_vehs_same_grp_HP = intersect(all_coupling_vehs_HP, all_vehs_same_grp);
%             % coupling vehicles with higher priorities in other parallel groups
%             coupled_with_other_grps_HP = setdiff(all_coupling_vehs_HP, coupling_vehs_same_grp_HP);
% 
%             % Filter out vehicles that are not adjacent
%             % create filter to keep self and coupling vehicles with higher priorities in the same group 
%             priority_filter_same_grp = false(1,nVeh);
%             priority_filter_same_grp(coupling_vehs_same_grp_HP) = true; % keep coupling vehicles in the same group with higher priority 
%             priority_filter_same_grp(vehicle_i) = true; % keep self
%             % create filter to keep coupling vehicles with higher priorities in other groups
%             priority_filter_other_grps = false(1,nVeh);
%             priority_filter_other_grps(coupled_with_other_grps_HP) = true; % keep coupling vehicles in other groups with higher priority
% 
%             scenario_filtered_same_grp = filter_scenario(scenario, priority_filter_same_grp);
%             iter_filtered_same_grp = filter_iter(iter, priority_filter_same_grp);
%             iter_filtered_other_grps = filter_iter(iter, priority_filter_other_grps);
% 
%             self_index = sum(priority_filter_same_grp(1:vehicle_i));        
%             v2o_filter = true(1,scenario_filtered_same_grp.nVeh);
%             v2o_filter(self_index) = false;
% 
%             % get the predicted areas of coupling vehicles with higher priorities in the same group
%             predicted_areas = cell(length(coupling_vehs_same_grp_HP), Hp);
%             for idx_j = 1:length(coupling_vehs_same_grp_HP)
%                 veh_i = coupling_vehs_same_grp_HP(idx_j);
%                 latest_msg_i = scenario.ros_subscribers{veh_i}.LatestMessage;
%                 predicted_areas_j = arrayfun(@(array) {[array.x';array.y']}, latest_msg_i.predicted_areas);
%                 % check the oldness of the latest message: the predicted
%                 % areas must be shifted if they are not sent in the current
%                 % time step. The last predicted area will be repeated such that the predicted area w  
%                 
%                 predicted_areas(idx_j,:) = predicted_areas_j;
%             end
% 
%             % add predicted trajecotries of the coupling vehicles with higher priorities in the same group as dynamic obstacles
%             [scenario_v, iter_v] = ...
%                 vehicles_as_dynamic_obstacles(scenario_filtered_same_grp, iter_filtered_same_grp, v2o_filter, predicted_areas);
% 
%             % add reachable sets of the coupling vehicles with higher priorities in other groups as dynamic obsticles
%             scenario_v = vehicles_reachable_sets_as_obstacles(scenario_v, iter_filtered_other_grps, coupled_with_other_grps_HP);
% 
%             % add all coupling vehicles with lower priorities as static obstacles
%             if right_of_way
%                 scenario_v = vehicles_as_static_obstacles(scenario_v, iter, all_coupling_vehs_LP);
%             end
