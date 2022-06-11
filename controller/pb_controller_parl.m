function [u, y_pred, info, scenario] = pb_controller_parl(scenario, iter)
% PB_CONTROLLER_PARL Plan trajectory for one time step using a
% priority-based controller. Vehicles inside one group plan in sequence and
% between groups plan in pararllel. Controller simulates multiple
% distributed controllers in a for-loop. 


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
        [CL_based_hierarchy, parl_groups_infos, edge_to_break, coupling_weights,...
            belonging_vector, veh_at_intersection, priority_list, coupling_infos, time_enter_intersection] = priority_parl(obj);
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

    % currently 
    nVeh = scenario.nVeh;
    Hp = scenario.Hp;

    % update properties of scenario 
    scenario.coupling_weights = coupling_weights;
    directed_adjacency = (scenario.coupling_weights ~= 0);
    scenario.directed_coupling = directed_adjacency;
    scenario.coupling_infos = coupling_infos;
    scenario.belonging_vector = belonging_vector;
    scenario.priority_list = priority_list;
    scenario.time_enter_intersection = time_enter_intersection;
    scenario.last_veh_at_intersection = veh_at_intersection;

    % visualize the coupling between vehicles
    % plot_coupling_lines(coupling_weights, iter.x0, belonging_vector, 'ShowWeights', true)
    
    y_pred = cell(nVeh,1);
    u = zeros(nVeh,1);
    
    info = struct;
    info.vehicle_fullres_path = cell(nVeh,1);
    info.trim_indices = (-1)*ones(nVeh,1);
    info.subcontroller_runtime = zeros(nVeh,1);
    info.shapes = cell(nVeh,Hp);
    info.next_node = node(-1, zeros(nVeh,1), zeros(nVeh,1), zeros(nVeh,1), zeros(nVeh,1), -1, -1);
    info.n_expanded = 0;
    info.computation_levels = length(CL_based_hierarchy);
    info.edge_to_break = edge_to_break;
    info.priority_list = priority_list;
    
    % graph-search to select the optimal motion primitive
    sub_controller = @(scenario, iter)...
        graph_search(scenario, iter); 
    
    for level_j = 1:length(CL_based_hierarchy)
        vehs_level_i = CL_based_hierarchy(level_j).members; % vehicles of all groups in the same computation level
        predecessors = CL_based_hierarchy(level_j).predecessors; % vehicles of all groups in the previous computation level
        
        for vehicle_i = vehs_level_i
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
                
                if ismember(veh_HP,coupling_vehs_same_grp_HP)
                    % if in the same group, read the current message and
                    % set the predicted occupied areas as dynamic obstacles  
                    latest_msg = read_message(scenario_v.vehicles.communicate, scenario_v.ros_subscribers{veh_HP}, scenario_v.k);
                    predicted_areas_HP = arrayfun(@(array) {[array.x';array.y']}, latest_msg.predicted_areas);
                    oldness_msg = scenario_v.k - latest_msg.time_step;
                    if oldness_msg ~= 0
                        % consider the oldness of the message: delete the
                        % first n entries and repeat the last entry for n times
                        predicted_areas_HP = del_first_rpt_last(predicted_areas_HP,oldness_msg);
                    end
                    scenario_v.dynamic_obstacle_area(end+1,:) = predicted_areas_HP;
                else
                    % if the selected vehicle is not in the same group, add their reachable sets as dynamic obstacles 
                    reachable_sets_HP = iter.reachable_sets(veh_HP,:);
                    % turn polyshape to plain array
                    reachable_sets_HP_array = cellfun(@(c) {[c.Vertices(:,1)';c.Vertices(:,2)']}, reachable_sets_HP);
                    scenario_v.dynamic_obstacle_reachableSets(end+1,:) = reachable_sets_HP_array;
                end
            end

            if ~scenario_v.is_allow_enter_crossing_area && ismember(vehicle_i,veh_at_intersection)
                % if follower is not allowed to enter the crossing areas before their leaders leave those areas at the intersection 
                find_leaders = find([scenario_v.coupling_infos.idFollower]==vehicle_i);
                % subtract the crossing area from lower priority vehicle's lanelet boundary if it is not allowed to enter this area 
                for i_coupling = find_leaders
                    id_leader = scenario_v.coupling_infos(i_coupling).idLeader;
                    if ismember(id_leader,veh_at_intersection) && strcmp(scenario_v.coupling_infos(i_coupling).type, CollisionType.type_2)
                        % both vehicles are at the intersection and they do not drive successively (side-impact collision) 
                        lanelet_crossing_area = intersect(iter_v.predicted_lanelet_boundary{3}, iter.predicted_lanelet_boundary{id_leader,3});
                        if lanelet_crossing_area.NumRegions == 1
                            scenario_v.lanelet_crossing_areas{end+1} = [lanelet_crossing_area.Vertices(:,1)';lanelet_crossing_area.Vertices(:,2)'];
                        elseif lanelet_crossing_area.NumRegions == 2
                            warning(['There are unexpectedly ' num2str(lanelet_crossing_area.NumRegions) ' regions.'])
                        end
                        % subtract the crossing area from lower priority vehicle's lanelet boundary 
                        iter_v.predicted_lanelet_boundary{3} = subtract(iter_v.predicted_lanelet_boundary{3}, iter.predicted_lanelet_boundary{id_leader,3});

                        num_regions = iter_v.predicted_lanelet_boundary{3}.NumRegions;
                        if num_regions > 1
                            % if multiple regions, only keep the one that is closest to vehicle
                            poly_sort = sortregions(iter_v.predicted_lanelet_boundary{3},'centroid','descend','ReferencePoint',[iter_v.x0(indices().x),iter_v.x0(indices().y)]); 
                            iter_v.predicted_lanelet_boundary{3} = rmboundary(poly_sort, 2:num_regions);
                        end
                    end
                end
            end

            % set lanelet boundary as static obstacle
%             scenario_v.obstacles{end+1} = [iter_v.predicted_lanelet_boundary{3}.Vertices(:,1)'; iter_v.predicted_lanelet_boundary{3}.Vertices(:,2)'];

            % coupling vehicles with lower priorities
            scenario_v = consideration_of_followers_by_leader(scenario_v, iter, all_coupling_vehs_LP);

            subcontroller_timer = tic;

%             if scenario.k>=2
% %                 if scenario_v.vehicles.ID==16
%                     disp('debug')
%                     plot_obstacles(scenario_v)
%                     pause(0.5)
% %                 end
%             end

            % execute sub controller for 1-veh scenario
            [u_v,y_pred_v,info_v] = sub_controller(scenario_v, iter_v);
            
            if scenario.k>=2
%                 plot_obstacles(scenario_v)
%                 pause(0.5)
%                 plot_obstacles(info_v.shapes)
%                 pause(0.5)
%                 graphs_visualization(belonging_vector, coupling_weights, 'ShowWeights', true)
            end
            
            % prepare output data
            info.tree{vehicle_i} = info_v.tree;
            info.tree_path(vehicle_i,:) = info_v.tree_path;
            info.subcontroller_runtime(vehicle_i) = toc(subcontroller_timer);
            info.n_expanded = info.n_expanded + info_v.tree.size();
            info.next_node = set_node(info.next_node, vehicle_i, info_v);
            info.shapes(vehicle_i,:) = info_v.shapes(:);
            info.vehicle_fullres_path(vehicle_i) = path_between(info_v.tree_path(1), info_v.tree_path(2), info_v.tree, scenario);
            info.trim_indices(vehicle_i) = info_v.trim_indices;
            info.predicted_trims(vehicle_i,:) = info_v.predicted_trims; % store the planned trims in the future Hp time steps
            info.trees{vehicle_i} = info_v.tree; % store tree information
            info.y_predicted{vehicle_i,1} = y_pred_v{:}; % store the information of the predicted output
            y_pred{vehicle_i,1} = y_pred_v{:};
            u(vehicle_i) = u_v(1);
        end
        
        % Communicate data to other vehicles
        % Trick: vehicles will wait for the last vehicle in the same computation level has planned 
        % to avoid receiving messages of the current time step from vehicles in the same computation level.         
        for vehicle_k = vehs_level_i
            predicted_trims = info.predicted_trims(vehicle_k,:); % including the current trim
            
            trim_current = predicted_trims(2);

            states_current = info.y_predicted{vehicle_k}(1,:);
            x0 = states_current(indices().x);
            y0 = states_current(indices().y);

            [predicted_lanelets,~,~] = get_predicted_lanelets(scenario.vehicles(vehicle_k), trim_current, x0, y0, scenario.mpa, scenario.dt);
            predicted_areas = info.shapes(vehicle_k,:);

            % send message
            send_message(scenario.vehicles(vehicle_k).communicate, scenario.k, predicted_trims, predicted_lanelets, predicted_areas);
%             pause(0.1)
        end

    end

    n_grps = length(parl_groups_infos); % number of parallel groups

    % % calculate the total run time: only one vehicle in each computation
    % level will be counted, this is the one with the maximum run time for each parallel group 
    run_time_total_all_grps = zeros(1,n_grps); % subcontroller time of each group

    for grp_i = 1:n_grps
        vehs_in_grp_i = parl_groups_infos(grp_i).vertices;
        for level_j = 1:length(CL_based_hierarchy)
            vehs_in_level_j = CL_based_hierarchy(level_j).members;
            find_in_same_level = ismember(vehs_in_grp_i,vehs_in_level_j);
            vehs_in_same_level = vehs_in_grp_i(find_in_same_level);

            % take the maximum run time among vehicles in the same group and in the same computation level
            if ~isempty(vehs_in_same_level)
                run_time_total_all_grps(grp_i) = run_time_total_all_grps(grp_i) + max(info.subcontroller_runtime(vehs_in_same_level));
            end
        end
    end

    % the subcontroller time of the whole system in each time step depends on the maximum subcontroller time used by each parallel group
    info.subcontroller_runtime_all_grps = run_time_total_all_grps;
    info.subcontroller_run_time_total = max(run_time_total_all_grps);
end

