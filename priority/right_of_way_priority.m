classdef  right_of_way_priority < interface_priority
% right_of_way  Instance of interface_priority used for dynamic priority
% assignment. For vehicles driving consectively, vehicles in the front are
% aasigned higher priorities; For vehicles crossing the intersection, vehicles
% closer to the center of intersection are assigned with higher priorities. 
% Vehicles at intersection keep higher priority and do not change their relative priority until they leave intersection
    
    properties

    end
    
    methods 
        
        function obj = right_of_way_priority()
            obj.is_assign_unique_priority = false; % whether to asign unique priority
        end

        %% priority
        function [veh_at_intersection,groups,edge_to_break,directed_adjacency,priority_list] = priority(obj,scenario,iter)
            % assign priorities to vehicles
            nVeh = length(scenario.vehicles);
            Hp = size(iter.referenceTrajectoryPoints,2);
            intersection_center = [2.25, 2];
            lanelets_idx = zeros(nVeh, Hp);
            veh_at_intersection = [];
            directed_adjacency = zeros(nVeh,nVeh);
            % assign priorities to vehicles driving consecutively
            
            % semi_adjacency matrix
            semi_adjacency= scenario.semi_adjacency(:,:,end); 

            
            % update the undirected adjacency matrix to a directed adajacency matrix
            for nveh = 1: nVeh-1

                % check semi_adjacent vehicles (i.e. vehicles driving consecutively)
                veh_semi_adjacent = find(semi_adjacency(nveh,:));
                
                % check the vehicles whose index is larger than the current vehicle, no repeated check
                veh_semi_adjacent = veh_semi_adjacent(veh_semi_adjacent > nveh);
                
                for iveh = veh_semi_adjacent  
                    is_leading_vehicle = check_driving_order(scenario,iter, nveh, iveh);
                    if is_leading_vehicle
                        directed_adjacency(nveh, iveh) = 1;
                    else
                        directed_adjacency(iveh, nveh) = 1;            
                    end                  
                end         
            end
            
            % identify vehicles at intersection and assign priorities accordingly
            for veh_idx = 1:nVeh   
                lanelets_idx(veh_idx,:) = lanelets_index(veh_idx,iter,scenario);
                is_at_intersection = (sum(ismember(lanelets_idx(veh_idx,:), scenario.intersection_lanelets)) > 0);
                if is_at_intersection
                    veh_at_intersection = [veh_at_intersection,veh_idx];
                    n_veh_at_intersection = length(veh_at_intersection);
                end
 
            end
            
            adjacency= scenario.adjacency(:,:,end);
            
            % assign priorities to vehicles crossing the intersection
            % (i.e., at the intersection, but not driving consecutively)
            if ~isempty(veh_at_intersection)
                % if there were vehicles at intersection at last step, remained vehicles at intersection keep priorities as last step,
                % and assign lower priorities to new vehicles coming into intersection based on the distance to the intersection center

                [~, idx,~] = intersect(scenario.last_vehs_at_intersection,veh_at_intersection);
                remained_veh_at_intersection = scenario.last_vehs_at_intersection(sort(idx));

                % new incoming vehicles at intersection
                new_veh_at_intersection = setdiff(veh_at_intersection, remained_veh_at_intersection);

                % assign priorities to new vehicles at intersection based on their distance to the intersection center
                if ~isempty(new_veh_at_intersection)

                    first_referenceTrajectoryPoints = [iter.referenceTrajectoryPoints(new_veh_at_intersection,1,1),iter.referenceTrajectoryPoints(new_veh_at_intersection,1,2)];
                    diff = first_referenceTrajectoryPoints - intersection_center;
                    distance_to_center = sqrt(diff(:,1).^2 + diff(:,2).^2);
                    [~, index] = sort(distance_to_center,'ascend');

                    % sort the vehicles at intersection based on their distance to the intersection center
                    % and assign higher priority to the vehicle closer to the intersection center
                    new_veh_at_intersection = new_veh_at_intersection(index);
                    veh_at_intersection = [remained_veh_at_intersection,new_veh_at_intersection];
                else                      
                    veh_at_intersection = remained_veh_at_intersection;
                end
       
                if n_veh_at_intersection > 1
                    
                    for veh_idx = 1:n_veh_at_intersection - 1 
                        veh_higher_prior = veh_at_intersection(veh_idx);
                        veh_adjacent = find(adjacency(veh_higher_prior,:));
                        veh_semi_adjacent = find(semi_adjacency(veh_higher_prior,:));

                        % find the vehicles that do not drive consecutively
                        veh_adjacent_diff = setdiff(veh_adjacent, veh_semi_adjacent);

                        for veh_adj_idx = (veh_idx+1):n_veh_at_intersection
                            veh_lower_prior = veh_at_intersection(veh_adj_idx);
                            if ismember(veh_lower_prior,veh_adjacent_diff)
                                directed_adjacency(veh_higher_prior,veh_lower_prior) = 1;
%                                 directed_adjacency(veh_lower_prior,veh_higher_prior) = 0;
                            end 
                        end       
                    end
                end
   
            end
               
            % Convert the Matrix "directed_adjacency" to a directed Graph
            Graph = digraph(directed_adjacency);
            
            %check if there is any cycles in the directed graph
            cycles = allcycles(Graph);
            
            
            % Convert directed cyclic graph to directed acyclic graph
            edge_to_break = {};
            while ~isempty(cycles)
                % break the cycle between vehicles which have the largest distance 
                cyclic_vehicles = cycles{1};
                n_cyclic_vehicles = length(cyclic_vehicles);  
                distance = 0;
                
                for vehi = 1:n_cyclic_vehicles
                    if vehi < n_cyclic_vehicles
                        distance_i = check_distance(iter, cyclic_vehicles(vehi),cyclic_vehicles(vehi+1));
                    else
                        distance_i = check_distance(iter, cyclic_vehicles(vehi),cyclic_vehicles(1));
                    end
                    
                    if distance_i > distance
                        distance = distance_i;
                        veh_to_break = vehi;
                    end
                    
                end
                
                if veh_to_break < n_cyclic_vehicles
                    i = cyclic_vehicles(veh_to_break);
                    j = cyclic_vehicles(veh_to_break + 1);
                else
                    i = cyclic_vehicles(veh_to_break);
                    j = cyclic_vehicles(1);
                    
                end
                edge_to_break{end+1} = [i,j];

                directed_adjacency(i,j) = 0;
                % construct a new graph and check the cycles in the graph
                Graph = digraph(directed_adjacency);
                cycles = allcycles(Graph);
            end
            
           % calculate computation levels using kahn algorithm(topological ordering)
            [~, L] = kahn(directed_adjacency);
            computation_levels = size(L,1);
%                 % visualize the directed graph  
%                 figure(); plot(Graph,'LineWidth',1) 


            % construct the priority groups
            groups = struct;
            for group_idx = 1:computation_levels
                groups(group_idx).members = find(L(group_idx,:));
                if group_idx == 1
                    groups(group_idx).predecessors = [];
                else
                    groups(group_idx).predecessors = [groups(group_idx-1).predecessors groups(group_idx-1).members];
                end
            end 

            % Assign prrority according to computation level
            % Vehicles with higher priorities plan trajectory before vehicles
            % with lower priorities            
            priority_list = obj.get_priority(groups,obj.is_assign_unique_priority);

        end

        %% priority_parl
        function [scenario,CL_based_hierarchy,lanelet_intersecting_areas] = priority_parl(obj, scenario, iter)
            % assign priorities to vehicles when parallel computation is used
%             scenario = scenario;

            % update the coupling information
%             [veh_at_intersection, coupling_weights, coupling_info] = get_coupling_info(scenario, iter);
%             [vehs_at_intersection, coupling_weights, coupling_info, time_enter_intersection] = get_coupling_info_version2(scenario, iter);
%             [veh_at_intersection, coupling_weights, coupling_info, time_enter_intersection] = get_coupling_info_version3(scenario, iter);

            traffic_info = TrafficInfo(scenario, iter);

            coupling_weights = traffic_info.coupling_weights;
            coupling_weights_origin = coupling_weights; % make a copy
            coupling_info = traffic_info.coupling_info;
            
            % Strategy to let vehicle without the right-of-way enter the intersecting area
            % Ignore coupling edge if not allowed to enter the intersecting area because no collision is possible anymore
            [coupling_weights,lanelet_intersecting_areas,coupling_info] = ...
                obj.strategy_enter_intersecting_area(iter,coupling_info,coupling_weights,scenario.strategy_enter_intersecting_area,scenario.nVeh);

            [coupling_weights,coupling_info] = obj.check_and_break_circle(coupling_weights,coupling_weights_origin,coupling_info);

            % visualize the directed graph  
%             figure(); plot(Graph,'LineWidth',1,'EdgeLabel',round(Graph.Edges.Weight,2))

%             if isempty(scenario.parl_groups_info)
%                 parl_groups_old = {};
%             else
%                 parl_groups_old = {scenario.parl_groups_info.vertices};
%             end
            % form parallel groups
            [CL_based_hierarchy, parl_groups_info, belonging_vector] = form_parallel_groups(coupling_weights, scenario.max_num_CLs, coupling_info, 'method', 's-t-cut');

            % Assign prrority according to computation level
            % Vehicles with higher priorities plan trajectory before vehicles
            % with lower priorities
            priority_list = obj.get_priority(CL_based_hierarchy,obj.is_assign_unique_priority);

            % update properties of scenario 
            scenario.coupling_weights = coupling_weights;
            directed_adjacency = (coupling_weights ~= 0);
            scenario.directed_coupling = directed_adjacency;
            scenario.coupling_info = coupling_info;
            scenario.belonging_vector = belonging_vector;
            scenario.priority_list = priority_list;
            scenario.time_enter_intersection = traffic_info.time_enter_intersection;
            scenario.last_vehs_at_intersection = traffic_info.vehs_at_intersection;
            scenario.parl_groups_info = parl_groups_info;
            
            % visualize the coupling between vehicles
%             plot_coupling_lines(coupling_weights, iter.x0, belonging_vector, 'ShowWeights', true)
        end
    end

    methods (Access = private, Static)
        function [coupling_weights,lanelet_intersecting_areas,coupling_info] = strategy_enter_intersecting_area(iter,coupling_info,coupling_weights,strategy_enter_intersecting_area,nVeh)
            % This function implement the strategies of letting vehicle enter
            % the intersecting area, which is the overlapping area of two
            % vehicles' lanelet boundaries. Four strategies are existed.

            lanelet_intersecting_areas = cell(nVeh,1);
            predicted_lanelet_boundary = iter.predicted_lanelet_boundary(:,3);

            for i = 1:length([coupling_info.veh_with_ROW])
                coupling_info(i).is_ignored = false; % whether the coupling is ignored
                veh_with_ROW = coupling_info(i).veh_with_ROW;
                veh_without_ROW = coupling_info(i).veh_without_ROW;
                is_at_intersection = coupling_info(i).is_at_intersection;
                is_side_impact_collision = strcmp(coupling_info(i).collision_type, CollisionType.type_2);
                is_rear_end_collision = strcmp(coupling_info(i).collision_type, CollisionType.type_1);
                is_intersecting_lanelets = strcmp(coupling_info(i).lanelet_relationship, LaneletRelationshipType.type_5);
                is_merging_lanelets = strcmp(coupling_info(i).lanelet_relationship, LaneletRelationshipType.type_3);

                % check if coupling edge should be ignored
                switch strategy_enter_intersecting_area
                    case '1'
                        % no constraint on entering the intersecting area
                        return
                    case '2'
                        % not allowed to enter the intersecting area if they are coupled at intersecting lanelets of the intersection
                        is_ignore_coupling = is_intersecting_lanelets && is_at_intersection;
                    case '3'
                        % not allowed to enter the intersecting area if they are coupled at intersecting or merging lanelets of the intersection
                        is_ignore_coupling = (is_intersecting_lanelets || is_merging_lanelets) && is_at_intersection;
                    case '4'
                        % not allowed to enter the intersecting area if they are coupled at intersecting or merging lanelets regardless whether they are at the intersection or not
                        is_ignore_coupling = is_intersecting_lanelets || is_merging_lanelets;
                    otherwise
                        warning("Please specify one of the following strategies to let vehicle enter intersecting area: '0', '1', '2', '3'.")
                        return
                end

                if is_ignore_coupling
                    % check if vehicle without right-of-way has already enter the intersecting area
                    
                    % get the intersecting area of two vehicles' lanelet
                    lanelet_intersecting_area = intersect(predicted_lanelet_boundary{veh_without_ROW}, iter.predicted_lanelet_boundary{veh_with_ROW,3});
                    if lanelet_intersecting_area.NumRegions <= 1
                        [x_lanelet_intersecting_area, y_lanelet_intersecting_area] = boundary(lanelet_intersecting_area);
                        lanelet_intersecting_area = [x_lanelet_intersecting_area';y_lanelet_intersecting_area'];
                    else
                        warning(['The intersecting area has unexpectedly multiple ' num2str(lanelet_intersecting_area.NumRegions) ' regions.'])
                        lanelet_intersecting_area = [];
                    end
                    
                    % if vehicle without the right-of-way cannot avoid entering the
                    % intersecting area (emergency breaking meneuver), the coupling is not allowed to be deleted anymore
                    if InterX(lanelet_intersecting_area, iter.emergency_braking_maneuvers{veh_without_ROW}.area_without_offset)
                        % vehicle without right-of-way has already entered
                        % the intersecting area: coupling cannot be ignored 
                        disp(['Vehicle ' num2str(veh_without_ROW) ' has already entered the intersecting area, thus the coupling with vehicle ' num2str(veh_with_ROW) ' cannot be ignored.'])
                        continue
                    else
                        % Vehicle without right-of-way has not entered
                        %  1. side-impact collision: not allowed to enter
                        %  2. rear-end collision: allowed to enter
                        if is_rear_end_collision
                            disp(['Vehicle ' num2str(veh_without_ROW) ' is allowed to enter the intersecting area since it has rear-end (not side-impact) collision possibility with vehicle ' num2str(veh_with_ROW) '.'])
                            continue
                        elseif is_side_impact_collision
                            % ignore coupling 
%                             disp(['Coupling from vehicle ' num2str(veh_with_ROW) ' to ' num2str(veh_without_ROW) ' is ignored by forbidding the latter to enter this area.'])
                            coupling_info(i).is_ignored = true;
                            coupling_weights(veh_with_ROW,veh_without_ROW) = 0;
                            % store intersecting area for later use
                            if ~isempty(lanelet_intersecting_area)
                                lanelet_intersecting_areas{veh_without_ROW}(end+1) = {lanelet_intersecting_area};
                            end
    
                            % subtract the intersecting area from vehicle's lanelet boundary 
                            predicted_lanelet_boundary{veh_without_ROW} = subtract(predicted_lanelet_boundary{veh_without_ROW}, iter.predicted_lanelet_boundary{veh_with_ROW,3});
            
                            num_regions = predicted_lanelet_boundary{veh_without_ROW}.NumRegions;
                            if num_regions > 1
                                % if multiple regions, only keep the one that is closest to vehicle
                                poly_sort = sortregions(predicted_lanelet_boundary{veh_without_ROW},'centroid','descend','ReferencePoint',[iter.x0(indices().x),iter.x0(indices().y)]); 
                                predicted_lanelet_boundary{veh_without_ROW} = rmboundary(poly_sort, 2:num_regions);
                            end
                        else
                            error('Unknown collision type')
                        end
                    end
                end
            end

        end


        function [coupling_weights,coupling_info] = check_and_break_circle(coupling_weights,coupling_weights_origin,coupling_info)
            % This function break coupling circle if exist any:
            % 1. If possible, coupling edge between vehicles at the
            % intersection will not be broke
            % 2. If possible, alway break coupling eage with a lower weight 

            edge_to_invert = {};
            % directed Graph
            Graph = digraph(coupling_weights);
            
            %check if there is any cycles in the directed graph
            [~, edges] = allcycles(Graph);

            % break the coupling edge with the lowest weight until no circle exists
            while ~isempty(edges)
                % find the edge with lowest weight
                edges_all = unique([edges{:}],'stable');
                [~,edge_sorted] = sort(Graph.Edges.Weight(edges_all),'ascend');

                count = 1; is_stop = false;
                while true
                    % if possible, do not break the edge of two vehicles at the intersection
                    edge_target = edge_sorted(count);
                    vertex_start = Graph.Edges.EndNodes(edge_target,1);
                    vertex_end = Graph.Edges.EndNodes(edge_target,2);
                    if (~ismember(vertex_start,vehs_at_intersection) && ~ismember(vertex_end,vehs_at_intersection)) || is_stop
                        break
                    end
                    count = count + 1;
                    if count > length(edge_sorted)
                        % if not possible, break the one with lowerst weight
                        count = 1;
                        is_stop = true;
                    end
                end
                
                edge_to_invert{end+1} = [vertex_start,vertex_end];
                coupling_weights(vertex_start,vertex_end) = 0; % break the coupling edge
                
                Graph = digraph(coupling_weights);
                [~, edges] = allcycles(Graph);
            end

            % recover the coupling edge by comparing their computation levels
            [valid,L] = kahn(coupling_weights); % calculate computation levels using kahn algorithm(topological ordering)
            assert(valid==true)

            for i = 1:length(edge_to_invert)
                vertex_start = edge_to_invert{i}(1);
                vertex_end = edge_to_invert{i}(2);
                level_start = find(L(:,vertex_start)~=0);
                level_end = find(L(:,vertex_end)~=0);
                if level_start > level_end
                    % the broken edge is recovered but the direction is inverted
                    coupling_weights(vertex_end,vertex_start) = coupling_weights_origin(vertex_start,vertex_end);

                    % swap leader and follower
                    find_vertex_start = find([coupling_info.veh_with_ROW]==vertex_start);
                    find_vertex_end = find([coupling_info.veh_without_ROW]==vertex_end);
                    coupling_i = intersect(find_vertex_start,find_vertex_end);
                    coupling_info(coupling_i).veh_with_ROW = vertex_end;
                    coupling_info(coupling_i).veh_without_ROW = vertex_start;
                    [coupling_info(coupling_i).veh_with_ROW, coupling_info(coupling_i).veh_without_ROW] = ...
                        swap(coupling_info(coupling_i).veh_with_ROW, coupling_info(coupling_i).veh_without_ROW);
                    disp(['Edge from ' num2str(vertex_start) ' to ' num2str(vertex_end) ' is inverted.'])
                else
                    % the broken edge is recovered and the direction is maintained
                    coupling_weights(vertex_start,vertex_end) = coupling_weights_origin(vertex_start,vertex_end);
                    disp(['Edge from ' num2str(vertex_start) ' to ' num2str(vertex_end) ' is not inverted.'])
                end
            end

            % Assign prrority according to computation level
            % Vehicles with higher priorities plan trajectory before vehicles
            % with lower priorities
            priority_list = zeros(1,scenario.nVeh);
            prio = 1;
            for level_i = 1:length(CL_based_hierarchy)
                vehs_in_level_i = CL_based_hierarchy(level_i).members; % vehicles in the selected computation level
                for veh_i = vehs_in_level_i
                    priority_list(veh_i) = prio; % assign unique priority
                    prio = prio + 1;
                end
            end

            % visualize the coupling between vehicles
%             plot_coupling_lines(coupling_weights, iter.x0, belonging_vector, 'ShowWeights', true)
        end




    end
  
end

