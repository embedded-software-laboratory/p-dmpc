function [scenario,CL_based_hierarchy,lanelet_intersecting_areas] = priority_assignment_parl(scenario, iter)
    % assign priorities to vehicles when parallel computation is used
    right_of_way = false;
    switch scenario.priority_option
        case 'topo_priority' 
            [CL_based_hierarchy, directed_adjacency, priority_list] = topo_priority().priority(scenario); 
            vehs_at_intersection = [];
        
        case 'right_of_way_priority'
            right_of_way = true;

        case 'constant_priority' 
            [CL_based_hierarchy, directed_adjacency, priority_list] = constant_priority().priority(scenario); 
            vehs_at_intersection = [];
        
        case 'random_priority'  
            [CL_based_hierarchy, directed_adjacency, priority_list, parl_groups_info] = random_priority().priority(scenario); 
            vehs_at_intersection = [];

        case 'FCA_priority' 
            [vehs_at_intersection, CL_based_hierarchy, directed_adjacency, priority_list] = FCA_priority().priority(scenario,iter);
    
        case 'mixed_traffic_priority'
            obj = mixed_traffic_priority(scenario);
            [CL_based_hierarchy, directed_adjacency] = obj.priority();
            vehs_at_intersection = [];
    end

    if strcmp(scenario.priority_option,'right_of_way_priority') || strcmp(scenario.priority_option,'mixed_traffic_priority')
        traffic_info = TrafficInfo(scenario, iter);

        coupling_weights = traffic_info.coupling_weights;
        coupling_weights_origin = coupling_weights; % make a copy
        coupling_info = traffic_info.coupling_info;
        
        if right_of_way
            % Strategy to let vehicle without the right-of-way enter the intersecting area
            % Ignore coupling edge if not allowed to enter the intersecting area because no collision is possible anymore
            [coupling_weights,lanelet_intersecting_areas,coupling_info] = ...
                strategy_enter_intersecting_area(iter,coupling_info,coupling_weights,scenario.strategy_enter_intersecting_area,scenario.nVeh);
        end

        [coupling_weights,coupling_info] = check_and_break_circle(coupling_weights,coupling_weights_origin,coupling_info,traffic_info.vehs_at_intersection);

        % form parallel CL_based_hierarchy
        [CL_based_hierarchy, parl_groups_info, belonging_vector] = form_parallel_groups(coupling_weights, scenario.max_num_CLs, coupling_info, 'method', 's-t-cut');

        % Assign prrority according to computation level
        % Vehicles with higher priorities plan trajectory before vehicles
        % with lower priorities
        priority_list = right_of_way_priority().get_priority(CL_based_hierarchy);

        
        scenario.coupling_weights = coupling_weights;
        scenario.time_enter_intersection = traffic_info.time_enter_intersection;
        scenario.parl_groups_info = parl_groups_info;
        scenario.belonging_vector = belonging_vector;
        scenario.coupling_info = coupling_info;
        directed_adjacency = (coupling_weights ~= 0);
        vehs_at_intersection = traffic_info.vehs_at_intersection;
    end 

    % update properties of scenario 
    scenario.directed_coupling = directed_adjacency;
    scenario.priority_list = priority_list;
    scenario.last_vehs_at_intersection = vehs_at_intersection;
    % visualize the coupling between vehicles
%             plot_coupling_lines(coupling_weights, iter.x0, belonging_vector, 'ShowWeights', true)
    % visualize the directed graph  
%             figure(); plot(Graph,'LineWidth',1,'EdgeLabel',round(Graph.Edges.Weight,2))
end

%% local function
function [coupling_weights,lanelet_intersecting_areas,coupling_info] = strategy_enter_intersecting_area(iter,coupling_info,coupling_weights,strategy_enter_intersecting_area,nVeh)
    % This function implement the strategies of letting vehicle enter
    % the intersecting area, which is the overlapping area of two
    % vehicles' lanelet boundaries. Four strategies are existed.

    lanelet_intersecting_areas = cell(nVeh,1);
    predicted_lanelet_boundary = iter.predicted_lanelet_boundary(:,3);

    for i = 1:length([coupling_info.veh_with_ROW])
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
                % disp(['Vehicle ' num2str(veh_without_ROW) ' has already entered the intersecting area, thus the coupling with vehicle ' num2str(veh_with_ROW) ' cannot be ignored.'])
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
                    coupling_info(i).is_ignored = true; % ignore coupling since no collision if possible anymore
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


%% local function
function [coupling_weights,coupling_info] = check_and_break_circle(coupling_weights,coupling_weights_origin,coupling_info,vehs_at_intersection)
    % This function break coupling circle if exist any:
    % 1. If possible, coupling edge between vehicles at the
    % intersection will not be broke
    % 2. If possible, alway break coupling eage with a lower weight 

    edge_to_invert = {};
    % directed Graph
    Graph = digraph(coupling_weights);
    
    if ~isdag(Graph)
        % get cycles in the directed graph
        [~, edges] = all_elem_cycles(Graph);
        assert(~isempty(edges))
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
            [~, edges] = all_elem_cycles(Graph);
        end
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

end 

