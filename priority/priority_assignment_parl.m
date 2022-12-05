function [scenario,iter,CL_based_hierarchy,lanelet_crossing_areas] = priority_assignment_parl(scenario, iter)
    % assign priorities to vehicles when parallel computation is used
    lanelet_crossing_areas = [];
    
    determine_couplings_timer = tic;

    % TODO: strategy here (use non-parallel assignment here)
    % adapt algorithms such that they return all needed data like here
    [vehs_at_intersection,coupling_weights,iter.coupling_weights_optimal,coupling_info,time_enter_intersection] = STAC_priority().priority(scenario,iter);

    if any(strcmp(scenario.options.priority,{'STAC_priority','FCA_priority','constant_priority','random_priority','coloring_priority'}))
        % Strategy to let vehicle without the right-of-way enter the crossing area
        % Ignore coupling edge if not allowed to enter the crossing area because no collision is possible anymore
        [coupling_weights_reduced,lanelet_crossing_areas,coupling_info,iter] = ...
            strategy_enter_lanelet_crossing_area(iter,coupling_info,coupling_weights,scenario.options.strategy_enter_lanelet_crossing_area,scenario.options.amount);

        [coupling_weights_reduced,coupling_info] = check_and_break_circle(coupling_weights_reduced,coupling_weights,coupling_info,vehs_at_intersection);
        iter.timer.determine_couplings = toc(determine_couplings_timer);

        group_vehs = tic;
        % form parallel CL_based_hierarchy
        method = 's-t-cut'; % 's-t-cut' or 'MILP'
        [CL_based_hierarchy, parl_groups_info, belonging_vector] = form_parallel_groups(coupling_weights_reduced, scenario.options.max_num_CLs, coupling_info, method, scenario.options);
        iter.timer.group_vehs = toc(group_vehs);
        
    elseif strcmp(scenario.options.priority,'mixed_traffic_priority')
        iter.timer.determine_couplings = toc(determine_couplings_timer);
        obj = mixed_traffic_priority(scenario,iter);
        [CL_based_hierarchy, directed_adjacency] = obj.priority();
        indexVehicleExpertMode = 0;
        for j = 1:scenario.options.amount
            if ((scenario.vehicle_ids(j) == scenario.manual_vehicle_id && scenario.options.firstManualVehicleMode == 2) ...
                || (scenario.vehicle_ids(j) == scenario.second_manual_vehicle_id && scenario.options.secondManualVehicleMode == 2))
                indexVehicleExpertMode = j;
            end
        end

        % [coupling_weights,coupling_info] = check_and_break_circle(coupling_weights,coupling_weights_origin,coupling_info);
        coupling_weights_copy = coupling_weights;
        
        % set coupling for group forming to 0 to ensure that only the vehicle in Expert-Mode is in the highest group
        for i = 1:length(coupling_weights_copy)
            if coupling_weights_copy(indexVehicleExpertMode, i) ~= 0
                coupling_weights_copy(indexVehicleExpertMode, i) = 0;
            end
        end
        method = 's-t-cut'; % 's-t-cut' or 'MILP'
        [CL_based_hierarchy, parl_groups_info, belonging_vector] = form_parallel_groups(coupling_weights_copy, scenario.options.max_num_CLs, coupling_info, method, options);
        % Assign prrority according to computation level
        % Vehicles with higher priorities plan trajectory before vehicles with lower priorities
    else
        warning([char(scenario.options.priority) ' is not supported if parallel computation is used. STAC_priority will be used instead.'])
    end
    
    % get priority list
    priority_list = STAC_priority().get_priority(CL_based_hierarchy);

    iter.coupling_weights = coupling_weights;
    iter.coupling_weights_reduced = coupling_weights_reduced;
    iter.time_enter_intersection = time_enter_intersection;
    iter.coupling_info = coupling_info;

    % update properties of scenario 
    iter.parl_groups_info = parl_groups_info;
    iter.belonging_vector = belonging_vector;
    iter.directed_coupling = (coupling_weights ~= 0);
    iter.directed_coupling_reduced = (coupling_weights_reduced ~= 0);
    iter.priority_list = priority_list;
    iter.last_vehs_at_intersection = vehs_at_intersection;

    % visualize the coupling between vehicles
    % plot_coupling_lines(coupling_weights, iter.x0, belonging_vector, 'ShowWeights', true)
    % visualize the directed graph  
    % figure(); plot(digraph(coupling_weights),'LineWidth',1)
end

%% local function
function [coupling_weights_reduced,lanelet_crossing_areas,coupling_info,iter] = strategy_enter_lanelet_crossing_area(iter,coupling_info,coupling_weights,strategy_enter_lanelet_crossing_area,nVeh)
    % This function implement the strategies of letting vehicle enter the crossing area, which is the overlapping area of two
    % vehicles' lanelet boundaries. Four strategies are existed.

    % Reduced coupling weights after forbidding vehicles entering their lanelet crossing areas
    coupling_weights_reduced = coupling_weights;

    lanelet_crossing_areas = cell(nVeh,1);

    for i = 1:length([coupling_info.veh_with_ROW])
        % rear-end collision at the same lanelet or successive lanelets
        % cannot be avoid by this strategy
        is_rear_end_collision = strcmp(coupling_info(i).collision_type, CollisionType.type_1);
        
        % only side-impact collision should be ignore by this strategy
        if is_rear_end_collision
            continue
        end

        veh_with_ROW = coupling_info(i).veh_with_ROW;
        veh_without_ROW = coupling_info(i).veh_without_ROW;

        is_at_intersection = coupling_info(i).is_at_intersection;
        is_intersecting_lanelets = strcmp(coupling_info(i).lanelet_relationship, LaneletRelationshipType.type_5);
        is_merging_lanelets = strcmp(coupling_info(i).lanelet_relationship, LaneletRelationshipType.type_3);

        % check if coupling edge should be ignored
        switch strategy_enter_lanelet_crossing_area
            case '1'
                % no constraint on entering the crossing area
                return
            case '2'
                % not allowed to enter the crossing area if they are coupled at intersecting lanelets of the intersection
                is_ignore_coupling = is_intersecting_lanelets && is_at_intersection;
            case '3'
                % not allowed to enter the crossing area if they are coupled at intersecting or merging lanelets of the intersection
                is_ignore_coupling = (is_intersecting_lanelets || is_merging_lanelets) && is_at_intersection;
            case '4'
                % not allowed to enter the crossing area if they are coupled at intersecting or merging lanelets regardless whether they are at the intersection or not
                is_ignore_coupling = is_intersecting_lanelets || is_merging_lanelets;
            otherwise
                warning("Please specify one of the following strategies to let vehicle enter crossing area: '0', '1', '2', '3'.")
                return
        end

        if is_ignore_coupling
           % check if vehicle without right-of-way has already enter the crossing area
            
            % get the crossing area of two vehicles' lanelet
            lanelet_crossing_area = intersect(iter.predicted_lanelet_boundary{veh_with_ROW,3}, iter.predicted_lanelet_boundary{veh_without_ROW,3});
        
            [lanelet_crossing_area_x, lanelet_crossing_area_y] = boundary(lanelet_crossing_area);
            lanelet_crossing_area_xy = [lanelet_crossing_area_x,lanelet_crossing_area_y]';

            
            % check if at least one emergency (braking/left turn/right turn) maneuver is collision-free with lanelet crossing area 
            if any(inpolygon(iter.occupied_areas{veh_without_ROW}.without_offset(1,:),iter.occupied_areas{veh_without_ROW}.without_offset(2,:),lanelet_crossing_area_x,lanelet_crossing_area_y)) || ... % ensure vehicle is not in the lanelet crossing area
                    (InterX(iter.emergency_maneuvers{veh_without_ROW}.braking_area_without_offset,lanelet_crossing_area_xy) && ... % check if emergency braking maneuver could avoid the LCA
                    InterX(iter.emergency_maneuvers{veh_without_ROW}.left_area_without_offset,lanelet_crossing_area_xy) && ... % check if emergency left-turn maneuver could avoid the LCA
                    InterX(iter.emergency_maneuvers{veh_without_ROW}.right_area_without_offset,lanelet_crossing_area_xy)) % check if emergency right-turn maneuver could avoid the LCA
                % The vehicle without the ROW must be allowed to enter the lanelet crossing area since all its emergency meneuvers
                % will collide with this area. In this case, we check if vehicle with the ROW could be forbidded to enter.
                if any(inpolygon(iter.occupied_areas{veh_with_ROW}.without_offset(1,:),iter.occupied_areas{veh_with_ROW}.without_offset(2,:),lanelet_crossing_area_x,lanelet_crossing_area_y)) || ... % ensure vehicle is not in the lanelet crossing area
                        (InterX(iter.emergency_maneuvers{veh_with_ROW}.braking_area_without_offset,lanelet_crossing_area_xy) && ... % check if emergency braking maneuver could avoid the LCA
                        InterX(iter.emergency_maneuvers{veh_with_ROW}.left_area_without_offset,lanelet_crossing_area_xy) && ... % check if emergency left-turn maneuver could avoid the LCA
                        InterX(iter.emergency_maneuvers{veh_with_ROW}.right_area_without_offset,lanelet_crossing_area_xy)) % check if emergency right-turn maneuver could avoid the LCA
                    % The vehicle with the ROW must also be allowed to enter 
                    % disp(['Both vehicle ' num2str(veh_with_ROW) ' and ' num2str(veh_without_ROW) ' have entered the crossing area, thus the coupling cannot be ignored.'])
                    continue
                else
                    % vehicle with the ROW could be forbidded to enter while vehicle without the ROW could not: swap their ROW and forbid vehicle with the ROW to enter
                    % disp(['Swap right-of-way: vehicle ' num2str(veh_without_ROW) ' now has right-of-way over ' num2str(veh_with_ROW) '.'])
                    veh_forbid = veh_with_ROW;
                    veh_free = veh_without_ROW;
                end
            else
                % forbid vehicle without the ROW to enter
                veh_forbid = veh_without_ROW;
                veh_free = veh_with_ROW;
            end

            % disp(['Ignore the coupling from vehicle ' num2str(veh_free) ' to ' num2str(veh_forbid) ' by forbidding the latter to enter the crossing area of their lanelets.'])
            coupling_info(i).is_ignored = true; % ignore coupling since no collision if possible anymore
            % update
            coupling_info(i).veh_with_ROW = veh_free;
            coupling_info(i).veh_without_ROW = veh_forbid;

            coupling_weights_reduced(veh_with_ROW,veh_without_ROW) = 0;
            % store lanelet crossing area for later use
            for iRegion = 1:lanelet_crossing_area.NumRegions
                [x_tmp,y_tmp] = boundary(lanelet_crossing_area,iRegion);
                lanelet_crossing_areas{veh_forbid}(end+1) = {[x_tmp';y_tmp']};
            end

        end
    end
end


%% local function
function [coupling_weights_reduced,coupling_info] = check_and_break_circle(coupling_weights_reduced,coupling_weights,coupling_info,vehs_at_intersection)
    % This function break coupling circle if exist any:
    % 1. If possible, coupling edge between vehicles at the
    % intersection will not be broke
    % 2. If possible, alway break coupling eage with a lower weight 

    edge_to_invert = {};
    % directed Graph
    Graph = digraph(coupling_weights_reduced);
    
    if ~isdag(Graph)
        % get cycles in the directed graph
        [~, edges] = all_elem_cycles(Graph);
        assert(~isempty(edges))
        % break the coupling edge with the lowest weight until no circle exists
        while ~isempty(edges)
            % find the edge with lowest weight
            edges_all = unique([edges{:}],'stable');
            [~,edge_idx] = sort(Graph.Edges.Weight(edges_all),'ascend');
            edge_sorted = edges_all(edge_idx);
    
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
            coupling_weights_reduced(vertex_start,vertex_end) = 0; % break the coupling edge
            
            Graph = digraph(coupling_weights_reduced);
            [~, edges] = all_elem_cycles(Graph);
        end
    end

    % not all edges in `edge_to_invert` must be inverted, some could keep its direction without
    % resulting in cycles
    for i = 1:length(edge_to_invert)
        vertex_start = edge_to_invert{i}(1);
        vertex_end = edge_to_invert{i}(2);
        % check if the recovering of an edge with the same direction will lead to cycle
        coupling_weights_reduced(vertex_start,vertex_end) = coupling_weights(vertex_start,vertex_end);
        if ~kahn(coupling_weights_reduced)
            % this edge is not allowed to be recovered with the same direction
            coupling_weights_reduced(vertex_start,vertex_end) = 0;
            % the broken edge is recovered but the direction is inverted
            coupling_weights_reduced(vertex_end,vertex_start) = coupling_weights(vertex_start,vertex_end);

            % swap leader and follower
            find_vertex_start = find([coupling_info.veh_with_ROW]==vertex_start);
            find_vertex_end = find([coupling_info.veh_without_ROW]==vertex_end);
            coupling_i = intersect(find_vertex_start,find_vertex_end);
            coupling_info(coupling_i).veh_with_ROW = vertex_end;
            coupling_info(coupling_i).veh_without_ROW = vertex_start;
%             disp(['Edge from ' num2str(vertex_start) ' to ' num2str(vertex_end) ' is inverted.'])
        else
            % this edge is allowed to be recovered with the same direction
%             disp(['Edge from ' num2str(vertex_start) ' to ' num2str(vertex_end) ' is not inverted.'])
        end

    end

end 

