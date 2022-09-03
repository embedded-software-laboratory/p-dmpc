function [optimal_coupling_weight] = get_optimal_coupling_weight(scenario,iter,veh_i,veh_j)
% GET_OPTIMAL_COUPLING_WEIGHT This function calculates the optimal coupling
% weight between two coupled vehicles by counting the number of valid
% motion primitives of the lower-priority vehicle. A motion primitive is
% valid, if a fail-safe trajectory could still be found after this motion
% primitive is executed.
%     xlim([1.4,3.1])
%     ylim([1.0,2.5])
    % h = findobj('LineWidth',0.3); % % delete shapes plotted during graph searching 
    % delete(h)
    % h = findobj('LineWidth',0.75); % % delete shapes plotted during graph searching 
    % delete(h)
    % Filter scenario and iter
    filter_self = false(1,scenario.options.amount);
    filter_self(veh_j) = true;
    scenario_v = filter_scenario(scenario,filter_self);
    % Reduce the prediction horizon by one
    scenario_v.options.Hp = scenario_v.options.Hp - 1;
    % Add reachable sets as dynamic obstacles
    reachable_sets_i = iter.reachable_sets(veh_i,2:end);
    reachable_sets_i_full = iter.reachable_sets(veh_i,:);
    % Turn polyshape to plain array (repeat the first row to enclosed the shape)
    reachable_sets_i_array = cellfun(@(c) {[c.Vertices(:,1)',c.Vertices(1,1)';c.Vertices(:,2)',c.Vertices(1,2)']}, reachable_sets_i); 
    scenario_v.dynamic_obstacle_reachableSets(end+1,:) = reachable_sets_i_array;

    reachable_sets_i_array_full = cellfun(@(c) {[c.Vertices(:,1)',c.Vertices(1,1)';c.Vertices(:,2)',c.Vertices(1,2)']}, reachable_sets_i_full); 
    plot_options_RS = struct('Color',[0 0.4470 0.7410],'LineWidth',0.45,'LineStyle','-');
    % plot_obstacles(reachable_sets_i_array_full,plot_options_RS)

    % Reduce the information by one step
    scenario_v.mpa.transition_matrix(:,:,1) = [];
    scenario_v.mpa.transition_matrix_single(:,:,1) = [];

    iter_v = filter_iter(iter,filter_self);

    % Reduce the information by one step
    iter_v.referenceTrajectoryIndex(1) = [];
    iter_v.referenceTrajectoryPoints(:,1,:) = [];
    iter_v.vRef(1) = [];

    lanelet_boundary = [scenario_v.vehicles.lanelet_boundary{1},[nan;nan],scenario_v.vehicles.lanelet_boundary{2}];
    plot_options_LB = struct('Color','k','LineWidth',0.65,'LineStyle','-');
    % plot_obstacles(lanelet_boundary,plot_options_LB)
    states_current = iter_v.x0;
    % Find all connected trims
    cur_trim_id = iter_v.trim_indices;
    connected_trims = find(scenario.mpa.transition_matrix(cur_trim_id, :, 1));
    are_valid = true(size(connected_trims));

    % Execute motion primitives
    for iTrim = 1:length(connected_trims)
        trim_next = connected_trims(iTrim);
        m = scenario.mpa.maneuvers{cur_trim_id,trim_next};
        [x_area_next,y_area_next] = translate_global(states_current(3),states_current(1),states_current(2),m.area(1,:),m.area(2,:));
        [x_area_next_without_offset,y_area_next_without_offset] = translate_global(states_current(3),states_current(1),states_current(2),m.area_without_offset(1,:),m.area_without_offset(2,:));
        area_next_xy = [x_area_next;y_area_next];
        area_next_xy_without_offset = [x_area_next_without_offset;y_area_next_without_offset];
        
        [RS_i_x,RS_i_y] = boundary(iter.reachable_sets{veh_i,1});
        RS_i_xy = [RS_i_x,RS_i_y]';
        
        % Check if the next motion primitive is collision-free with the
        % reachable set of the higher-priority vehicle in the first time
        % step and with the lanelet boundary   
        if InterX(area_next_xy,RS_i_xy) 
            % If not collision-free
            are_valid(iTrim) = false;
            % disp('The next motion primitive collides with the reachable set at the first step.')
        elseif InterX(area_next_xy_without_offset,lanelet_boundary)
            are_valid(iTrim) = false;
            % disp('The next motion primitive collides with lanelet boundary.')
        else
            % If collision-free, execute graph search and see if a fail-safe
            % trajectory (for Hp-1 steps) could still be found if the current motion
            % primitive is executed

            % Update iteration information since a motion primitive is
            % executed
            iter_v.trim_indices = trim_next;
            [x0_next,y0_next] = translate_global(states_current(3),states_current(1),states_current(2),m.dx,m.dy);
            yaw0_next = states_current(3) + m.dyaw;
            speed0_next = scenario.mpa.trims(trim_next).speed;
            iter_v.x0 = [x0_next,y0_next,yaw0_next,speed0_next];

            info_v = graph_search(scenario_v,iter_v);
            if info_v.is_exhausted
                are_valid(iTrim) = false;
                % disp('Graph search is exhausted.')
            else
                plot_options_trajectories = struct('Color','b','LineWidth',0.55,'LineStyle','-');
                % plot_obstacles(info_v.shapes,plot_options_trajectories)
                % disp('Motion primitive is valid.')
            end
        end
        plot_options_next_primitive = struct('Color','b','LineWidth',0.55,'LineStyle','-');
        % plot_obstacles(area_next_xy,plot_options_next_primitive)
        % h = findobj('LineWidth',plot_options_next_primitive.LineWidth);
        % delete(h)   
        % h = findobj('LineWidth',0.3);
        % delete(h)
        % h = findobj('LineWidth',0.75);
        % delete(h)
    end

    % Set the coupling weight to the percentage of invalid motion
    % primitives
    optimal_coupling_weight = 1 - nnz(are_valid)/length(are_valid);
    if optimal_coupling_weight == 0
        % keep the fact that those two vehicles are coupled because their reachable sets intersect 
        optimal_coupling_weight = 1e-3;
    end

    if optimal_coupling_weight==1
        disp('')
    end
    % h = findobj('LineWidth',plot_options_RS.LineWidth);
    % delete(h)
end