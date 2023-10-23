classdef OptimalWeigher < Weigher
    % OPTIMALWEIGHER  Instance of weight used for coupling weighing
    % weight optimal between two coupled vehicles by counting the number of valid
    % motion primitives of the lower-priority vehicle. A motion primitive is
    % valid, if a fail-safe trajectory could still be found after this motion
    % primitive is executed.

    properties (Access = private)
    end

    methods

        function obj = OptimalWeigher()
        end

        function [weighted_coupling] = weigh(obj, scenario, mpa, iter)
            weighted_coupling = iter.directed_coupling;

            [rows, cols] = find(iter.directed_coupling);

            for i_row_col_pair = 1:length(rows)
                row = rows(i_row_col_pair);
                col = cols(i_row_col_pair);
                weighted_coupling(row, col) = obj.get_optimal_coupling_weight(scenario, mpa, iter, row, col);
            end

        end

    end

    methods (Access = private)

        function [optimal_coupling_weight] = get_optimal_coupling_weight(~, scenario, mpa, iter, veh_i, veh_j)
            % GET_OPTIMAL_COUPLING_WEIGHT This function calculates the optimal coupling
            % weight between two coupled vehicles by counting the number of valid
            % motion primitives of the lower-priority vehicle. A motion primitive is
            % valid, if a fail-safe trajectory could still be found after this motion
            % primitive is executed.

            % Filter scenario and iter
            filter_self = false(1, scenario.options.amount);
            filter_self(veh_j) = true;
            iter_v = filter_iter(iter, filter_self);
            % Reduce the prediction horizon by one
            scenario.options.Hp = scenario.options.Hp - 1;
            % Add reachable sets as dynamic obstacles
            reachable_sets_i = iter.reachable_sets(veh_i, 2:end);
            % Turn polyshape to plain array (repeat the first row to enclosed the shape)
            reachable_sets_i_array = cellfun(@(c) {[c.Vertices(:, 1)', c.Vertices(1, 1)'; c.Vertices(:, 2)', c.Vertices(1, 2)']}, reachable_sets_i);
            iter_v.dynamic_obstacle_reachableSets(end + 1, :) = reachable_sets_i_array;

            % Reduce the information by one step
            mpa.transition_matrix(:, :, 1) = [];
            mpa.transition_matrix_single(:, :, 1) = [];

            % Reduce the information by one step
            iter_v.reference_trajectory_index(1) = [];
            iter_v.reference_trajectory_points(:, 1, :) = [];
            iter_v.v_ref(1) = [];

            predicted_lanelet_boundary = [iter_v.predicted_lanelet_boundary{1}, [nan; nan], iter_v.predicted_lanelet_boundary{2}];

            states_current = iter_v.x0;
            % Find all connected trims
            cur_trim_id = iter_v.trim_indices;
            connected_trims = find(mpa.transition_matrix(cur_trim_id, :, 1));
            are_valid = true(size(connected_trims));

            % Execute motion primitives
            for iTrim = 1:length(connected_trims)
                trim_next = connected_trims(iTrim);
                m = mpa.maneuvers{cur_trim_id, trim_next};
                [x_area_next, y_area_next] = translate_global(states_current(3), states_current(1), states_current(2), m.area(1, :), m.area(2, :));
                [x_area_next_without_offset, y_area_next_without_offset] = translate_global(states_current(3), states_current(1), states_current(2), m.area_without_offset(1, :), m.area_without_offset(2, :));
                area_next_xy = [x_area_next; y_area_next];
                area_next_xy_without_offset = [x_area_next_without_offset; y_area_next_without_offset];

                [RS_i_x, RS_i_y] = boundary(iter.reachable_sets{veh_i, 1});
                RS_i_xy = [RS_i_x, RS_i_y]';

                % Check if the next motion primitive is collision-free with the
                % reachable set of the higher-priority vehicle in the first time
                % step and with the lanelet boundary
                if InterX(area_next_xy, RS_i_xy)
                    % If not collision-free
                    are_valid(iTrim) = false;
                elseif ~anynan(predicted_lanelet_boundary) % there is a lanelet boundary
                    are_valid(iTrim) = ~InterX( ...
                        area_next_xy_without_offset, ...
                        predicted_lanelet_boundary ...
                    );
                else
                    % If collision-free, execute graph search and see if a fail-safe
                    % trajectory (for Hp-1 steps) could still be found if the current motion
                    % primitive is executed

                    % Update iteration information since a motion primitive is
                    % executed
                    iter_v.trim_indices = trim_next;
                    [x0_next, y0_next] = translate_global(states_current(3), states_current(1), states_current(2), m.dx, m.dy);
                    yaw0_next = states_current(3) + m.dyaw;
                    speed0_next = mpa.trims(trim_next).speed;
                    iter_v.x0 = [x0_next, y0_next, yaw0_next, speed0_next];

                    [info_v, ~] = GraphSearch(scenario, mpa).run_optimizer(iter_v, []);

                    if info_v.is_exhausted
                        are_valid(iTrim) = false;
                    end

                end

            end

            % Set the coupling weight to the percentage of invalid motion
            % primitives
            optimal_coupling_weight = 1 - nnz(are_valid) / length(are_valid);

            if optimal_coupling_weight == 0
                % keep the fact that those two vehicles are coupled because their reachable sets intersect
                optimal_coupling_weight = 1e-3;
            end

            if optimal_coupling_weight == 1
                % a coupling weight of 1 means if those two vehicles are not in the
                % same group, the lower-priority one cannot find a feasible
                % trajectory; therefore, set it to a very large value so that the
                % graph cut algorithm will put them in the same group
                optimal_coupling_weight = 10;
            end

        end

    end

end
