classdef DistanceCoupler < Coupler

    methods

        function [adjacency] = couple(obj, iter, options, max_mpa_speed, adjacency_lanelets)
            amount = size(iter.reachable_sets, 1);

            adjacency = zeros(amount, amount);

            max_distance = 2 * max_mpa_speed * options.dt_seconds * options.Hp;

            i_x = indices().x;
            i_y = indices().y;

            for i_vehicle_1 = 1:amount

                for i_vehicle_2 = (i_vehicle_1 + 1):amount
                    % If the predicted lanelets of both vehicles are not adjacent, skip the distance measurement
                    % (only checked if information of adjacent lanelets exist, i.e., in commonroad or lanelet2 scenarios)
                    if (~isempty(adjacency_lanelets) ...
                            && ~obj.is_predicted_lanelets_adjacent(iter.predicted_lanelets, adjacency_lanelets, i_vehicle_1, i_vehicle_2))
                        continue;
                    end

                    % Check the distance between the vehicles for coupling
                    position_1 = [iter.x0(i_vehicle_1, i_x);
                                  iter.x0(i_vehicle_1, i_y)];
                    position_2 = [iter.x0(i_vehicle_2, i_x);
                                  iter.x0(i_vehicle_2, i_y)];

                    distance = norm(position_1 - position_2);
                    is_coupled = (distance <= max_distance);

                    adjacency(i_vehicle_1, i_vehicle_2) = is_coupled;
                    adjacency(i_vehicle_2, i_vehicle_1) = is_coupled;

                end

            end

        end

    end

    methods (Access = private)

        function [is_adjacent] = is_predicted_lanelets_adjacent(obj, predicted_lanelets, adjacency_lanelets, i_vehicle_1, i_vehicle_2)
            % IS_PREDICTED_LANELETS_ADJACENT checks whether the predicted lanelets of the vehicles with the given indices
            %   are adjacent to each other
            is_adjacent = false;

            for predicted_lanelet_1 = predicted_lanelets{i_vehicle_1, 1}

                for predicted_lanelet_2 = predicted_lanelets{i_vehicle_2, 1}

                    if adjacency_lanelets(predicted_lanelet_1, predicted_lanelet_2)
                        is_adjacent = true;
                        return;
                    end

                end

            end

        end

    end

end
