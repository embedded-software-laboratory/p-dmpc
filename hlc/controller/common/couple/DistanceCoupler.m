classdef DistanceCoupler < Coupler

    properties (Access = private)
        adjacency_lanelets
    end

    methods

        function obj = DistanceCoupler(scenario)
            obj@Coupler();

            obj.adjacency_lanelets = scenario.adjacency_lanelets;
        end

        function [adjacency] = couple(obj, options, max_mpa_speed, iter)
            adjacency = zeros(options.amount, options.amount);

            max_distance = 2 * max_mpa_speed * options.dt_seconds * options.Hp;

            i_x = indices().x;
            i_y = indices().y;

            for i_vehicle_1 = 1:options.amount

                for i_vehicle_2 = (i_vehicle_1 + 1):options.amount
                    % If the predicted lanelets of both vehicles are not adjacent, skip the distance measurement
                    % (only checked if information of adjacent lanelets exist, i.e., in commonroad scenario)
                    if (~isempty(obj.adjacency_lanelets) ...
                            && ~obj.is_any_lanelet_adjacent(iter.current_lanelet, iter.predicted_lanelets, i_vehicle_1, i_vehicle_2))
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

        function is_adjacent = is_any_lanelet_adjacent(obj, current_lanelet, predicted_lanelets, i_vehicle_1, i_vehicle_2)
            % IS_ANY_LANELET_ADJACENT checks whether the predicted and current lanelets of the vehicles
            %   with the given indices are adjacent to each other
            lanelets_1 = unique([current_lanelet(i_vehicle_1), predicted_lanelets{i_vehicle_1, 1}]);
            lanelets_2 = unique([current_lanelet(i_vehicle_2), predicted_lanelets{i_vehicle_2, 1}]);
            is_adjacent = any(obj.adjacency_lanelets(lanelets_1, lanelets_2), 'all');
        end

    end

end
