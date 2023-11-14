classdef DistanceCoupler < Coupler

    methods

        function [adjacency] = couple(obj, iter, options, max_mpa_speed)
            amount = size(iter.reachable_sets, 1);

            adjacency = zeros(amount, amount);

            max_distance = 2 * max_mpa_speed * options.dt_seconds * options.Hp;

            i_x = indices().x;
            i_y = indices().y;

            for i_vehicle_1 = 1:amount

                for i_vehicle_2 = (i_vehicle_1 + 1):amount

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

end
