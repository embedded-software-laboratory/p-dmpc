classdef DistanceWeighter < Weighter
    % DISTANCEWEIGHTER gives higher weight for lower distance

    properties (Access = private)
    end

    methods

        function obj = DistanceWeighter()
        end

        function [weighted_coupling] = weigh(~, scenario, mpa, iter)
            weighted_coupling = iter.directed_coupling;

            [row, col] = find(iter.directed_coupling);
            n_couplings = length(row);

            max_distance = 2 * mpa.get_max_speed_of_mpa() ...
                * scenario.options.dt_seconds ...
                * scenario.options.Hp;

            i_x = indices().x;
            i_y = indices().y;

            for i_coupling = 1:n_couplings
                i_vehicle_1 = row(i_coupling);
                i_vehicle_2 = col(i_coupling);

                position_1 = [iter.x0(i_vehicle_1, i_x);
                              iter.x0(i_vehicle_1, i_y)];
                position_2 = [iter.x0(i_vehicle_2, i_x);
                              iter.x0(i_vehicle_2, i_y)];

                distance = norm(position_1 - position_2);

                weighted_coupling(i_vehicle_1, i_vehicle_2) = 1 - distance / max_distance;
            end

        end

    end

end
