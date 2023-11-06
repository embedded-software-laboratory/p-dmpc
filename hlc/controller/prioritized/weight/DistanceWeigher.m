classdef DistanceWeigher < Weigher
    % DISTANCEWEIGHER gives higher weight for lower distance

    properties (Access = private)
    end

    methods

        function obj = DistanceWeigher()
        end

        function [weighted_coupling] = weigh(~, iter, ~, options, max_mpa_speed)
            weighted_coupling = iter.directed_coupling;

            [row, col] = find(iter.directed_coupling);
            n_couplings = length(row);

            max_distance = 2 * max_mpa_speed * options.dt_seconds * options.Hp;

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
