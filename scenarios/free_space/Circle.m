classdef Circle < Scenario
    % CIRCLE   Scenario with vehicles circular arranged heading
    % to the center of the circle.

    methods

        function obj = Circle(options)
            obj = obj@Scenario(options);

            radius = 2;
            nVeh = options.amount;
            yaws = pi * 2 / nVeh * (0:nVeh - 1);

            for iVeh = 1:nVeh
                yaw = yaws(iVeh);
                s = sin(yaw);
                c = cos(yaw);

                obj.vehicles(iVeh).x_start = -c * radius;
                obj.vehicles(iVeh).y_start = -s * radius;
                obj.vehicles(iVeh).yaw_start = yaw;
                % Lab: translate by center
                center_x = 2.25;
                center_y = 2;
                obj.vehicles(iVeh).x_start = obj.vehicles(iVeh).x_start + center_x;
                obj.vehicles(iVeh).y_start = obj.vehicles(iVeh).y_start + center_y;
                x_end = obj.vehicles(iVeh).x_start + c * 2 * radius;
                y_end = obj.vehicles(iVeh).y_start + s * 2 * radius;

                obj.vehicles(iVeh).reference_path = [obj.vehicles(iVeh).x_start obj.vehicles(iVeh).y_start
                                      x_end y_end];

                obj.vehicles = [obj.vehicles, veh];
            end

        end

    end

end
