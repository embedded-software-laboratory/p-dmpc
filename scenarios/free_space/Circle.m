classdef Circle < Scenario
    % CIRCLE   Scenario with vehicles circular arranged heading
    % to the center of the circle.

    methods

        function obj = Circle(amount)
            obj = obj@Scenario();

            radius = 2;
            nVeh = amount;
            yaws = pi * 2 / nVeh * (0:nVeh - 1);

            for iVeh = 1:nVeh
                yaw = yaws(iVeh);
                s = sin(yaw);
                c = cos(yaw);
                veh = Vehicle();

                veh.x_start = -c * radius;
                veh.y_start = -s * radius;
                veh.yaw_start = yaw;
                % Lab: translate by center
                center_x = 2.25;
                center_y = 2;
                veh.x_start = veh.x_start + center_x;
                veh.y_start = veh.y_start + center_y;
                x_end = veh.x_start + c * 2 * radius;
                y_end = veh.y_start + s * 2 * radius;

                veh.reference_path = [veh.x_start veh.y_start
                                      x_end y_end];

                obj.vehicles = [obj.vehicles, veh];
            end

        end

    end

end
