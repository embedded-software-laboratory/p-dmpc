function scenario = circle_scenario(options)
    % CIRCLE_SCENARIO   Constructor for scenario with vehicles circular arranged heading to the center of the circle.

    scenario = Scenario();

    radius = 2;
    nVeh = options.amount;
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

        scenario.vehicles = [scenario.vehicles, veh];
    end

    scenario.model = BicycleModel(veh.Lf, veh.Lr);

end
