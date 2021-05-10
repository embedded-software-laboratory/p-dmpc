function draw_cars(scenario)
%DRAW_CAR Plots a triangle according to the pose
    for i = 1 : scenario.nVeh
        cur_color = vehColor(i);
        vehicle_yaw = scenario.vehicles(i).yaw_start - 2*0.785398;
        R = [cos(vehicle_yaw) -sin(vehicle_yaw); sin(vehicle_yaw) cos(vehicle_yaw)];
        vehicle_points = 2*[-0.5 0 0.5;-1/3 2/3 -1/3];
        vehicle_points = R*vehicle_points;
        vehicle = polyshape(vehicle_points(1,:),vehicle_points(2,:));
        vehicle = translate(vehicle,[scenario.vehicles(i).x_start scenario.vehicles(i).y_start]);
        plot(vehicle,'FaceColor', cur_color, 'EdgeColor', 'no');
    end
end

