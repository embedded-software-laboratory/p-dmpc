function draw_cars(poses)
%DRAW_CAR Plots a triangle according to the pose
    n_veh = length(poses.xs);        
    for i = 1 : n_veh
        cur_color = vehColor(i);
        vehicle_yaw = poses.yaws(i) - 2*0.785398;
        R = [cos(vehicle_yaw) -sin(vehicle_yaw); sin(vehicle_yaw) cos(vehicle_yaw)];
        vehicle_points = 2*[-0.5 0 0.5;-1/3 2/3 -1/3];
        vehicle_points = R*vehicle_points;
        vehicle = polyshape(vehicle_points(1,:),vehicle_points(2,:));
        vehicle = translate(vehicle,[poses.xs(i) poses.ys(i)]);
        plot(vehicle,'FaceColor', cur_color, 'EdgeColor', 'no');
    end
end

