function draw_cars(poses)
%VIS_CAR Plots a triangle according to the pose
    n_veh = length(poses);
    vehColors = [0.8941    0.1020    0.1098;...
                 0.2157    0.4941    0.7216;...
                 0.3020    0.6863    0.2902;...
                 0.5961    0.3059    0.6392;...
                 1.0000    0.4980    0     ;...
                 1.0000    1.0000    0.2000;...
                 0.6510    0.3373    0.1569;...
                 0.9686    0.5059    0.7490];
                        
    for i = 1 : n_veh
        cur_color = vehColors(mod(i-1,size(vehColors,1))+1,:);
        vehicle_yaw = poses(i).yaw - 2*0.785398;
        R = [cos(vehicle_yaw) -sin(vehicle_yaw); sin(vehicle_yaw) cos(vehicle_yaw)];
        vehicle_points = [-0.5 0 0.5;-1/3 2/3 -1/3];
        vehicle_points = R*vehicle_points;
        vehicle = polyshape(vehicle_points(1,:),vehicle_points(2,:));
        vehicle = translate(vehicle,[poses(i).x poses(i).y]);
        plot(vehicle,'FaceColor',cur_color);
    end
end

