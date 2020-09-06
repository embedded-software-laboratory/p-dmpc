function vis_car(pose, color)
%VIS_CAR Plots a triangle according to the pose
vehicle_yaw = pose.yaw - 2*0.785398;
R = [cos(vehicle_yaw) -sin(vehicle_yaw); sin(vehicle_yaw) cos(vehicle_yaw)];
vehicle_points = [-0.5 0 0.5;-1/3 2/3 -1/3];
vehicle_points = R*vehicle_points;
vehicle = polyshape(vehicle_points(1,:),vehicle_points(2,:));
vehicle = translate(vehicle,[pose.x pose.y]);
plot(vehicle,'FaceColor',color);
end

