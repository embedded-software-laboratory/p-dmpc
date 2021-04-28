function draw_destination(scenario)
%DRAW_DESTINATION Plots a circle at the target positions
    for i = 1 : scenario.nVeh
        hold on
        cur_color = vehColor(i);
        radius = scenario.r_goal;
        center = [scenario.vehicles(i).x_goal, scenario.vehicles(i).y_goal];
        position = [center - radius, 2*radius, 2*radius];
        rectangle('Position',position,'Curvature',[1 1], 'FaceColor', [cur_color, 0.3], 'EdgeColor', 'no');
    end
end

