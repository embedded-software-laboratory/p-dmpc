function draw_destination(target_poses)
%DRAW_DESTINATION Plots a circle at the target positions
    n_veh = length(target_poses.xs);
                        
    for i = 1 : n_veh
        hold on
        cur_color = vehColor(i);
        radius = 1;
        center = [target_poses.xs(i), target_poses.ys(i)];
        position = [center - radius, 2*radius, 2*radius];
        rectangle('Position',position,'Curvature',[1 1], 'FaceColor', [cur_color, 0.3], 'EdgeColor', 'no');
    end
end

