function [visualization_command] = lab_visualize_lane_change(scenario, trajectory_points, index, isLine)

    visualization_command = Visualization;

    if isLine
        visualization_command.id = uint64(17*scenario.vehicle_ids(index));
        visualization_command.type = VisualizationType.LineStrips;
    else
        visualization_command.type = VisualizationType.FilledCircle;
        visualization_command.id = uint64(19*scenario.vehicle_ids(index));
    end

    visualization_command.time_to_live = uint64(250000000);
    
    trajectory_points_2D = [];
    for i = 1:length(trajectory_points)
        point.x = double(trajectory_points(1,i).px);
        point.y = double(trajectory_points(1,i).py);

        trajectory_points_2D = [trajectory_points_2D, point];
    end

    visualization_command.points = trajectory_points_2D; 

    laneChangeColor = Color;
    laneChangeColor.r = uint8(0);
    laneChangeColor.g = uint8(230);
    laneChangeColor.b = uint8(26);

    visualization_command.color = laneChangeColor;
    visualization_command.size = double(0.03);

end