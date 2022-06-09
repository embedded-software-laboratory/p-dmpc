function [visualization_command] = lab_visualize_point(scenario, visualization_point, index, color)

    visualization_command = Visualization;

    visualization_command.id = uint64(visualization_point.x * scenario.vehicle_ids(index));
    visualization_command.type = VisualizationType.FilledCircle;

    visualization_command.time_to_live = uint64(250000000);
    

        point.x = double(visualization_point.x);
        point.y = double(visualization_point.y);

        visualization_points_2D = point;


    visualization_command.points = visualization_points_2D; 

    visualization_command.color = color;
    visualization_command.size = double(0.03);

end