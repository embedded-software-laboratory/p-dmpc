function [visualization_command] = lab_visualize_polygon(scenario, points, index)

    visualization_command = Visualization;

    visualization_command.id = uint64(points(1,1) * scenario.vehicle_ids(index));
    visualization_command.type = VisualizationType.Polygon;

    visualization_command.time_to_live = uint64(250000000);
    

   points_2D = [];
    for i = 1:length(points)
        point.x = double(points(i,1));
        point.y = double(points(i,2));

        points_2D = [points_2D, point];
    end

    visualization_command.points = points_2D; 

    color = Color;
    color.r = uint8(255);
    color.g = uint8(0);
    color.b = uint8(0);
    visualization_command.color = color;
    visualization_command.size = double(0.03);

end
