function [visualization_command] = lab_visualizer(points, visualization_type, color)

    visualization_command = Visualization;
    id = randi(100000);
    visColor = Color;
    visualization_points_2D = [];

    switch visualization_type
        case 'referenceTrajectory' 
            visualization_command.id = uint64(id);
            visualization_command.type = VisualizationType.LineStrips;

            % light blue
            visColor.r = uint8(0);
            visColor.g = uint8(255);
            visColor.b = uint8(255);

            for i = 1:length(points)
                point.x = double(points(i,1));
                point.y = double(points(i,2));
                visualization_points_2D = [visualization_points_2D, point];
            end

        case 'laneChange' 
            visualization_command.id = uint64(id);
            visualization_command.type = VisualizationType.LineStrips;

            % light green
            visColor.r = uint8(0);
            visColor.g = uint8(230);
            visColor.b = uint8(26);

            for i = 1:length(points)
                point.x = double(points(1,i).px);
                point.y = double(points(1,i).py);
                visualization_points_2D = [visualization_points_2D, point];
            end

        case 'point'
            visualization_command.id = uint64(id);
            visualization_command.type = VisualizationType.FilledCircle;

            visColor = color;

            point.x = double(points.x);
            point.y = double(points.y);
            visualization_points_2D = point;


        case 'polygon'
            visualization_command.id = uint64(id);
            visualization_command.type = VisualizationType.Polygon;

            % red
            visColor.r = uint8(255);
            visColor.g = uint8(0);
            visColor.b = uint8(0);

            for i = 1:length(points)
                point.x = double(points(i,1));
                point.y = double(points(i,2));
                visualization_points_2D = [visualization_points_2D, point];
            end
    end

    visualization_command.time_to_live = uint64(250000000);
    visualization_command.points = visualization_points_2D; 
    visualization_command.color = visColor;
    visualization_command.size = double(0.03);

end