function [min,max] = project_polygon_interval(axis,shapes)
% PROJECT_POLYGON_INTERVAL  Return min and max values when projecting all points form shapes onto axis.

    noe = length(shapes(1,:));
    dotprod = dot(axis,shapes(:,1));
    min = dotprod;
    max = dotprod;
    for i = 2:noe
        dotprod = dot(axis,shapes(:,i));
        if dotprod < min
            min = dotprod;
        end
        if dotprod > max
            max = dotprod;
        end
    end
end
