function [minimum,maximum] = project_polygon_interval(axis,shapes)
% PROJECT_POLYGON_INTERVAL  Return min and max values when projecting all points form shapes onto axis.

    dotprod = axis*shapes;
    minimum = min(dotprod);
    maximum = max(dotprod);    
end
