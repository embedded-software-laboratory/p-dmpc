classdef node
    %NODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        depth
        trims
        xs
        ys
        yaws
        shapes
        g_values
        h_values
    end
    
    methods
        function obj = node(depth, trims, xs, ys, yaws, g_values, h_values)
            %NODE Construct an instance of this class
            %   Detailed explanation goes here
            obj.depth = depth;
            obj.trims = trims;
            obj.xs = xs;
            obj.ys = ys;
            obj.yaws = yaws;
            for i = 1:length(yaws)
                [shape_x, shape_y] = translate_global(yaws(i), xs(i), ys(i), [-2.2 2.2 -2.2 2.2],[-0.9 -0.9 0.9 0.9]);
                obj.shapes = [obj.shapes; polyshape(shape_x,shape_y,'Simplify',false)];
            end
            obj.g_values = g_values;
            obj.h_values = h_values;
        end
    end
end
