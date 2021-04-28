classdef node
    
    properties
        depth
        trims
        xs
        ys
        yaws
%         shapes
        g_values
        h_values
        exact_eval = false
    end
    properties (Dependent)
    end
    
    methods
        function obj = node(depth, trims, xs, ys, yaws, g_values, h_values)
            if nargin == 0
                return;
            end
            obj.depth = depth;
            obj.trims = trims;
            obj.xs = xs;
            obj.ys = ys;
            obj.yaws = yaws;
%             nVeh = length(yaws);
%             obj.shapes = cell(nVeh,1);
%             for i = 1:nVeh
%                 [shape_x, shape_y] = translate_global(yaws(i), xs(i), ys(i), [-2.2 2.2 -2.2 2.2],[-0.9 -0.9 0.9 0.9]);
%                 obj.shapes{i} = [shape_x;shape_y];
%             end
            obj.g_values = g_values;
            obj.h_values = h_values;
        end
    end
    
end
