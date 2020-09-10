classdef node
    %NODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        depth
        trims
        xs
        ys
        yaws
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
            obj.g_values = g_values;
            obj.h_values = h_values;
        end
    end
end
