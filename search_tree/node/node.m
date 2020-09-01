classdef node
    %NODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id
        parent
        trims
        xs
        ys
        yaws
        g_values
        h_values
    end
    
    methods
        function obj = node(id, parent, trims, xs, ys, yaws, g_values, h_values)
            %NODE Construct an instance of this class
            %   Detailed explanation goes here
            obj.id = id;
            obj.parent = parent;
            obj.trims = trims;
            obj.xs = xs;
            obj.ys = ys;
            obj.yaws = yaws;
            obj.g_values = g_values;
            obj.h_values = h_values;
        end
    end
end
