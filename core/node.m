classdef node
    %NODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id
        parent
        values
        trims
        xs
        ys
        yaws
        driven
    end
    
    methods
        function obj = node(id, parent, values, trims, xs, ys, yaws, driven)
            %NODE Construct an instance of this class
            %   Detailed explanation goes here
            obj.id = id;
            obj.parent = parent;
            obj.values = values;
            obj.trims = trims;
            obj.xs = xs;
            obj.ys = ys;
            obj.yaws = yaws;
            obj.driven = driven;
        end
    end
end
