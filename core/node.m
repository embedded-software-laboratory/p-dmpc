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
        gvalues
        hvalues
    end
    
    methods
        function obj = node(id, parent, trims, xs, ys, yaws, gvalues, hvalues)
            %NODE Construct an instance of this class
            %   Detailed explanation goes here
            obj.id = id;
            obj.parent = parent;
            obj.trims = trims;
            obj.xs = xs;
            obj.ys = ys;
            obj.yaws = yaws;
            obj.gvalues = gvalues;
            obj.hvalues = hvalues;
        end
    end
end
