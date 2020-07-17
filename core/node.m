classdef node
    %NODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id
        parent
        value
        trim
        pose
    end
    
    methods
        function obj = node(id, parent, value, trim, pose)
            %NODE Construct an instance of this class
            %   Detailed explanation goes here
            obj.id = id;
            obj.parent = parent;
            obj.value = value;
            obj.trim = trim;
            obj.pose = pose;
        end
    end
end

