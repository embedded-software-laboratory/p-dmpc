classdef (Abstract) interface_priority < handle
% interface_priority    Abstract class used for defining properties and methods used by priority based distributed controller.
    
    properties (Access=protected)
        scenario
        groups
    end
    

    methods (Abstract) 
        priority(obj)
    end
    
    
    
    
end
