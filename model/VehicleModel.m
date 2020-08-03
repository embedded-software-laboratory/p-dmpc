classdef VehicleModel
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        nx
        nu
        ny
    end
    
    methods(Abstract)
        dx = ode(x,u)
    end
end
