classdef VehicleModel
% VEHICLEMODEL  Parent Class for all vehicle models
    
    properties
        nx
        nu
        ny
    end
    
    methods(Abstract)
        dx = ode(obj,x,u)
    end
end
