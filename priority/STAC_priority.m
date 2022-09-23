classdef  STAC_priority < interface_priority
% STAC_priority  Instance of interface_priority used for dynamic priority
% assignment. For vehicles driving consectively, vehicles in the front are
% aasigned higher priorities; For vehicles crossing the intersection, vehicles
% that can arrive at the assumed collision point earlier are assigned with higher priorities. 
% Vehicles at intersection keep higher priority and do not change their relative priority until they leave intersection
    
    properties

    end
    
    methods 
        function obj = STAC_priority()
            obj.is_assign_unique_priority = false; % whether to asign unique priority
        end

        %% priority
        function [vehs_at_intersection,coupling_weights,coupling_weights_optimal,coupling_info,time_enter_intersection] = priority(~,scenario,iter)
            traffic_info = TrafficInfo(scenario, iter);
            vehs_at_intersection = traffic_info.vehs_at_intersection;
            coupling_weights = traffic_info.coupling_weights;
            coupling_weights_optimal = traffic_info.coupling_weights_optimal;
            coupling_info = traffic_info.coupling_info;
            time_enter_intersection = traffic_info.time_enter_intersection;
        end
    end

    methods (Access = private, Static)
        
    end 
  
end

