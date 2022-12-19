classdef (Abstract) InterfaceExperiment < handle
% INTERFACEEXPERIMENT    Abstract class used for defining properties and methods used by different experiment setups.
    
    properties (Access=protected)
        % struct used for every iteration
        x0
        trim_indices
        cur_node
        scenario
        k
        veh_ids % which vehicles will controlled by this experiment instance
        amount
    end
    
    methods (Abstract)
        setup(obj)
        [ x0, trim_indices ] = measure(obj, ~)
        apply(obj, info)
        got_stop = is_stop(obj)
        end_run(obj)
    end
    
    methods
        function obj = InterfaceExperiment(veh_ids)
            obj.veh_ids = veh_ids;
            obj.amount = length(veh_ids);
        end

        function [ x0, trim_indices ] = measure_node(obj)
            % take last planned state as new actual state
            speeds = zeros(obj.amount,1);
            for iVeh=1:obj.amount
                speeds(iVeh) = obj.scenario.mpa.trims(obj.cur_node(iVeh,NodeInfo.trim)).speed;
            end
            x0 = [obj.cur_node(:,NodeInfo.x), obj.cur_node(:,NodeInfo.y), obj.cur_node(:,NodeInfo.yaw), speeds];
            trim_indices = obj.cur_node(:,NodeInfo.trim);
        end
    end

end

