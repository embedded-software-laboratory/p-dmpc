classdef (Abstract) InterfaceExperiment < handle
% INTERFACEEXPERIMENT    Abstract class used for defining properties and methods used by different experiment setups.
    
    properties (Access=protected)
        % struct used for every iteration
        x0
        trim_indices
        cur_node
        scenario
        k
    end
    
    methods (Abstract) 
        setup(obj)
        [ x0, trim_indices ] = measure(obj, ~)
        apply(obj, info)
        got_stop = is_stop(obj)
        end_run(obj)
    end
    
    methods
        function [ x0, trim_indices ] = measure_node(obj, reader_vehicleStateList)
            % take last planned state as new actual state
            speeds = zeros(obj.scenario.nVeh,1);
            for iVeh=1:obj.scenario.nVeh
                speeds(iVeh) = obj.scenario.mpa.trims(obj.cur_node(iVeh,NodeInfo.trim)).speed;
            end
            x0 = [obj.cur_node(:,NodeInfo.x), obj.cur_node(:,NodeInfo.y), obj.cur_node(:,NodeInfo.yaw), speeds];
            trim_indices = obj.cur_node(:,NodeInfo.trim);
            
            %{
            indexVehicleExpertMode = 0;
            for j = 1:obj.scenario.nVeh
                if ((obj.scenario.vehicle_ids(j) == obj.scenario.manual_vehicle_id && obj.scenario.options.firstManualVehicleMode == 2) ...
                    || (obj.scenario.vehicle_ids(j) == obj.scenario.second_manual_vehicle_id && obj.scenario.options.secondManualVehicleMode == 2))
                    indexVehicleExpertMode = j;
                end
            end

            if exist('reader_vehicleStateList','var') && indexVehicleExpertMode ~= 0
                [sample,~,~,~] = reader_vehicleStateList.take();
                
                pose = [sample(end).state_list.pose];
                x0(indexVehicleExpertMode,1) = [pose(1,indexVehicleExpertMode).x];
                x0(indexVehicleExpertMode,2) = [pose(1,indexVehicleExpertMode).y];
                x0(indexVehicleExpertMode,3) = [pose(1,indexVehicleExpertMode).yaw];
                x0(indexVehicleExpertMode,4) = [sample(end).state_list(1,indexVehicleExpertMode).speed];
            end
            %}
        end
    end

end

