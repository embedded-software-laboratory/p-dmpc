classdef ManualControlConfig
    % ManualControlConfig Configuration specified to Manual Control Scenarios

    properties
        % whether manual vehicles should be considered
        is_active (1, 1) logical = false;
        % amount of manually controlled vehicles
        amount (1, 1) double = 0;
        % vehicle ids of manually controlled vehicle (MCV)
        % data type is related to *vehicle_ids in Plant.m
        hdv_ids (1, :) uint8 = [];
    end

    methods

        function obj = ManualControlConfig()
        end

        function obj = jsondecode(obj, json_struct)
            % for each loop requires fields as row vector
            fields = string(fieldnames(json_struct)).';

            for field = fields

                if ~isprop(obj, field)
                    warning('Cannot set property %s for class ManualControlConfig as it does not exist', field);
                    continue
                end

                obj.(field) = json_struct.(field);

            end

        end

    end

end
