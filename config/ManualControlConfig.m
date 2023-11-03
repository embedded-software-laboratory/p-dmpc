classdef ManualControlConfig
    %MANUALCONTROLCONFIG Configuration specificed to Manual Control Scenarios

    properties
        is_active = false; % whether manual vehicles should be considered
        amount = 0;
        hdv_ids = []; % vehicle ID of manually controlled vehicle (MCV) TODO: assing manually
    end

    methods

        function obj = ManualControlConfig()
        end

        function obj = jsondecode(obj, json_struct)
            fields = fieldnames(json_struct);

            for i_field = 1:length(fields)
                field = fields{i_field};

                if ~isprop(obj, field)
                    warning('Cannot set property %s for class ManualControlConfig as it does not exist', field);
                    continue
                end

                obj.(field) = json_struct.(field);

            end

        end

    end

end
