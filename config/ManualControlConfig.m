classdef ManualControlConfig
    %MANUALCONTROLCONFIG Configuration specificed to Manual Control Scenarios

    properties
        amount = 0;
        hdv_ids = []; % vehicle ID of manually controlled vehicle (MCV) TODO: assing manually
    end

    methods

        function obj = ManualControlConfig()
        end

        function obj = assign_data(obj, struct)
            %ManualControlConfig Construct an instance of this class
            %   Detailed explanation goes here
            obj.amount = struct.amount;
            obj.hdv_ids = struct.hdv_ids;
        end

    end

end
