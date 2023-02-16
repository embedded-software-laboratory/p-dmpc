classdef ManualControlConfig
    %Mixed_traffic_config Configuration specificed to mixed traffic mode
    
    properties
      amount = 0;
      hdv_ids = [];                           % vehicle ID of manually controlled vehicle (MCV) TODO: assing manually
    end

    methods
        function obj = ManualControlConfig()
        end

        function obj = assign_data(obj, struct)
            %Mixed_traffic_config Construct an instance of this class
            %   Detailed explanation goes here
            obj.amount = struct.amount;
            obj.hdv_ids = struct.hdv_ids;
        end
    end
end

