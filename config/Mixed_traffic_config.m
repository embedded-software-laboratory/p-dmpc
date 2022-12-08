classdef Mixed_traffic_config
    %Mixed_traffic_config Configuration specificed to mixed traffic mode
    
    properties
        first_manual_vehicle_mode Control_Mode
        first_manual_vehicle_id string
        force_feedback logical
        second_manual_vehicle_mode Control_Mode
        second_manual_vehicle_id string
        consider_rss logical
        collision_avoidance Collision_Avoidance_Mode
    end

    methods
        function obj = Mixed_traffic_config()
        end

        function obj = assign_data(obj, struct)
            %Mixed_traffic_config Construct an instance of this class
            %   Detailed explanation goes here
            obj.first_manual_vehicle_mode = struct.first_manual_vehicle_mode;
            obj.first_manual_vehicle_id = struct.first_manual_vehicle_id;
            obj.force_feedback = struct.force_feedback;
            obj.second_manual_vehicle_mode = struct.second_manual_vehicle_mode;
            obj.second_manual_vehicle_id = struct.second_manual_vehicle_id;
            obj.consider_rss = struct.consider_rss;
            obj.collision_avoidance = struct.collision_avoidance;
        end
    end
end

