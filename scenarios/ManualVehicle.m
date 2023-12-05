classdef ManualVehicle
    % MANUALVEHICLE   Class representing manual vehicles in CpmLab Experiment

    properties
        Length = 0.22; % Vehicle length (bumper to bumper)[m]
        Width = 0.1; % Vehicle width [m]
        Lf = 0.1; % Distance between vehicle center and front axle center [m]
        Lr = 0.1; % Distance between vehicle center and rear axle center [m]
        ID = -1; % vehicle ID (should be positive integer)
        lanelets_index;
        points_index;
        road_raw_data;
        options;
        mpa;
    end

    methods

        function obj = ManualVehicle(id, options, road_raw_data)
            options.mpa_type = MpaType.single_speed;
            options.is_prioritized = true;
            options.is_use_dynamic_programming = true;
            options.recursive_feasibility = false;
            options.is_bounded_reachable_set_used = true;
            obj.options = options;
            obj.road_raw_data = road_raw_data;
            obj.ID = id;
            obj.mpa = MotionPrimitiveAutomaton(BicycleModel(obj.Lf, obj.Lr), options);
        end

        function reachable_sets = compute_reachable_lane(obj, measurement, lanelet_ids)
            local_reachable_sets = obj.mpa.local_reachable_sets;
            lanelet_struct = obj.road_raw_data.lanelet;
            reachable_sets = get_reachable_sets(measurement.x, measurement.y, measurement.yaw, local_reachable_sets(1, :), cell(1, 3), obj.options);
            lanelets_poly = polyshape;

            for id = lanelet_ids
                successor_id = lanelet_struct(id).successor.refAttribute;
                current_lanelet_poly = polyshape([lanelet_struct(id).leftBound.point(:).x, lanelet_struct(id).rightBound.point(end:-1:1).x
                                                  lanelet_struct(id).leftBound.point(:).y, lanelet_struct(id).rightBound.point(end:-1:1).y]');
                successor_lanelet_poly = polyshape([lanelet_struct(successor_id).leftBound.point(:).x, lanelet_struct(successor_id).rightBound.point(end:-1:1).x
                                                    lanelet_struct(successor_id).leftBound.point(:).y, lanelet_struct(successor_id).rightBound.point(end:-1:1).y]');
                lanelets_poly = union([lanelets_poly, current_lanelet_poly, successor_lanelet_poly]);
            end

            for index = 1:length(reachable_sets)
                reachable_sets{index} = intersect(reachable_sets{index}, lanelets_poly);
            end

        end

        function plot(obj, color)
            vehicle_polygon = transformed_rectangle(obj.Length, obj.Width);
            patch(vehicle_polygon(1, :), vehicle_polygon(2, :), color);
        end

    end

end
