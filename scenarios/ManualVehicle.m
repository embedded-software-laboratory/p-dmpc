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
        model;
        mpa;
    end

    methods

        function obj = ManualVehicle(id, scenario)
            options = scenario.options;
            options.trim_set = 7;
            options.is_prioritized = true;
            options.is_save_mpa = true;
            options.is_use_dynamic_programming = true;
            options.recursive_feasibility = false;
            options.bound_reachable_sets = true;
            obj.options = options;
            obj.road_raw_data = scenario.road_raw_data;
            obj.ID = id;
            obj.model = BicycleModel(obj.Lf, obj.Lr);
            obj.mpa = MotionPrimitiveAutomaton(obj.model, options);
        end

        function reachable_sets = compute_reachable_lane(obj, x, lanelet_ids)
            local_reachable_sets = obj.mpa.local_reachable_sets;
            lanelet_struct = obj.road_raw_data.lanelet;
            reachable_sets = get_reachable_sets(x(1), x(2), x(3), local_reachable_sets(1, :), cell(1, 3), obj.options);
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
