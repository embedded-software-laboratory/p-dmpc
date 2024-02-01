classdef Commonroad < Scenario

    methods

        function obj = Commonroad(amount, path_ids)
            obj = obj@Scenario();
            % get road data
            road_data = RoadDataCommonRoad().get_road_data();
            obj.lanelets = road_data.lanelets;
            obj.intersection_lanelets = road_data.intersection_lanelets;
            obj.lanelet_boundary = road_data.lanelet_boundary;
            obj.road_raw_data = road_data.road_raw_data;
            obj.lanelet_relationships = road_data.lanelet_relationships;
            obj.adjacency_lanelets = road_data.adjacency_lanelets;

            nVeh = amount;

            for iveh = 1:nVeh

                veh = Vehicle();

                lanelet_indices_loop = get_reference_lanelets_loop(path_ids(iveh));
                reference_path_struct = generate_reference_path_loop(lanelet_indices_loop, obj.lanelets); % function to generate refpath based on CPM Lab road geometry
                veh.lanelets_index = reference_path_struct.lanelets_index;
                lanelet_ij = [reference_path_struct.lanelets_index(1), reference_path_struct.lanelets_index(end)];

                % check if the reference path is a loop
                lanelet_relationship = obj.lanelet_relationships{min(lanelet_ij), max(lanelet_ij)};

                if ~isempty(lanelet_relationship) && obj.lanelet_relationships{min(lanelet_ij), max(lanelet_ij)}.type == LaneletRelationshipType.longitudinal
                    veh.is_loop = true;
                else
                    veh.is_loop = false;
                end

                veh.x_start = reference_path_struct.path(1, 1);
                veh.y_start = reference_path_struct.path(1, 2);

                veh.reference_path = reference_path_struct.path;

                veh.points_index = reference_path_struct.points_index;

                yaw = calculate_yaw(veh.reference_path);
                veh.yaw_start = yaw(1);
                obj.vehicles = [obj.vehicles, veh];
            end

        end

        function plot(obj, options, optional)

            arguments
                obj (1, 1) Commonroad;
                options (1, 1) Config;
                optional.fig (1, 1) matlab.ui.Figure = figure(Visible = "on");
            end

            plot@Scenario(obj, options, fig = optional.fig);

            % lanelets
            plot_lanelets(obj.road_raw_data.lanelet);

        end

    end

end
