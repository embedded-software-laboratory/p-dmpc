classdef Commonroad < Scenario

    methods

        function obj = Commonroad(options)
            obj = obj@Scenario(options);
            % get road data
            road_data = RoadDataCommonRoad().get_road_data();
            obj.lanelets = road_data.lanelets;
            obj.intersection_lanelets = road_data.intersection_lanelets;
            obj.lanelet_boundary = road_data.lanelet_boundary;
            obj.road_raw_data = road_data.road_raw_data;
            obj.lanelet_relationships = road_data.lanelet_relationships;
            obj.adjacency_lanelets = road_data.adjacency_lanelets;

            nVeh = options.amount;

            rand_stream = RandStream("mt19937ar", "Seed", sum(options.path_ids));
            straight_speeds = MotionPrimitiveAutomaton(options).get_straight_speeds_of_mpa();

            for iveh = 1:nVeh
                lanelet_indices_loop = get_reference_lanelets_loop(options.path_ids(iveh));
                reference_path_struct = generate_reference_path_loop(lanelet_indices_loop, obj.lanelets); % function to generate refpath based on CPM Lab road geometry
                obj.vehicles(iveh).lanelets_index = reference_path_struct.lanelets_index;
                lanelet_ij = [reference_path_struct.lanelets_index(1), reference_path_struct.lanelets_index(end)];

                % check if the reference path is a loop
                lanelet_relationship = obj.lanelet_relationships{min(lanelet_ij), max(lanelet_ij)};

                if ~isempty(lanelet_relationship) && obj.lanelet_relationships{min(lanelet_ij), max(lanelet_ij)}.type == LaneletRelationshipType.longitudinal
                    obj.vehicles(iveh).is_loop = true;
                else
                    obj.vehicles(iveh).is_loop = false;
                end

                obj.vehicles(iveh).reference_path = reference_path_struct.path;
                obj.vehicles(iveh).points_index = reference_path_struct.points_index;

                [ ...
                     obj.vehicles(iveh).x_start, ...
                     obj.vehicles(iveh).y_start, ...
                     obj.vehicles(iveh).yaw_start] = obj.start_pose(iveh, options);

                % set a random speed level in mpa as reference speed
                obj.vehicles(iveh).reference_speed = straight_speeds(randi(rand_stream, numel(straight_speeds)));
            end

        end

        function [x, y, yaw] = start_pose(obj, iveh, options)

            arguments
                obj (1, 1) Commonroad;
                iveh (1, 1) double;
                options (1, 1) Config;
            end

            if ~isempty(options.start_poses)
                x = options.start_poses(1, iveh);
                y = options.start_poses(2, iveh);
                yaw = options.start_poses(3, iveh);
            else
                x = obj.vehicles(iveh).reference_path(1, 1);
                y = obj.vehicles(iveh).reference_path(1, 2);
                yaw_trajectory = calculate_yaw(obj.vehicles(iveh).reference_path);
                yaw = yaw_trajectory(1);
            end

        end

        function plot(obj, optional)

            arguments
                obj (1, 1) Commonroad;
                optional.fig (1, 1) matlab.ui.Figure = figure(Visible = "on");
            end

            plot@Scenario(obj, fig = optional.fig);

            % lanelets
            plot_lanelets(obj.road_raw_data.lanelet);

        end

        function plot_with_lanelet_ids(obj)

            arguments
                obj (1, 1) Commonroad;
            end

            fig = gcf;
            obj.plot(fig = fig);

            for i_lanelet = 1:numel(obj.lanelets)
                lanelet = obj.lanelets{i_lanelet};
                n_half = floor(size(lanelet, 1) / 2);
                text( ...
                    lanelet(n_half, LaneletInfo.cx) ...
                    , lanelet(n_half, LaneletInfo.cy) ...
                    , num2str(i_lanelet) ...
                    , FontSize = 10 ...
                    , HorizontalAlignment = "center" ...
                );
            end

        end

    end

end
