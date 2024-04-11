classdef Lanelet2 < Scenario

    methods

        function obj = Lanelet2(options, filepath_lanelet2_map)

            arguments
                options (1, 1) Config
                filepath_lanelet2_map (1, :) char = [];
            end

            obj = obj@Scenario(options);

            if isempty(filepath_lanelet2_map)
                % Get road data from default location
                road_data = RoadDataLanelet2(false).get_road_data();
            else
                % UTI is required for that.
                assert(isa(plant, "UnifiedTestbedInterface"));

                % Store string in temporary file
                tmp_file_name = 'received_lanelet2_map_via_unifiedTestbedInterface.osm';
                writelines(filepath_lanelet2_map, [tempdir, filesep, tmp_file_name]);

                % Retrieve road data
                road_data = RoadDataLanelet2(false).get_road_data(tmp_file_name, tempdir);
            end

            obj.lanelets = road_data.lanelets;
            obj.intersection_lanelets = road_data.intersection_lanelets;
            obj.lanelet_boundary = road_data.lanelet_boundary;
            obj.road_raw_data = road_data.road_raw_data;
            obj.lanelet_relationships = road_data.lanelet_relationships;
            obj.adjacency_lanelets = road_data.adjacency_lanelets;
            obj.road_data_file_path = [road_data.road_folder_path, filesep, road_data.road_name];

            nVeh = options.amount;

            rand_stream = RandStream("mt19937ar", "Seed", sum(options.path_ids));
            straight_speeds = MotionPrimitiveAutomaton(options).get_straight_speeds_of_mpa();

            for iveh = 1:nVeh
                % Generate a ref path using the Lanelet2 Interface and generate_reference_path_loop
                reference_path_loops = {Lanelet2_Interface.generate_reference_path_indices(obj.road_data_file_path)};
                % take loop from all defined loops
                reference_path_loop = reference_path_loops{1};
                % define starting lanelet
                start_idx = mod(options.path_ids(iveh) * 2 - 1 + 8, width(reference_path_loop));
                % shift reference_path_loop depending on start_idx
                lanelets_index = [reference_path_loop(start_idx:end), reference_path_loop(1:start_idx - 1)];

                reference_path_struct = generate_reference_path_loop(lanelets_index, obj.lanelets);
                obj.vehicles(iveh).lanelets_index = reference_path_struct.lanelets_index;
                lanelet_ij = [reference_path_struct.lanelets_index(1), reference_path_struct.lanelets_index(end)];

                % check if the reference path is a loop
                lanelet_relationship = obj.lanelet_relationships{min(lanelet_ij), max(lanelet_ij)};

                if ~isempty(lanelet_relationship) && obj.lanelet_relationships{min(lanelet_ij), max(lanelet_ij)}.type == LaneletRelationshipType.longitudinal
                    obj.vehicles(iveh).is_loop = true;
                else
                    obj.vehicles(iveh).is_loop = false;
                end

                obj.vehicles(iveh).x_start = reference_path_struct.path(1, 1);
                obj.vehicles(iveh).y_start = reference_path_struct.path(1, 2);

                obj.vehicles(iveh).reference_path = reference_path_struct.path;

                obj.vehicles(iveh).points_index = reference_path_struct.points_index;

                yaw = calculate_yaw(obj.vehicles(iveh).reference_path);
                obj.vehicles(iveh).yaw_start = yaw(1);

                % set a random speed level in mpa as reference speed
                obj.vehicles(iveh).reference_speed = straight_speeds(randi(rand_stream, numel(straight_speeds)));
            end

        end

        function plot(obj, optional)

            arguments
                obj (1, 1) Lanelet2;
                optional.fig (1, 1) matlab.ui.Figure = figure(Visible = "on");
            end

            plot@Scenario(obj, fig = optional.fig);

            % lanelets
            plot_lanelets(obj.road_raw_data.lanelet);
        end

    end

end
