function scenario = lanelet2_scenario(options, filepath_lanelet2_map)
    % Lanelet2_Scenario
    % FIXME Should be a class; Avoid code duplication with Commonroad

    scenario = Scenario(options);

    if nargin > 2
        % ULA is required for that.
        assert(isa(plant, "UnifiedLabApi"));

        % Store string in temporary file
        tmp_file_name = 'received_lanelet2_map_via_unifiedLabAPI.osm';
        writelines(filepath_lanelet2_map, [tempdir, filesep, tmp_file_name]);

        % Retrieve road data
        road_data = RoadDataLanelet2(false).get_road_data(tmp_file_name, tempdir);
    else
        % Get road data from default location
        road_data = RoadDataLanelet2(false).get_road_data();
    end

    scenario.lanelets = road_data.lanelets;
    scenario.intersection_lanelets = road_data.intersection_lanelets;
    scenario.lanelet_boundary = road_data.lanelet_boundary;
    scenario.road_raw_data = road_data.road_raw_data;
    scenario.lanelet_relationships = road_data.lanelet_relationships;
    scenario.road_data_file_path = [road_data.road_folder_path, filesep, road_data.road_name];
    scenario.adjacency_lanelets = road_data.adjacency_lanelets;

    nVeh = options.amount;

    rand_stream = RandStream("mt19937ar", "Seed", sum(options.path_ids));

    for iveh = 1:nVeh
        % Generate a ref path using the Lanelet2 Interface and generate_reference_path_loop
        reference_path_loops = {Lanelet2_Interface.generate_reference_path_indices(scenario.road_data_file_path)};
        % take loop from all defined loops
        reference_path_loop = reference_path_loops{1};
        % define starting lanelet
        start_idx = mod(options.path_ids(iveh) * 2 - 1, width(reference_path_loop));
        % shift reference_path_loop depending on start_idx
        lanelets_index = [reference_path_loop(start_idx:end), reference_path_loop(1:start_idx - 1)];

        reference_path_struct = generate_reference_path_loop(lanelets_index, scenario.lanelets);
        scenario.vehicles(iveh).lanelets_index = reference_path_struct.lanelets_index;
        lanelet_ij = [reference_path_struct.lanelets_index(1), reference_path_struct.lanelets_index(end)];

        % check if the reference path is a loop
        lanelet_relationship = scenario.lanelet_relationships{min(lanelet_ij), max(lanelet_ij)};

        if ~isempty(lanelet_relationship) && scenario.lanelet_relationships{min(lanelet_ij), max(lanelet_ij)}.type == LaneletRelationshipType.longitudinal
            scenario.vehicles(iveh).is_loop = true;
        else
            scenario.vehicles(iveh).is_loop = false;
        end

        scenario.vehicles(iveh).x_start = reference_path_struct.path(1, 1);
        scenario.vehicles(iveh).y_start = reference_path_struct.path(1, 2);

        scenario.vehicles(iveh).reference_path = reference_path_struct.path;

        scenario.vehicles(iveh).points_index = reference_path_struct.points_index;

        yaw = calculate_yaw(scenario.vehicles(iveh).reference_path);
        scenario.vehicles(iveh).yaw_start = yaw(1);

        % set a random speed level in mpa as reference speed
        straight_speeds = MotionPrimitiveAutomaton(options).get_straight_speeds_of_mpa();
        scenario.vehicles(iveh).reference_speed = straight_speeds(randi(rand_stream, numel(straight_speeds)));
    end

end
