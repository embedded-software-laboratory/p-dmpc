function scenario = lanelet2_scenario(amount, path_ids, scenario_type, plant)
    % Commonroad_Scenario

    scenario = Scenario();

    % get road data
    if scenario_type == ScenarioType.lab_default
        % ULA is required for that.
        assert(isa(plant, "UnifiedLabApi"));

        disp('Retrieve map from lab via unified lab API.')

        % Receive map via ULA interface
        map_as_string = plant.receive_map();

        % Store string in temporary file
        tmp_file_name = 'received_lanelet2_map_via_unifiedLabAPI.osm';
        writelines(map_as_string, [tempdir, filesep, tmp_file_name]);

        % Retrieve road data
        road_data = RoadDataLanelet2(false).get_road_data(tmp_file_name, tempdir);
    else
        assert(scenario_type == ScenarioType.lanelet2);

        disp('Create Lanelet2 scenario.')

        % Get road data from default location
        road_data = RoadDataLanelet2(false).get_road_data();
    end

    scenario.lanelets = road_data.lanelets;
    scenario.intersection_lanelets = road_data.intersection_lanelets;
    scenario.lanelet_boundary = road_data.lanelet_boundary;
    scenario.road_raw_data = road_data.road_raw_data;
    scenario.lanelet_relationships = road_data.lanelet_relationships;
    scenario.road_data_file_path = [road_data.road_folder_path, filesep, road_data.road_name];

    nVeh = amount;

    for iveh = 1:nVeh

        veh = Vehicle();

        % Generate a ref path using the Lanelet2 Interface and generate_reference_path_loop
        reference_path_loops = {Lanelet2_Interface.generate_reference_path_indices(scenario.road_data_file_path)};

        % FIXME lanelets_index is not passed to `generate_reference_path_loop` as of !179
        reference_path_loop = reference_path_loops{1};
        start_idx = mod(path_ids(iveh) * 2 - 1, width(reference_path_loop));

        if start_idx == 1
            lanelets_index = reference_path_loop;
        else
            lanelets_index = [reference_path_loop(start_idx:end), reference_path_loop(1:start_idx - 1)];
        end

        reference_path_struct = generate_reference_path_loop(path_ids(iveh), scenario.lanelets, lanelets_index);
        veh.lanelets_index = reference_path_struct.lanelets_index;
        lanelet_ij = [reference_path_struct.lanelets_index(1), reference_path_struct.lanelets_index(end)];

        % check if the reference path is a loop
        lanelet_relationship = scenario.lanelet_relationships{min(lanelet_ij), max(lanelet_ij)};

        if ~isempty(lanelet_relationship) && scenario.lanelet_relationships{min(lanelet_ij), max(lanelet_ij)}.type == LaneletRelationshipType.longitudinal
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
        scenario.vehicles = [scenario.vehicles, veh];
    end

    scenario.model = BicycleModel(veh.Lf, veh.Lr);

end
