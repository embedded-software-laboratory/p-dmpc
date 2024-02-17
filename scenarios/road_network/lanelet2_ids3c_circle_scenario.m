function scenario = lanelet2_ids3c_circle_scenario(options, vehicle_ids, plant)
    % Commonroad_Scenario

    scenario = Scenario();

    options.recursive_feasibility = true;
    % read from options
    scenario.options = options;

    options.is_allow_non_convex = true;

    disp('Create Lanelet2_IDS3C_circle scenario.')

    if options.scenario_type == ScenarioType.lanelet2_ids3c_circle
        file_name = 'IDS3C_Map_Circle_18.osm';
        scenario.options.plot_limits = [-0.5, 4.2; -2, 2];
    else
        assert(options.scenario_type == ScenarioType.lanelet2_ids3c_circle_in_cpm);
        file_name = 'IDS3C_Map_Circle_18_Translated_To_CPM_Mirrored.osm';
        scenario.options.plot_limits = [0, 4.5; 0, 4];
    end

    % Get road data from default location
    road_data = RoadDataLanelet2(false).get_road_data(file_name);
    % end

    assignin('base', 'road_data_testbed_default_scenario', road_data);
    scenario.lanelets = road_data.lanelets;
    scenario.intersection_lanelets = road_data.intersection_lanelets;
    scenario.lanelet_boundary = road_data.lanelet_boundary;
    scenario.road_raw_data = road_data.road_raw_data;
    scenario.lanelet_relationships = road_data.lanelet_relationships;
    scenario.road_data_file_path = [road_data.road_folder_path, filesep, road_data.road_name];

    nVeh = options.amount;

    paths = [5332, 2488, 2396; 2396, 2488, 2396; 5311, 2488, 2396];

    for iveh = 1:nVeh

        veh = Vehicle();
        veh.trim_config = 1;

        % Generate a ref path using the Lanelet2 Interface and generate_ref_path_loop
        lanelets_index = Lanelet2_Interface.generate_lanelet2_ref_path_separate_segments_indices(scenario.road_data_file_path, paths(iveh, :));

        ref_path = generate_ref_path(vehicle_ids(iveh), scenario.lanelets, lanelets_index);
        veh.lanelets_index = ref_path.lanelets_index;
        lanelet_ij = [ref_path.lanelets_index(1), ref_path.lanelets_index(end)];

        % check if the reference path is a loop
        lanelet_relationship = scenario.lanelet_relationships{min(lanelet_ij), max(lanelet_ij)};

        if ~isempty(lanelet_relationship) && scenario.lanelet_relationships{min(lanelet_ij), max(lanelet_ij)}.type == LaneletRelationshipType.longitudinal
            veh.is_loop = true;
        else
            veh.is_loop = false;
        end

        start_point = 1;

        veh.x_start = ref_path.path(start_point, 1);
        veh.y_start = ref_path.path(start_point, 2);
        veh.x_goal = ref_path.path([start_point + 1:end, 1:start_point - 1], 1);
        veh.y_goal = ref_path.path([start_point + 1:end, 1:start_point - 1], 2);

        veh.reference_trajectory = [veh.x_start veh.y_start
                                    veh.x_goal veh.y_goal];

        veh.points_index = ref_path.points_index - start_point + 1;

        yaw = calculate_yaw(veh.reference_trajectory);
        veh.yaw_start = yaw(1);
        veh.yaw_goal = yaw(2:end);
        scenario.vehicles = [scenario.vehicles, veh];
    end

    scenario.model = BicycleModel(veh.Lf, veh.Lr);

    if options.is_prioritized
        scenario.assignPrios = true;
    end

    scenario.mpa = MotionPrimitiveAutomaton(scenario.model, options);

    % initialize speed profile vector, currently 3 speed profiles are available
    scenario.speed_profile_mpas = [scenario.mpa, scenario.mpa, scenario.mpa];

    %     plot_local_reachable_sets(scenario.mpa, scenario.options.is_allow_non_convex)
end
