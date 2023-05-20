function scenario = lanelet2_ids3c_circle_scenario(options, vehicle_ids, plant)
    % Commonroad_Scenario

    scenario = Scenario();

    options.recursive_feasibility = true;
    % read from options
    scenario.options = options;

    options.is_allow_non_convex = true;

    % get road data
    % if strcmp(options.scenario_name, 'Lab_default')
    %     % ULA is required for that.
    %     assert(isa(plant, "UnifiedLabApi"));
    %
    %     disp('Retrieve map from lab via unified lab API.')
    %
    %     % Receive map via ULA interface
    %     map_as_string = plant.receive_map();
    %
    %     % Store string in temporary file
    %     tmp_file_name = 'received_lanelet2_map_via_unifiedLabAPI.osm';
    %     writelines(map_as_string, [tempdir, filesep, tmp_file_name]);
    %
    %     % Retrieve road data
    %     road_data = RoadDataLanelet2(false).get_road_data(tmp_file_name, tempdir);
    % elseif strcmp(options.scenario_name, 'Lanelet2')
    %     assert(strcmp(options.scenario_name, 'Lanelet2'));
    %
    %     disp('Create Lanelet2 scenario.')
    %
    %     % Get road data from default location
    %     road_data = RoadDataLanelet2(false).get_road_data();
    % else
    assert(options.scenario_type == ScenarioType.lanelet2_ids3c_circle);

    disp('Create Lanelet2_IDS3C_circle scenario.')
    file_name = 'IDS3C_Map_circle.osm';

    % Get road data from default location
    road_data = RoadDataLanelet2(false).get_road_data(file_name);
    % end

    assignin('base', 'road_data_lab_default_scenario', road_data);
    scenario.lanelets = road_data.lanelets;
    scenario.intersection_lanelets = road_data.intersection_lanelets;
    scenario.lanelet_boundary = road_data.lanelet_boundary;
    scenario.road_raw_data = road_data.road_raw_data;
    scenario.lanelet_relationships = road_data.lanelet_relationships;
    scenario.road_data_file_path = [road_data.road_folder_path, filesep, road_data.road_name];

    nVeh = options.amount;

    paths = [5332,2488,2396; 2396,2488,2396; 5311,2488,2396];

    for iveh = 1:nVeh

        veh = Vehicle();
        veh.trim_config = 1;

        % Generate a ref path using the Lanelet2 Interface and generate_ref_path_loop
        % ref_path_loops = {Lanelet2_Interface.generate_lanelet2_ref_path_separate_segments_indices(scenario.road_data_file_path, paths(iveh,:))};
        lanelets_index = Lanelet2_Interface.generate_lanelet2_ref_path_separate_segments_indices(scenario.road_data_file_path, paths(iveh,:));
        % ref_path_loop = ref_path_loops{1};
        % lanelets_index = ref_path_loop;

        % if isempty(options.reference_path.lanelets_index)
        %     ref_path_loop = ref_path_loops{1};
        %     start_idx = mod(vehicle_ids(iveh) * 2 - 1, width(ref_path_loop));
        % 
        %     if start_idx == 1
        %         lanelets_index = ref_path_loop;
        %     else
        %         lanelets_index = [ref_path_loop(start_idx:end), ref_path_loop(1:start_idx - 1)];
        %     end
        % 
        % else
        %     lanelets_index = options.reference_path.lanelets_index{iveh};
        % end

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

        % if isempty(options.reference_path.start_point)
        start_point = 1;
        % else
        %     start_point = options.reference_path.start_point(iveh);
        % end

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

    scenario.options.plot_limits = [-2, 3; -2, 3];
    scenario.model = BicycleModel(veh.Lf, veh.Lr);

    if options.is_prioritized
        scenario.assignPrios = true;
    end

    scenario.mpa = MotionPrimitiveAutomaton(scenario.model, options);

    % initialize speed profile vector, currently 3 speed profiles are available
    scenario.speed_profile_mpas = [scenario.mpa, scenario.mpa, scenario.mpa];

    %     plot_local_reachable_sets(scenario.mpa, scenario.options.is_allow_non_convex)
end
