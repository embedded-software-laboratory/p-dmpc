function scenario = commonroad(options, vehicle_ids)
    % Commonroad_Scenario

    scenario = Scenario();

    % read from optionos
    scenario.options = options;

    options.is_allow_non_convex = true;

    % convert to char for cpp
    scenario.options.scenario_name = char(ScenarioType.commonroad);

    % get road data
    road_data = RoadDataCommonRoad().get_road_data();
    assignin('base', 'road_data', road_data);
    scenario.lanelets = road_data.lanelets;
    scenario.intersection_lanelets = road_data.intersection_lanelets;
    scenario.lanelet_boundary = road_data.lanelet_boundary;
    scenario.road_raw_data = road_data.road_raw_data;
    scenario.lanelet_relationships = road_data.lanelet_relationships;

    nVeh = options.amount;

    for iveh = 1:nVeh

        veh = Vehicle();

        if isempty(options.reference_path.lanelets_index)
            lanelets_index = [];
        else
            lanelets_index = options.reference_path.lanelets_index{iveh};
        end

        ref_path = generate_ref_path_loop(vehicle_ids(iveh), scenario.lanelets, lanelets_index); % function to generate refpath based on CPM Lab road geometry
        veh.lanelets_index = ref_path.lanelets_index;
        lanelet_ij = [ref_path.lanelets_index(1), ref_path.lanelets_index(end)];

        % check if the reference path is a loop
        lanelet_relationship = scenario.lanelet_relationships{min(lanelet_ij), max(lanelet_ij)};

        if ~isempty(lanelet_relationship) && strcmp(scenario.lanelet_relationships{min(lanelet_ij), max(lanelet_ij)}.type, LaneletRelationshipType.type_1)
            veh.is_loop = true;
        else
            veh.is_loop = false;
        end

        if isempty(options.reference_path.start_point)
            start_point = 1;
        else
            start_point = options.reference_path.start_point(iveh);
        end

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

    scenario.options.plot_limits = [0, 4.5; 0, 4];
    scenario.model = BicycleModel(veh.Lf, veh.Lr);

    if options.is_prioritized
        scenario.assignPrios = true;
    end

    scenario.mpa = MotionPrimitiveAutomaton(scenario.model, options);
    % initialize speed profile vector, currently 3 speed profiles are available
    scenario.speed_profile_mpas = [scenario.mpa, scenario.mpa, scenario.mpa];

    %     plot_local_reachable_sets(scenario.mpa, scenario.options.is_allow_non_convex)
end
