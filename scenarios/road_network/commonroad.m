function scenario = commonroad(amount, path_ids)
    % Commonroad_Scenario

    scenario = Scenario();

    % get road data
    road_data = RoadDataCommonRoad().get_road_data();
    scenario.lanelets = road_data.lanelets;
    scenario.intersection_lanelets = road_data.intersection_lanelets;
    scenario.lanelet_boundary = road_data.lanelet_boundary;
    scenario.road_raw_data = road_data.road_raw_data;
    scenario.lanelet_relationships = road_data.lanelet_relationships;

    nVeh = amount;

    for iveh = 1:nVeh

        veh = Vehicle();

        reference_path_struct = generate_reference_path_loop(path_ids(iveh), scenario.lanelets); % function to generate refpath based on CPM Lab road geometry
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
