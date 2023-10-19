classdef IterationData
    %ITERATIONDATA  Class as data structure for data that changes in every iteration during the experiment.

    properties
        reference_trajectory_points
        reference_trajectory_index
        k % current time/simulation step
        x0 % state
        trim_indices % current trim
        v_ref % reference speed
        predicted_lanelets % vehicle's predicted lanelets
        predicted_lanelet_boundary % first column for left boundary, second column for right boundary, third column for MATLAB polyshape instance
        reachable_sets % cells to store instances of MATLAB calss `polyshape`
        hdv_reachable_sets % reachable sets of hdvs
        occupied_areas % currently occupied areas with normal offset of vehicles
        emergency_maneuvers % occupied area of emergency braking maneuver
        last_trajectory_index % initial trajectory index
        adjacency % (nVeh x nVeh) matrix, entry is 1 if two vehicles drive in two adjacent lanelets and their distance are smaller enough
        hdv_adjacency % (nCAV x nHDV) matrix, entry (i,j) is 1 if CAV i is next to or in behind of HDV j
        obstacles
        dynamic_obstacle_area
        dynamic_obstacle_shape
        dynamic_obstacle_fullres
        dynamic_obstacle_reachableSets
        vehicle_to_lanelet
        weighted_coupling % (nVeh x nVeh) matrix, coupling weights of all coupling vehicle pair; higher value indicates stronger coupling
        weighted_coupling_reduced % reduced coupling weights by forbidding vehicles entering their lanelet crossing areas
        coupling_info % couling information of each coupling pair
        belonging_vector % a column vector whose value indicate which group each vehicle belongs to
        parl_groups_info % struct, store information of parallel groups
        directed_coupling % nVeh-by-nVeh matrix, entry if 1 if the corresponding two vehicles are coupled
        directed_coupling_reduced % nVeh-by-nVeh matrix, reduced directed adjacency by forbidding vehicles entering their lanelet crossing area
        priority_list
        lanelet_crossing_areas
        vehicles % copy of vehicle list (useful for filtered)
        vehicle_ids
        amount % number of involved vehicles (useful for filtered)
    end

    methods

        function obj = IterationData(scenario, k, vehicle_ids)
            nVeh = scenario.options.amount;
            Hp = scenario.options.Hp;
            hdv_amount = scenario.options.manual_control_config.amount;
            obj.k = k;
            obj.reference_trajectory_points = zeros(nVeh, Hp, 2);
            obj.reference_trajectory_index = zeros(nVeh, Hp, 1);
            obj.x0 = zeros(nVeh, 4);
            obj.trim_indices = zeros(nVeh, 1);
            obj.v_ref = zeros(nVeh, Hp);
            obj.predicted_lanelets = cell(nVeh, 1);
            obj.predicted_lanelet_boundary = cell(nVeh, 3);
            obj.reachable_sets = cell(nVeh, Hp);
            obj.hdv_reachable_sets = cell(hdv_amount, Hp);
            obj.occupied_areas = cell(nVeh, 1);
            obj.emergency_maneuvers = cell(nVeh, 1);
            obj.last_trajectory_index = ones(nVeh, 1) * 10;
            obj.adjacency = zeros(nVeh, nVeh);
            obj.hdv_adjacency = zeros(nVeh, hdv_amount);
            obj.dynamic_obstacle_area = scenario.dynamic_obstacle_area;
            obj.dynamic_obstacle_shape = {};
            obj.dynamic_obstacle_fullres = {};
            obj.dynamic_obstacle_reachableSets = {};
            obj.vehicle_to_lanelet = zeros(nVeh, 1);
            obj.directed_coupling = zeros(nVeh, nVeh);
            obj.directed_coupling_reduced = zeros(nVeh, nVeh);
            obj.weighted_coupling = zeros(nVeh, nVeh);
            obj.weighted_coupling_reduced = zeros(nVeh, nVeh);
            obj.priority_list = ones(nVeh, 1);
            obj.belonging_vector = ones(nVeh, 1);
            obj.obstacles = scenario.obstacles;
            obj.lanelet_crossing_areas = cell(nVeh, 1);
            obj.amount = nVeh;
            obj.vehicles = scenario.vehicles;
            obj.vehicle_ids = vehicle_ids;
            obj.coupling_info = cell(nVeh, nVeh);
        end

    end

end
