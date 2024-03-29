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
        lane_change_lanes % positions of the lane before the lane change and the lane change lane in the lanelet vector
        lane_change_indices % indices of the trajectory points of the lane before the lane change and the lane change lane
        lanes_before_update % lanes before path has been automatically updated in CPM Lab mode
        auto_updated_path % set in rhc_init to memorize when path is updated automatically
        adjacency % (nVeh x nVeh) matrix, entry is 1 if two vehicles drive in two adjacent lanelets and their distance are smaller enough
        hdv_adjacency % (nCAV x nHDV) matrix, entry (i,j) is 1 if CAV i is next to or in behind of HDV j
        obstacles
        dynamic_obstacle_area
        dynamic_obstacle_shape
        dynamic_obstacle_fullres
        dynamic_obstacle_reachableSets
        vehicle_to_lanelet
        coupling_weights % (nVeh x nVeh) matrix, coupling weights of all coupling vehicle pair; higher value indicates stronger coupling
        coupling_weights_optimal % "optimal" coupling weights
        coupling_weights_reduced % reduced coupling weights by forbidding vehicles entering their lanelet crossing areas
        coupling_weights_random % random coupling weights
        coupling_info % couling information of each coupling pair
        num_couplings_between_grps % number of couplings between parallel groups
        num_couplings_between_grps_ignored % reduced number of couplings between groups by using lanelet crossing lanelets
        belonging_vector % a column vector whose value indicate which group each vehicle belongs to
        parl_groups_info % struct, store information of parallel groups
        directed_coupling % nVeh-by-nVeh matrix, entry if 1 if the corresponding two vehicles are coupled
        directed_coupling_reduced % nVeh-by-nVeh matrix, reduced directed adjacency by forbidding vehicles entering their lanelet crossing area
        last_vehs_at_intersection % store information about which vehicles were at the intersection in the last time step
        time_enter_intersection % time step when vehicle enters the intersection
        priority_list
        lanelet_crossing_areas
        timer % struct, used to store computation time of different parts
        vehicles % copy of vehicle list (useful for filtered)
        amount % number of involved vehicles (useful for filtered)
    end

    methods

        function obj = IterationData(scenario, k)
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
            obj.lane_change_lanes = zeros(nVeh, 10, 2);
            obj.lane_change_indices = zeros(nVeh, 10, 4);
            obj.lanes_before_update = zeros(nVeh, 1, 2);
            obj.auto_updated_path = false(nVeh, 1);
            obj.adjacency = zeros(nVeh, nVeh);
            obj.hdv_adjacency = zeros(nVeh, hdv_amount);
            obj.dynamic_obstacle_area = scenario.dynamic_obstacle_area;
            obj.dynamic_obstacle_shape = scenario.dynamic_obstacle_shape;
            obj.dynamic_obstacle_fullres = scenario.dynamic_obstacle_fullres;
            obj.dynamic_obstacle_reachableSets = scenario.dynamic_obstacle_reachableSets;
            obj.vehicle_to_lanelet = zeros(nVeh, 1);
            obj.coupling_weights = zeros(nVeh, nVeh);
            obj.coupling_weights_optimal = zeros(nVeh, nVeh);
            obj.coupling_weights_reduced = zeros(nVeh, nVeh);
            obj.coupling_weights_random = zeros(nVeh, nVeh);
            obj.directed_coupling = zeros(nVeh, nVeh);
            obj.directed_coupling_reduced = zeros(nVeh, nVeh);
            obj.last_vehs_at_intersection = [];
            obj.time_enter_intersection = [];
            obj.priority_list = ones(nVeh, 1);
            obj.belonging_vector = zeros(nVeh, 1);
            obj.obstacles = scenario.obstacles;
            obj.lanelet_crossing_areas = {};
            obj.amount = nVeh;
            obj.vehicles = scenario.vehicles;
        end

    end

end
