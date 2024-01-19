classdef IterationData
    %ITERATIONDATA  Class as data structure for data that changes in every iteration during the experiment.

    properties
        reference_trajectory_points (:, :, 2) double % n_vehicles x Hp x (px,py)
        reference_trajectory_index
        priority_permutation % permuation index for multiple priorities
        x0 (:, 4) % state
        trim_indices % current trim
        v_ref % reference speed
        current_lanelet % vehicle's current lanelet
        predicted_lanelets % vehicle's predicted lanelets
        predicted_lanelet_boundary % first column for left boundary, second column for right boundary, third column for MATLAB polyshape instance

        occupied_areas % currently occupied areas with normal offset of vehicles
        emergency_maneuvers % occupied area of emergency braking maneuver
        reachable_sets % cells to store instances of MATLAB calss `polyshape`

        obstacles
        dynamic_obstacle_area % (nObs x Hp) cell of areas [x; y]
        lanelet_crossing_areas

        adjacency % (nVeh x nVeh) matrix, entry is 1 if two vehicles drive in two adjacent lanelets and their distance are smaller enough
        weighted_coupling % (nVeh x nVeh) matrix, coupling weights of all coupling vehicle pair; higher value indicates stronger coupling
        coupling_info % couling information of each coupling pair
        directed_coupling % nVeh-by-nVeh matrix, entry if 1 if the corresponding two vehicles are coupled
        directed_coupling_reduced % nVeh-by-nVeh matrix, reduced directed adjacency by forbidding vehicles entering their lanelet crossing area
        directed_coupling_sequential % nVeh-by-nVeh matrix, after graph partitioning

        hdv_reachable_sets % reachable sets of hdvs
        hdv_adjacency % (nCAV x nHDV) matrix, entry (i,j) is 1 if CAV i is next to or in behind of HDV j

        amount % number of total vehicles / controlled vehicles
    end

    methods

        function obj = IterationData(options, scenario)
            nVeh = options.amount;
            Hp = options.Hp;
            hdv_amount = options.manual_control_config.amount;
            obj.priority_permutation = 0;
            obj.reference_trajectory_points = zeros(nVeh, Hp, 2);
            obj.reference_trajectory_index = zeros(nVeh, Hp, 1);
            obj.x0 = zeros(nVeh, 4);
            obj.trim_indices = zeros(nVeh, 1);
            obj.v_ref = zeros(nVeh, Hp);
            obj.current_lanelet = zeros(nVeh, 1);
            obj.predicted_lanelets = cell(nVeh, 1);
            obj.predicted_lanelet_boundary = cell(nVeh, 3);
            obj.reachable_sets = cell(nVeh, Hp);
            obj.hdv_reachable_sets = cell(hdv_amount, Hp);
            obj.occupied_areas = cell(nVeh, 1);
            obj.emergency_maneuvers = cell(nVeh, 1);
            obj.adjacency = zeros(nVeh, nVeh);
            obj.hdv_adjacency = zeros(nVeh, hdv_amount);
            obj.dynamic_obstacle_area = scenario.dynamic_obstacle_area;
            obj.directed_coupling = zeros(nVeh, nVeh);
            obj.directed_coupling_reduced = zeros(nVeh, nVeh);
            obj.directed_coupling_sequential = zeros(nVeh, nVeh);
            obj.weighted_coupling = zeros(nVeh, nVeh);
            obj.obstacles = scenario.obstacles;
            obj.lanelet_crossing_areas = repmat({{}}, nVeh, 1);
            obj.amount = nVeh;
            obj.coupling_info = cell(nVeh, nVeh);
        end

        function equal = is_equal(obj, compare_obj)

            arguments
                obj (1, 1) IterationData;
                compare_obj (1, 1) IterationData;
            end

            equal = obj.amount == compare_obj.amount;

            for i_veh = 1:obj.amount
                equal = equal && ~any(abs(obj.x0(i_veh) - compare_obj.x0(i_veh)));

                if (~equal)
                    return;
                end

            end

        end

    end

    methods (Static)

        function cleaned_object = clean(object)

            arguments
                object (1, 1) IterationData
            end

            cleaned_object = object;
            cleaned_object.predicted_lanelets = [];
            cleaned_object.predicted_lanelet_boundary = [];
            cleaned_object.emergency_maneuvers = [];
            cleaned_object.occupied_areas = [];
            cleaned_object.priority_permutation = [];
            cleaned_object.current_lanelet = [];
            cleaned_object.dynamic_obstacle_area = [];
            cleaned_object.coupling_info = [];
            cleaned_object.hdv_reachable_sets = [];
            cleaned_object.hdv_adjacency = [];
        end

    end

end
