classdef IterationData
    %ITERATIONDATA  Class as data structure for data that changes in every iteration during the experiment.

    properties
        reference_trajectory_points (:, :, 2) double % n_vehicles x Hp x (px,py)

        priority_permutation % permutation index for multiple priorities
        x0 (:, 4) % state: (x, y, yaw, speed)
        trim_indices % current trim
        v_ref % reference speed
        current_lanelet % vehicle's current lanelet
        predicted_lanelets % vehicle's predicted lanelets
        predicted_lanelet_boundary % first column for left boundary, second column for right boundary, third column for MATLAB polyshape instance

        occupied_areas % currently occupied areas with normal offset of vehicles
        reachable_sets % cells to store instances of MATLAB class `polyshape`

        obstacles
        dynamic_obstacle_area % (nObs x Hp) cell of areas [x; y]

        adjacency (:, :) logical % (nVeh x nVeh) matrix, entry is 1 if two vehicles drive in two adjacent lanelets and their distance are smaller enough
        weighted_coupling (:, :) double % (nVeh x nVeh) matrix, coupling weights of all coupling vehicle pair; higher value indicates stronger coupling
        directed_coupling (:, :) logical % nVeh-by-nVeh matrix, entry if 1 if the corresponding two vehicles are coupled
        directed_coupling_sequential (:, :) logical % nVeh-by-nVeh matrix, after graph partitioning

        fallbacks (:, 1) logical % n_vehicles x 1, true if fallback

        hdv_reachable_sets % reachable sets of hdvs
        hdv_adjacency % (nCAV x nHDV) matrix, entry (i,j) is 1 if CAV i is next to or in behind of HDV j

        amount % number of total vehicles / controlled vehicles
    end

    properties (Dependent)
        number_of_computation_levels
    end

    methods

        function obj = IterationData(options, scenario)
            nVeh = options.amount;
            Hp = options.Hp;
            hdv_amount = options.manual_control_config.amount;
            obj.priority_permutation = 0;
            obj.reference_trajectory_points = zeros(nVeh, Hp, 2);
            obj.x0 = zeros(nVeh, 4);
            obj.trim_indices = zeros(nVeh, 1);
            obj.v_ref = zeros(nVeh, Hp);
            obj.current_lanelet = zeros(nVeh, 1);
            obj.predicted_lanelets = cell(nVeh, 1);
            obj.predicted_lanelet_boundary = cell(nVeh, 3);
            obj.reachable_sets = cell(nVeh, Hp);
            obj.hdv_reachable_sets = cell(hdv_amount, Hp);
            obj.occupied_areas = cell(nVeh, 1);
            obj.adjacency = zeros(nVeh, nVeh);
            obj.hdv_adjacency = zeros(nVeh, hdv_amount);
            obj.dynamic_obstacle_area = scenario.dynamic_obstacle_area;
            obj.directed_coupling = zeros(nVeh, nVeh);
            obj.directed_coupling_sequential = zeros(nVeh, nVeh);
            obj.weighted_coupling = zeros(nVeh, nVeh);
            obj.obstacles = scenario.obstacles;
            obj.amount = nVeh;
            obj.fallbacks = false(nVeh, 1);
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

        function result = get.number_of_computation_levels(obj)
            result = size(kahn(obj.directed_coupling_sequential), 1);
        end

    end

    methods (Static)

        function filtered_object = filter(object, vehicle_filter)

            arguments
                object (1, 1) IterationData
                vehicle_filter (1, :) logical
            end

            % FILTER   Reduce iter structure to selected vehicles.

            filtered_object = object;
            filtered_object.x0 = filtered_object.x0(vehicle_filter, :);
            filtered_object.trim_indices = filtered_object.trim_indices(vehicle_filter);
            filtered_object.v_ref = filtered_object.v_ref(vehicle_filter, :);
            filtered_object.reference_trajectory_points = filtered_object.reference_trajectory_points(vehicle_filter, :, :);
            filtered_object.predicted_lanelets = filtered_object.predicted_lanelets(vehicle_filter);
            filtered_object.predicted_lanelet_boundary = filtered_object.predicted_lanelet_boundary(vehicle_filter, :);
            filtered_object.reachable_sets = filtered_object.reachable_sets(vehicle_filter, :);
            filtered_object.amount = sum(vehicle_filter);
            filtered_object.hdv_adjacency = filtered_object.hdv_adjacency(vehicle_filter, :);
        end

        function cleaned_object = clean(object)

            arguments
                object (1, 1) IterationData
            end

            cleaned_object = object;
            cleaned_object.predicted_lanelets = [];
            cleaned_object.predicted_lanelet_boundary = [];
            cleaned_object.occupied_areas = [];
            cleaned_object.priority_permutation = [];
            cleaned_object.current_lanelet = [];
            cleaned_object.dynamic_obstacle_area = [];
            cleaned_object.hdv_reachable_sets = [];
            cleaned_object.hdv_adjacency = [];
            cleaned_object.v_ref = [];
            cleaned_object.reachable_sets = {};
            cleaned_object.fallbacks = [];
        end

    end

end
