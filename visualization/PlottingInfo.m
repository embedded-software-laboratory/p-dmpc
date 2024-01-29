classdef PlottingInfo

    properties
        x0 (3, :) % (x, y, yaw) x n_vehicles
        % predicted trajectory including x0, (x, y, yaw) x prediction horizon x n_vehicles
        trajectory_predictions (3, :, :)
        ref_trajectory
        n_obstacles
        obstacles
        reachable_sets
        step
        time_seconds
        vehicle_indices % vehicles to which the plot info belong
        lanelet_crossing_areas
        directed_coupling
        directed_coupling_sequential
        weighted_coupling
        directed_coupling_reduced
    end

    methods

        function obj = PlottingInfo( ...
                vehicle_indices, ...
                experiment_result, ...
                k, ...
                time_seconds, ...
                x0, ...
                optional ...
            )

            arguments
                vehicle_indices (1, :) double
                experiment_result ExperimentResult % (1, 1)
                k double % (1, 1)
                time_seconds (1, 1) double = -1
                x0 (3, :) double = zeros(3, 0) % 3 x n_vehicles
                optional.reachable_sets (:, :) cell = {} % n_vehicles x prediction_horizon
            end

            if nargin == 0
                return;
            end

            if nargin == 3
                time_seconds = k * experiment_result.options.dt_seconds;
                x0 = experiment_result.iteration_data(k).x0(vehicle_indices, 1:3)';
            end

            obj.vehicle_indices = vehicle_indices;
            obj.step = k;
            obj.time_seconds = time_seconds;
            obj.x0 = x0;
            obj.trajectory_predictions = experiment_result.get_y_predicted(k);
            obj.ref_trajectory = experiment_result.iteration_data(k).reference_trajectory_points;
            obj.n_obstacles = size(experiment_result.iteration_data(k).obstacles, 1);

            if obj.n_obstacles > 0
                obj.obstacles = experiment_result.iteration_data(k).obstacles;
            end

            obj.reachable_sets = optional.reachable_sets;

            obj.lanelet_crossing_areas = experiment_result.iteration_data(k).lanelet_crossing_areas(:);

            obj.directed_coupling = experiment_result.iteration_data(k).directed_coupling;
            obj.directed_coupling_sequential = experiment_result.iteration_data(k).directed_coupling_sequential;

            obj.weighted_coupling = experiment_result.iteration_data(k).weighted_coupling;

            obj.directed_coupling_reduced = experiment_result.iteration_data(k).directed_coupling_reduced;

        end

    end

end
