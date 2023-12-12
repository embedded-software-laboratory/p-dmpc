classdef PlottingInfo

    properties
        trajectory_predictions
        ref_trajectory
        n_obstacles
        n_dynamic_obstacles
        obstacles
        dynamic_obstacles
        dynamic_obstacles_shape
        reachable_sets
        step
        vehicle_indices % vehicles to which the plot info belong
        tick_now
        lanelet_crossing_areas
        weighted_coupling_reduced
        directed_coupling
        directed_coupling_sequential
        is_virtual_obstacle
    end

    methods

        function obj = PlottingInfo(vehicle_indices, experiment_result, k, tick_now)

            if nargin == 0
                return;
            end

            obj.vehicle_indices = vehicle_indices;
            obj.step = k;
            obj.tick_now = tick_now;
            obj.trajectory_predictions = experiment_result.trajectory_predictions(:, k);
            obj.ref_trajectory = experiment_result.iteration_data{k}.reference_trajectory_points;
            obj.n_obstacles = size(experiment_result.iteration_data{k}.obstacles, 1);
            obj.n_dynamic_obstacles = size(experiment_result.iteration_data{k}.dynamic_obstacle_fullres, 1);

            if obj.n_obstacles > 0
                obj.obstacles = experiment_result.iteration_data{k}.obstacles;
            end

            if obj.n_dynamic_obstacles > 0
                obj.dynamic_obstacles = experiment_result.iteration_data{k}.dynamic_obstacle_fullres{:, k};
                obj.dynamic_obstacles_shape = experiment_result.iteration_data{k}.dynamic_obstacle_shape;
            end

            obj.reachable_sets = experiment_result.iteration_data{k}.reachable_sets;

            obj.lanelet_crossing_areas = experiment_result.iteration_data{k}.lanelet_crossing_areas(:);

            obj.directed_coupling = experiment_result.iteration_data{k}.directed_coupling;
            obj.directed_coupling_sequential = experiment_result.iteration_data{k}.directed_coupling_sequential;

            obj.is_virtual_obstacle = ( ...
                experiment_result.iteration_data{k}.directed_coupling ~= ...
                experiment_result.iteration_data{k}.directed_coupling_reduced ...
            );

            obj.weighted_coupling_reduced = experiment_result.iteration_data{k}.weighted_coupling_reduced;

        end

        function obj = filter(obj, overall_amount_of_veh, plot_options)
            filter_self = false(1, overall_amount_of_veh);
            filter_self(obj.vehicle_indices(1)) = true;
            obj.trajectory_predictions = obj.trajectory_predictions{filter_self'};
            obj.ref_trajectory = obj.ref_trajectory(filter_self, :, :);

            if plot_options.plot_reachable_sets
                obj.reachable_sets = obj.reachable_sets{filter_self, :};
            end

            if plot_options.plot_lanelet_crossing_areas
                obj.lanelet_crossing_areas = obj.lanelet_crossing_areas{filter_self};
            end

        end

    end

end
