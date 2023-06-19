classdef PlottingInfo

    properties
        trajectory_predictions
        ref_trajectory
        priorities
        n_obstacles
        n_dynamic_obstacles
        obstacles
        dynamic_obstacles
        dynamic_obstacles_shape
        reachable_sets
        step
        veh_indices % vehicles to which the plot info belong
        tick_now
        lanelet_crossing_areas
        weighted_coupling_reduced
        directed_coupling
        belonging_vector = []
        coupling_info
    end

    methods

        function obj = PlottingInfo(veh_indices, result, k, tick_now)

            if nargin == 0
                return;
            end

            obj.veh_indices = veh_indices;
            obj.step = k;
            obj.tick_now = tick_now;
            obj.trajectory_predictions = result.trajectory_predictions(:, k);
            obj.ref_trajectory = result.iteration_structs{k}.reference_trajectory_points;
            obj.priorities = result.priority_list(:, k);
            obj.n_obstacles = size(result.obstacles, 2);
            obj.n_dynamic_obstacles = size(result.iteration_structs{k}.dynamic_obstacle_fullres, 1);

            if obj.n_obstacles > 0
                obj.obstacles = result.obstacles;
            end

            if obj.n_dynamic_obstacles > 0
                obj.dynamic_obstacles = result.iteration_structs{k}.dynamic_obstacle_fullres{:, k};
                obj.dynamic_obstacles_shape = result.iteration_structs{k}.dynamic_obstacle_shape;
            end

            obj.reachable_sets = result.iteration_structs{k}.reachable_sets;

            if isfield(result, "lanelet_crossing_areas")
                obj.lanelet_crossing_areas = result.lanelet_crossing_areas{k};
            end

            obj.directed_coupling = result.directed_coupling{k};

            if ~isempty(result.iteration_structs{k}.weighted_coupling_reduced)
                obj.weighted_coupling_reduced = result.iteration_structs{k}.weighted_coupling_reduced;

                if ( ...
                        result.scenario.options.is_prioritized && ...
                        result.scenario.options.scenario_type == ScenarioType.commonroad ...
                    )
                    obj.belonging_vector = result.belonging_vector(:, k);
                    obj.coupling_info = result.coupling_info{k};
                end

            end

        end

        function obj = filter(obj, overall_amount_of_veh, plot_options)
            filter_self = false(1, overall_amount_of_veh);
            filter_self(obj.veh_indices(1)) = true;
            obj.trajectory_predictions = obj.trajectory_predictions{filter_self'};
            obj.ref_trajectory = obj.ref_trajectory(filter_self, :, :);
            obj.priorities = obj.priorities(filter_self');

            if plot_options.plot_reachable_sets
                obj.reachable_sets = obj.reachable_sets{filter_self, :};
            end

            if plot_options.plot_lanelet_crossing_areas
                obj.lanelet_crossing_areas = obj.lanelet_crossing_areas{filter_self};
            end

            obj.coupling_info = rmfield(obj.coupling_info, {'stac', 'distance', 'collision_type', 'lanelet_realtionship_type', 'is_intersection', 'is_move_side_by_side'});

        end

    end

end
