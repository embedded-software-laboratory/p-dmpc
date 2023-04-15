classdef PlottingInfo
    properties
        trajectory_predictions
        trajectory_previous % trajectory of last time step
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
        coupling_weights_reduced
        directed_coupling
        belonging_vector = []
        coupling_info        
    end

    methods (Static)
        function obj = PlottingInfo(veh_indices, result, k, tick_now, plot_options)
            obj.veh_indices = veh_indices;
            obj.step = k;
            obj.tick_now = tick_now;
            obj.trajectory_predictions = result.trajectory_predictions(:,k);
            if k > 1
                obj.trajectory_previous = result.trajectory_predictions(:,k-1);
            else
                obj.trajectory_previous = []; % initial time step
            end
            obj.ref_trajectory = result.iteration_structs{k}.referenceTrajectoryPoints;
            obj.priorities = result.priority_list(:,k);
            obj.n_obstacles = size(result.obstacles,2);
            obj.n_dynamic_obstacles = size(result.iteration_structs{k}.dynamic_obstacle_fullres,1);
            if obj.n_obstacles > 0
                obj.obstacles = result.obstacles;
            end
            if obj.n_dynamic_obstacles > 0
                obj.dynamic_obstacles = result.iteration_structs{k}.dynamic_obstacle_fullres{:,k};
                obj.dynamic_obstacles_shape = result.iteration_structs{k}.dynamic_obstacle_shape;
            end
            if plot_options.plot_reachable_sets
                obj.reachable_sets = result.iteration_structs{k}.reachable_sets;
            end
            if plot_options.plot_lanelet_crossing_areaas
                obj.lanelet_crossing_areas = result.lanelet_crossing_areas{k};
            end
            obj.directed_coupling = result.directed_coupling{k};
            if ~isempty(result.iteration_structs{k}.coupling_weights_reduced)
                obj.coupling_weights_reduced = result.iteration_structs{k}.coupling_weights_reduced;
                if result.scenario.options.isPB && result.scenario.options.scenario_name == Scenario_Type.Commonroad
                    obj.belonging_vector = result.belonging_vector(:,k);
                    obj.coupling_info = result.coupling_info{k};
                end
            end
        end
    end

    methods
        function obj = filter(obj, overall_amount_of_veh, plot_options)
            filter_self = false(1,overall_amount_of_veh);
            filter_self(obj.veh_indices(1)) = true;
            obj.trajectory_predictions = obj.trajectory_predictions{filter_self'};
            obj.ref_trajectory = obj.ref_trajectory(filter_self,:,:);
            obj.priorities = obj.priorities(filter_self');
            if plot_options.plot_reachable_sets
                obj.reachable_sets = obj.reachable_sets{filter_self,:};
            end
            if plot_options.plot_lanelet_crossing_areaas
                obj.lanelet_crossing_areas = obj.lanelet_crossing_areas{filter_self};
            end
            if plot_options.plot_predicted_occupancy_previous && ~isempty(obj.trajectory_previous)
                obj.trajectory_previous = obj.trajectory_previous{filter_self};
            end
        end
    end
end
