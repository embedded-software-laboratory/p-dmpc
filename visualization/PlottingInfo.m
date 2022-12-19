classdef PlottingInfo < handle
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
        exploration
        step
        veh_ids % vehicle_ids to whcih the plot info belong
        tick_now
        lanelet_crossing_areas
        coupling_weights_reduced
        directed_coupling
        belonging_vector
        coupling_info        
    end

    methods (Static)
        function obj = PlottingInfo(veh_ids, result, k, tick_now, exploration_struct, plot_options)
            obj.veh_ids = veh_ids;
            obj.step = k;
            obj.tick_now = tick_now;
            obj.trajectory_predictions = result.trajectory_predictions(:,k);
            obj.ref_trajectory = result.iteration_structs{k}.referenceTrajectoryPoints;
            obj.priorities = result.priority(:,k);
            obj.n_obstacles = size(result.scenario.obstacles,2);
            obj.n_dynamic_obstacles = size(result.scenario.dynamic_obstacle_fullres,1);
            if obj.n_obstacles > 0
                obj.obstacles = result.scenario.obstacles;
            end
            if obj.n_dynamic_obstacles > 0
                obj.dynamic_obstacles = result.scenario.dynamic_obstacle_fullres{:,k};
                obj.dynamic_obstacles_shape = result.scenario.dynamic_obstacle_shape;
            end
            if plot_options.isShowReachableSets
                obj.reachable_sets = result.iteration_structs{k}.reachable_sets;
            end
            if plot_options.isShowLaneletCrossingAreas
                obj.lanelet_crossing_areas = result.lanelet_crossing_areas{k};
            end
            obj.directed_coupling = result.directed_coupling{k};
            if ~isempty(result.scenario.coupling_weights_reduced)
                obj.coupling_weights_reduced = result.scenario.coupling_weights_reduced;
                obj.belonging_vector = result.belonging_vector(:,k);
                obj.coupling_info = result.coupling_info{k};
            end
            obj.exploration = exploration_struct;
        end
    end
end
