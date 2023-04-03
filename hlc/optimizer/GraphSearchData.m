classdef GraphSearchData
    %GraphSearchData  Class as data structure for graph search data that changes in every iteration during the experiment.

    properties
        % size of this class has an impact on the performace/overhead of mex calls. Keep this
        % class as small as possible
        referenceTrajectoryPoints
        referenceTrajectoryIndex
        x0 % state
        trim_indices % current trim
        v_ref % reference speed
        predicted_lanelets % vehicle's predicted lanelets
        predicted_lanelet_boundary % first column for left boundary, second column for right boundary, third column for MATLAB polyshape instance % number of involved vehicles (useful for filtered)
        vehicle_obstacles
        hdv_obstacles
        lanelet_crossing_areas
        veh_idx (1, :) uint8 % indices of vehicles used in this Graph Search Iteration. Length = iter.amount
    end

    methods

        function obj = GraphSearchData(iter, scenario, veh_idx)
            obj.referenceTrajectoryPoints = iter.referenceTrajectoryPoints;
            obj.referenceTrajectoryIndex = iter.referenceTrajectoryIndex;
            obj.x0 = iter.x0;
            obj.trim_indices = iter.trim_indices;
            obj.v_ref = iter.v_ref;
            obj.predicted_lanelets = iter.predicted_lanelets;
            obj.predicted_lanelet_boundary = iter.predicted_lanelet_boundary(:, 1:2);
            obj.lanelet_crossing_areas = iter.lanelet_crossing_areas;
            [obj.vehicle_obstacles, obj.hdv_obstacles] = get_all_obstacles(iter, scenario);
            obj.veh_idx = veh_idx;
        end

    end

end
