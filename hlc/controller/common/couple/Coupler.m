classdef (Abstract) Coupler < handle

    properties (Access = protected)
        intersection_ids
    end

    properties (Constant, Access = protected)
        intersection_distance_threshold = 1.2; % vehicles are considered as at the intersection if their distances to the center point of intersection is smaller than this value
    end

    methods (Abstract)

        % Returns the adjacency matrix
        [adjacency] = couple(obj, options, max_mpa_speed, adjacency_lanelets, iter)

    end

    methods (Static)

        function coupler = get_coupler(coupling, amount)
            %GET_COUPLER creates a coupler according to the set option
            arguments
                coupling CouplingStrategies;
                amount double; % options.amount for constant coupling
            end

            switch (coupling)
                case CouplingStrategies.reachable_set_coupling
                    coupler = ReachableSetCoupler();
                case CouplingStrategies.full_coupling
                    coupler = ConstantCoupler(ones(amount) - eye(amount));
                case CouplingStrategies.no_coupling
                    coupler = ConstantCoupler(zeros(amount));
                case CouplingStrategies.distance_coupling
                    coupler = DistanceCoupler();
            end

        end

    end

    methods

        function obj = Coupler()
        end

        function [coupling_info] = calculate_coupling_info(obj, options, mpa, scenario, iter, time_step)
            % Calculates information going beyong the adjacency matrix like distance, stac, etc.

            adjacency = iter.adjacency;
            amount = options.amount;
            coupling_info = cell(amount, amount);

            [obj.intersection_ids, ~] = vehicles_at_intersection(time_step, obj.intersection_ids, [], obj.intersection_distance_threshold, iter.x0, scenario.intersection_center, amount);

            for veh_i = 1:(amount - 1)

                for veh_j = (veh_i + 1):amount

                    if adjacency(veh_i, veh_j)

                        % both vehicles are at intersection
                        is_intersection = ismember([veh_i, veh_j], obj.intersection_ids);

                        [stac, distance_i, distance_j, collision_type, lanelet_relationship, is_move_side_by_side] = ...
                            calculate_stac(veh_i, veh_j, options.dt_seconds, scenario, mpa, iter);

                        coupling_info{veh_i, veh_j} = struct( ...
                            'stac', stac, ...
                            'distance', [distance_i, distance_j], ...
                            'collision_type', collision_type, ...
                            'lanelet_relationship', lanelet_relationship, ...
                            'is_intersection', is_intersection, ...
                            'is_move_side_by_side', is_move_side_by_side ...
                        );

                        coupling_info{veh_j, veh_i} = coupling_info{veh_i, veh_j};

                    end

                end

            end

        end

    end

end
