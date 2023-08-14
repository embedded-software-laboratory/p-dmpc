classdef Coupler < handle

    properties (Access = private)
        intersection_ids
        previous_intersection_ids
    end

    properties (Constant, Access = private)
        intersection_distance_threshold = 1.2; % vehicles are considered as at the intersecrion if their distances to the center point of intersection is smaller than this value
    end

    methods

        function obj = Coupler()
        end

        function [adjacency] = couple(~, iter)

            reachable_sets = iter.reachable_sets;
            adjacency = coupling_adjacency_reachable_sets(reachable_sets);

        end

        function [coupling_info] = calculate_coupling_info(obj, scenario, iter)

            adjacency = iter.adjacency;
            amount = scenario.options.amount;
            coupling_info = cell(amount, amount);

            obj.previous_intersection_ids = obj.intersection_ids;
            [obj.intersection_ids, ~] = vehicles_at_intersection(scenario, iter, obj.intersection_ids, [], obj.intersection_distance_threshold);

            for veh_i = 1:(amount - 1)

                for veh_j = (veh_i + 1):amount

                    if adjacency(veh_i, veh_j)

                        % both vehicles are at intersection
                        is_intersection = ismember([veh_i, veh_j], obj.intersection_ids);

                        [stac, distance_i, distance_j, collision_type, lanelet_relationship, is_move_side_by_side] = calculate_stac(veh_i, veh_j, scenario, iter);
                        coupling_info{veh_i, veh_j} = struct('stac', stac, 'distance', [distance_i, distance_j], 'collision_type', collision_type, ...
                            'lanelet_relationship', lanelet_relationship, 'is_intersection', is_intersection, 'is_virtual_obstacle', false, 'is_move_side_by_side', is_move_side_by_side);
                        [stac, distance_j, distance_i, collision_type, lanelet_relationship, is_move_side_by_side] = calculate_stac(veh_j, veh_i, scenario, iter);
                        coupling_info{veh_j, veh_i} = struct('stac', stac, 'distance', [distance_j, distance_i], 'collision_type', collision_type, ...
                            'lanelet_relationship', lanelet_relationship, 'is_intersection', is_intersection, 'is_virtual_obstacle', false, 'is_move_side_by_side', is_move_side_by_side);
                    end

                end

            end

        end

    end

end
