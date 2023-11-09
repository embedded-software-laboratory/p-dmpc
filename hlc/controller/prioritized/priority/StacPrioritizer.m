classdef StacPrioritizer < Prioritizer
    % STACPRIORITIZER  Instance of interface_priority used for dynamic priority
    % assignment. For vehicles driving consectively, vehicles in the front are
    % aasigned higher priorities; For vehicles crossing the intersection, vehicles
    % that can arrive at the assumed collision point earlier are assigned with higher priorities.
    % Vehicles at intersection keep higher priority and do not change their relative priority until they leave intersection
    properties (Access = private)
        intersection_entry_times
        intersection_ids
        previous_intersection_ids
    end

    properties (Constant, Access = private)
        intersection_distance_threshold = 1.2; % vehicles are considered as at the intersection if their distances to the center point of intersection is smaller than this value
    end

    methods

        function obj = StacPrioritizer()
            obj.is_assign_unique_priority = false; % whether to asign unique priority
        end

        % priority
        function [directed_coupling] = prioritize(obj, iter, time_step, options, intersection_center)
            adjacency = iter.adjacency;
            directed_coupling = adjacency;
            amount = options.amount;

            if ((time_step == 1) && ~(options.scenario_type == ScenarioType.commonroad))
                warning('STAC Priority only available for Commonroad scenarios! May behave unexpectedly!')
            end

            obj.previous_intersection_ids = obj.intersection_ids;
            [obj.intersection_ids, obj.intersection_entry_times] = vehicles_at_intersection(time_step, obj.intersection_ids, obj.intersection_entry_times, obj.intersection_distance_threshold, iter.x0, intersection_center, amount);

            for veh_i = 1:(amount - 1)

                for veh_j = (veh_i + 1):amount

                    if adjacency(veh_i, veh_j)

                        if ismember([veh_i, veh_j], obj.intersection_ids) % both vehicles are at intersection

                            if obj.intersection_entry_times(veh_i) < obj.intersection_entry_times(veh_j)
                                directed_coupling(veh_j, veh_i) = 0;
                            elseif obj.intersection_entry_times(veh_j) < obj.intersection_entry_times(veh_i)
                                directed_coupling(veh_i, veh_j) = 0;
                            else
                                % vehicle which has the priority before will
                                % continually has the priority
                                directed_coupling(veh_i, veh_j) = iter.directed_coupling(veh_i, veh_j);
                                directed_coupling(veh_j, veh_i) = iter.directed_coupling(veh_j, veh_i);
                            end

                        end

                        if sum([directed_coupling(veh_i, veh_j), directed_coupling(veh_j, veh_i)]) ~= 1
                            % if they are not coupled at previous time step, vehicle closer to the collision point is the leader

                            distance = iter.coupling_info{veh_i, veh_j}.distance;
                            distance_i = distance(1);
                            distance_j = distance(2);

                            if distance_i < distance_j
                                directed_coupling(veh_j, veh_i) = 0;
                                directed_coupling(veh_i, veh_j) = 1;
                            else
                                directed_coupling(veh_i, veh_j) = 0;
                                directed_coupling(veh_j, veh_i) = 1;
                            end

                            lanelet_relationship = iter.coupling_info{veh_i, veh_j}.lanelet_relationship;

                            if lanelet_relationship == LaneletRelationshipType.forking
                                % for forking lenelts, vehicle closer to the collision point is the follower
                                directed_coupling(veh_i, veh_j) = ~directed_coupling(veh_i, veh_j);
                                directed_coupling(veh_j, veh_i) = ~directed_coupling(veh_j, veh_i);
                            end

                        end

                    end

                end

            end

        end

    end

end
