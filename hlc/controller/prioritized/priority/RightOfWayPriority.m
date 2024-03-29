classdef RightOfWayPriority < Priority
    % right_of_way  Instance of interface_priority used for dynamic priority
    % assignment. For vehicles driving consectively, vehicles in the front are
    % aasigned higher priorities; For vehicles crossing the intersection, vehicles
    % closer to the center of intersection are assigned with higher priorities.
    % Vehicles at intersection keep higher priority and do not change their relative priority until they leave intersection

    properties

    end

    methods

        function obj = RightOfWayPriority()
            obj.is_assign_unique_priority = false; % whether to asign unique priority
        end

        %% priority
        function [veh_at_intersection, level, directed_adjacency, priority_list] = priority(obj, scenario, iter)
            % assign priorities to vehicles
            nVeh = length(iter.vehicles);
            Hp = size(iter.reference_trajectory_points, 2);
            intersection_center = [2.25, 2];
            lanelets_idx = zeros(nVeh, Hp);
            veh_at_intersection = [];
            directed_adjacency = zeros(nVeh, nVeh);
            % assign priorities to vehicles driving consecutively

            % identify vehicles at intersection and assign priorities accordingly
            for veh_idx = 1:nVeh
                lanelets_idx(veh_idx, :) = lanelets_index(veh_idx, iter, scenario);
                is_at_intersection = (sum(ismember(lanelets_idx(veh_idx, :), scenario.intersection_lanelets)) > 0);

                if is_at_intersection
                    veh_at_intersection = [veh_at_intersection, veh_idx];
                    n_veh_at_intersection = length(veh_at_intersection);
                end

            end

            adjacency = iter.adjacency;

            % assign priorities to vehicles crossing the intersection
            % (i.e., at the intersection, but not driving consecutively)
            if ~isempty(veh_at_intersection)
                % if there were vehicles at intersection at last step, remained vehicles at intersection keep priorities as last step,
                % and assign lower priorities to new vehicles coming into intersection based on the distance to the intersection center

                [~, idx, ~] = intersect(iter.last_vehs_at_intersection, veh_at_intersection);
                remained_veh_at_intersection = iter.last_vehs_at_intersection(sort(idx));

                % new incoming vehicles at intersection
                new_veh_at_intersection = setdiff(veh_at_intersection, remained_veh_at_intersection);

                % assign priorities to new vehicles at intersection based on their distance to the intersection center
                if ~isempty(new_veh_at_intersection)

                    first_referenceTrajectoryPoints = [iter.reference_trajectory_points(new_veh_at_intersection, 1, 1), iter.reference_trajectory_points(new_veh_at_intersection, 1, 2)];
                    diff = first_referenceTrajectoryPoints - intersection_center;
                    distance_to_center = sqrt(diff(:, 1).^2 + diff(:, 2).^2);
                    [~, index] = sort(distance_to_center, 'ascend');

                    % sort the vehicles at intersection based on their distance to the intersection center
                    % and assign higher priority to the vehicle closer to the intersection center
                    new_veh_at_intersection = new_veh_at_intersection(index);
                    veh_at_intersection = [remained_veh_at_intersection, new_veh_at_intersection];
                else
                    veh_at_intersection = remained_veh_at_intersection;
                end

                if n_veh_at_intersection > 1

                    for veh_idx = 1:n_veh_at_intersection - 1
                        veh_higher_prior = veh_at_intersection(veh_idx);
                        veh_adjacent = find(adjacency(veh_higher_prior, :));

                        for veh_adj_idx = (veh_idx + 1):n_veh_at_intersection
                            veh_lower_prior = veh_at_intersection(veh_adj_idx);

                            if ismember(veh_lower_prior, veh_adjacent)
                                directed_adjacency(veh_higher_prior, veh_lower_prior) = 1;
                                %                                 directed_adjacency(veh_lower_prior,veh_higher_prior) = 0;
                            end

                        end

                    end

                end

            end

            % Convert the Matrix "directed_adjacency" to a directed Graph
            Graph = digraph(directed_adjacency);

            edge_to_break = {};

            if ~isdag(Graph)
                %check if there is any cycles in the directed graph
                cycles = all_elem_cycles(Graph);

                % Convert directed cyclic graph to directed acyclic graph
                while ~isempty(cycles)
                    % break the cycle between vehicles which have the largest distance
                    cyclic_vehicles = cycles{1};
                    n_cyclic_vehicles = length(cyclic_vehicles);
                    distance = 0;

                    for vehi = 1:n_cyclic_vehicles

                        if vehi < n_cyclic_vehicles
                            distance_i = check_distance(iter, cyclic_vehicles(vehi), cyclic_vehicles(vehi + 1));
                        else
                            distance_i = check_distance(iter, cyclic_vehicles(vehi), cyclic_vehicles(1));
                        end

                        if distance_i > distance
                            distance = distance_i;
                            veh_to_break = vehi;
                        end

                    end

                    if veh_to_break < n_cyclic_vehicles
                        i = cyclic_vehicles(veh_to_break);
                        j = cyclic_vehicles(veh_to_break + 1);
                    else
                        i = cyclic_vehicles(veh_to_break);
                        j = cyclic_vehicles(1);

                    end

                    edge_to_break{end + 1} = [i, j];

                    directed_adjacency(i, j) = 0;
                    % construct a new graph and check the cycles in the graph
                    Graph = digraph(directed_adjacency);
                    cycles = all_elem_cycles(Graph);
                end

            end

            % calculate computation levels using kahn algorithm(topological ordering)
            [isDAG, level_matrix] = kahn(directed_adjacency);
            %                 % visualize the directed graph
            %                 figure(); plot(Graph,'LineWidth',1)

            assert(isDAG, 'Coupling matrix is not a DAG');

            level = computation_level_members(level_matrix);

            % Assign prrority
            % Vehicles with higher priorities plan trajectory before vehicles
            % with lower priorities
            priority_list = obj.get_priority(directed_adjacency);

        end

    end

    methods (Access = private, Static)

    end

end
