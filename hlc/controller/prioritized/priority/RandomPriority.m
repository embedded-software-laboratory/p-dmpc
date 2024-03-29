classdef RandomPriority < Priority
    % random_priority  Instance of interface_priority used for dynamic priority
    % assignment, randomly assign priority to vehicles

    properties (Access = private)
    end

    methods

        function obj = RandomPriority()
            obj.is_assign_unique_priority = false; % whether to asign unique priority
        end

        function [level, directed_adjacency, priority_list] = priority(obj, scenario, iter)
            priority_rand_stream = RandStream("mt19937ar", "Seed", iter.k);
            directed_adjacency = iter.adjacency(:, :, end);
            nVeh = scenario.options.amount;
            RandPrio = randperm(priority_rand_stream, nVeh, nVeh);

            for iVeh = 1:nVeh

                for jVeh = 1:nVeh

                    if directed_adjacency(iVeh, jVeh) && (RandPrio(iVeh) > RandPrio(jVeh))
                        directed_adjacency(iVeh, jVeh) = 0;
                    end

                end

            end

            [isDAG, level_matrix] = kahn(directed_adjacency);

            assert(isDAG, 'Coupling matrix is not a DAG');

            level = computation_level_members(level_matrix);

            % Assign prrority according to computation level
            % Vehicles with higher priorities plan trajectory before vehicles
            % with lower priorities
            priority_list = obj.get_priority(directed_adjacency);
        end

    end

end
