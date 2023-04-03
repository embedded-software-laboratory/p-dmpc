classdef constant_priority < interface_priority
    % constant_priority  Instance of interface_priority used for priority
    % assignment, fixed priority according to vehicle ids

    properties (Access = private)
    end

    methods

        function obj = constant_priority()
        end

        function [level, directed_adjacency, priority_list] = priority(obj, scenario, iter)

            directed_adjacency = iter.adjacency;
            nVeh = scenario.options.amount;
            ConstPrio = 1:nVeh;

            for iVeh = 1:nVeh

                for jVeh = 1:nVeh

                    if directed_adjacency(iVeh, jVeh) && (ConstPrio(iVeh) > ConstPrio(jVeh))
                        directed_adjacency(iVeh, jVeh) = 0;
                    end

                end

            end

            [isDAG, level_matrix] = kahn(directed_adjacency);

            assert(isDAG, 'Coupling matrix is not a DAG');

            level = computation_level_members(level_matrix);

            % Assign prrority
            % Vehicles with higher priorities plan trajectory before vehicles
            % with lower priorities
            priority_list = obj.get_priority(directed_adjacency);
        end

    end

end
