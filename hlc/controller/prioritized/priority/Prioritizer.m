classdef (Abstract) Prioritizer < handle
    % PRIORITIZER    Abstract class used for defining properties and methods used by priority based distributed controller.

    properties
        is_assign_unique_priority = false % whether to assign unique priority
    end

    methods (Abstract)
        prioritize(obj, scenario, iter)
    end

    methods (Static)

        function prioritizer = get_prioritizer(strategy)
            % GET_PRIORITIZER  Given a prioritization strategy this function returns the corresponding prioritizer.

            switch strategy
                case PriorityStrategies.coloring_priority
                    prioritizer = ColoringPrioritizer();
                case PriorityStrategies.constant_priority
                    prioritizer = ConstantPrioritizer();
                case PriorityStrategies.random_priority
                    prioritizer = RandomPrioritizer();
                case PriorityStrategies.FCA_priority
                    prioritizer = FcaPrioritizer();
                case PriorityStrategies.STAC_priority
                    prioritizer = StacPrioritizer();
                otherwise
                    prioritizer = ConstantPrioritizer();
                    warning('Unavailable Priority Strategy chosen. Using Constant Priority.');
            end

        end

    end

    methods

        function CL_list = get_CL_list(obj, CL_based_hierarchy)
            % Assign the computation level of each vehicle
            %
            % INPUT:
            %   CL_based_hierarchy: a struct contains in which computation
            %   level each vehicle is
            %
            % OUTPUT:
            %   priority_list: values of vehicles' priorities, smaller
            %   values indicate higher priorities

            nVeh = length([CL_based_hierarchy.members]);
            CL_list = zeros(1, nVeh);

            if obj.is_assign_unique_priority
                % each vehicle has unique priority
                prio = 1;

                for level_i = 1:length(CL_based_hierarchy)
                    vehs_in_level_i = CL_based_hierarchy(level_i).members; % vehicles in the selected computation level

                    for veh_i = vehs_in_level_i
                        CL_list(veh_i) = prio;
                        prio = prio + 1;
                    end

                end

            else
                % vehicles in the same computation level have the same priority
                for level_i = 1:length(CL_based_hierarchy)
                    vehs_in_level_i = CL_based_hierarchy(level_i).members;
                    CL_list(vehs_in_level_i) = level_i;
                end

            end

        end

        function priority_list = get_priority_list(~, weighted_coupling)
            [isValid, L] = kahn(weighted_coupling);
            priority_list = zeros(1, size(L, 2));

            if isValid

                for iVeh = 1:size(L, 2)
                    priority_list(iVeh) = find(L(:, iVeh));
                end

            else
                priority_list = [];
            end

        end

    end

    methods (Static)

        function directed_coupling = direct_coupling(undirected_coupling, topo_groups)
            % determine directed adjacency
            directed_coupling = undirected_coupling;
            [rows, cols] = find(undirected_coupling ~= 0);

            for k = 1:length(rows)
                v_i = rows(k);
                v_j = cols(k);

                if v_i == v_j
                    directed_coupling(v_i, v_j) = 0;
                    continue
                end

                level_i = find(topo_groups(:, v_i) == 1);
                level_j = find(topo_groups(:, v_j) == 1);
                % edge comes from vertex in the front level, ends in vertex in
                % back level
                if level_i > level_j
                    directed_coupling(v_i, v_j) = 0;
                end

            end

        end

    end

end
