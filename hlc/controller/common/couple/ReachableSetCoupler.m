classdef ReachableSetCoupler < Coupler

    methods

        function [adjacency] = couple(~, options, ~, iter)
            % COUPLE Calculate coupling adjacency matrix
            %        Coupling check is performed by checking which
            %        reachable set overlap
            reachable_sets = iter.reachable_sets;

            adjacency = zeros(options.amount, options.amount);

            for veh_i = 1:(options.amount - 1)

                for veh_j = (veh_i + 1):options.amount

                    if InterX(reachable_sets{veh_i, end}, reachable_sets{veh_j, end})
                        adjacency(veh_i, veh_j) = 1;
                        adjacency(veh_j, veh_i) = 1;
                    end

                end

            end

        end

    end

end
