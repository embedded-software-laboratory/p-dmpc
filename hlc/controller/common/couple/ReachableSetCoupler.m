classdef ReachableSetCoupler < Coupler

    methods

        function [adjacency] = couple(~, iter)

            reachable_sets = iter.reachable_sets;
            adjacency = coupling_adjacency_reachable_sets(reachable_sets);

        end

    end

end
