classdef FullyConnectedCoupler < Coupler

    methods

        function [adjacency] = couple(~, iter)
            adjacency = ones(iter.amount) - eye(iter.amount);
        end

    end

end
