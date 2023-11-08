classdef FullyConnectedCoupler < Coupler

    methods

        function [adjacency] = couple(~, iter)
            adjacency = ones(iter.amount) - eye(iter.amount); % create matrix with 0s on the diagonal and 1s everywhere else
        end

    end

end
