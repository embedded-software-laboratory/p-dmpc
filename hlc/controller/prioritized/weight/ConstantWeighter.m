classdef ConstantWeighter < Weighter
    % CONSTANTWEIGHTER  Instance of weight used for coupling weighting
    % fixed weight for every coupling

    properties (Access = private)
        constant_weight = 0.5
    end

    methods

        function obj = ConstantWeighter()
        end

        function [weighted_coupling] = weigh(obj, ~, iter)
            weighted_coupling = iter.directed_coupling * obj.constant_weight;
        end

    end

end
