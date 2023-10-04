classdef RandomWeighter < Weighter
    % RANDOMWEIGHTER  Instance of weight used for coupling weighting
    % random weight for every coupling

    properties (Access = private)
    end

    methods

        function obj = RandomWeighter()
        end

        function [weighted_coupling] = weigh(obj, ~, ~, iter)
            rand_stream = RandStream("mt19937ar", "Seed", iter.k);
            weighted_coupling = iter.directed_coupling;

            for entry = find(iter.directed_coupling)
                weighted_coupling(entry) = rand(rand_stream, 1);
            end

        end

    end

end
