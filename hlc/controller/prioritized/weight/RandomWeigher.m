classdef RandomWeigher < Weigher
    % RANDOMWEIGHER  Instance of weight used for coupling weighing
    % random weight for every coupling

    properties (Access = private)
    end

    methods

        function obj = RandomWeigher()
        end

        function [weighted_coupling] = weigh(~, ~, ~, iter)
            rand_stream = RandStream("mt19937ar", "Seed", iter.k);
            weighted_coupling = iter.directed_coupling;

            for entry = find(iter.directed_coupling)
                weighted_coupling(entry) = rand(rand_stream, 1);
            end

        end

    end

end
