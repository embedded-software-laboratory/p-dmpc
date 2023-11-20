classdef ConstantPrioritizer < Prioritizer
    % ConstantPrioritizer  Assign fixed priority. Defaults to priorities
    % according to vehicle ids.
    properties
        % priority array must be longer than the number of agents
        current_priorities (1, :) double = 1:50;
    end

    methods

        function obj = ConstantPrioritizer()
        end

        function [directed_coupling] = prioritize(obj, iter, ~, ~, ~)

            directed_coupling = Prioritizer.directed_coupling_from_priorities( ...
                iter.adjacency, obj.current_priorities ...
            );

        end

    end

end
