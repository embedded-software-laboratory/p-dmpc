classdef ConstantPrioritizer < Prioritizer
    % constant_priority  Instance of interface_priority used for priority
    % assignment, fixed priority according to vehicle ids
    properties
        current_priorities
    end

    methods

        function obj = ConstantPrioritizer()
        end

        function set_priorities(obj, priorities)
            obj.current_priorities = priorities;
        end

        function [directed_coupling] = prioritize(obj, iter, ~, ~, ~)

            directed_coupling = Prioritizer.directed_coupling_from_priorities( ...
                iter.adjacency, obj.current_priorities ...
            );

        end

    end

end
