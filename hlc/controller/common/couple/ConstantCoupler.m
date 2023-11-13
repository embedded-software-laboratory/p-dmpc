classdef ConstantCoupler < Coupler

    properties (Access = protected)
        adjacency (:, :) double
    end

    methods

        function obj = ConstantCoupler(adjacency)
            % creates a coupler with a constant adjacency matrix
            arguments
                adjacency (:, :) double
            end

            obj = obj@Coupler();
            obj.adjacency = adjacency;
        end

        function [adjacency] = couple(obj, ~)
            adjacency = obj.adjacency;
        end

    end

end
