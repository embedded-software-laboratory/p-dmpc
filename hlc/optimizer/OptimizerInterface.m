classdef (Abstract) OptimizerInterface < handle

    properties (Access = protected)
    end

    methods

        function obj = OptimizerInterface()
        end

    end

    methods (Abstract)
        info = run_optimizer(obj, veh_index, iter, mpa, options);
    end

end
