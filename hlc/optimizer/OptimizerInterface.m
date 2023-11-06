classdef (Abstract) OptimizerInterface < handle

    properties (Access = protected)
        scenario;
        mpa;
    end

    methods

        function obj = OptimizerInterface(scenario, mpa)
            obj.scenario = scenario;
            obj.mpa = mpa;
        end

    end

    methods (Abstract)
        info = run_optimizer(obj, veh_index, iter, mpa, options);
    end

end
