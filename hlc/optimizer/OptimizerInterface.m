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
        info = run_optimizer(obj, iter, veh_index);
    end

end
