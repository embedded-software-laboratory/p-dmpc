classdef (Abstract) OptimizerInterface < handle

    properties (Access = protected)
        scenario;
    end

    methods

        function obj = OptimizerInterface(scenario)
            obj.scenario = scenario;
        end

    end

    methods (Abstract)
        [info, graph_search_time] = run_optimizer(obj, iter, veh_index);
    end

end
