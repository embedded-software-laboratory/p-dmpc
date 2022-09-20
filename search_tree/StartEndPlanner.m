classdef StartEndPlanner < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        k = 0;
    end

    methods
        function obj = StartEndPlanner()
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
        end

        function [u, y_pred, info] = run(obj,scenario,iter)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            scenario.Hp = scenario.Hp - obj.k;
            [u, y_pred, info] ...
                = graph_search(scenario,iter);
            obj.k = obj.k + 1;
        end
    end
end