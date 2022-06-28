classdef StartEndPlanner < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        u
        y_pred
        info
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
%             
%             if (isempty(obj.u))
%                 % First run
%                 [obj.u, obj.y_pred, obj.info] ...
%                     = graph_search(scenario,iter);
%             end
%             u = obj.u;
%             y_pred = {obj.y_pred{1}((obj.k-1)*41+1:end,:)};
%             info = obj.info;
%             info.tree_path = info.tree_path(obj.k:end);
%             obj.k = obj.k + 1;   

            
            scenario.Hp = scenario.Hp - obj.k;
            [u, y_pred, info] ...
                = graph_search(scenario,iter);
            obj.k = obj.k + 1;
        end
    end
end