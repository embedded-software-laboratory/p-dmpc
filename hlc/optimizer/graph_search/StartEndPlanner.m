classdef StartEndPlanner < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        info
    end

    methods

        function obj = StartEndPlanner()
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
        end

        function info = run(obj, scenario, iter)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            % First run: Compute complete horizon
            if isempty(obj.info)
                obj.info = graph_search(scenario, iter);
                % Subsequent runs: append stillstand
            else
                obj.info.tree_path(1) = obj.info.tree_path(end);
                obj.info.tree_path = circshift(obj.info.tree_path, -1);
                obj.info.shapes(1) = obj.info.shapes(end);
                obj.info.shapes = circshift(obj.info.shapes, -1);
                obj.info.predicted_trims(1) = obj.info.predicted_trims(end);
                obj.info.predicted_trims = circshift( ...
                    obj.info.predicted_trims, ...
                    -1 ...
                );
                obj.info.y_predicted{1} ...
                    (1:(scenario.options.tick_per_step + 1), :) = ...
                    repmat( ...
                    obj.info.y_predicted{1}(end, :), ...
                    (scenario.options.tick_per_step + 1), ...
                    1 ...
                );
                obj.info.y_predicted{:} = circshift( ...
                    obj.info.y_predicted{:}, ...
                    - (scenario.options.tick_per_step + 1) ...
                );
            end

            info = obj.info;
        end

    end

end
