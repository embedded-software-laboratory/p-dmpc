classdef (Abstract) GraphSearchMex < OptimizerInterface

    properties (Access = protected)
        mexhosts = matlab.mex.MexHost.empty;
    end

    methods

        function obj = GraphSearchMex(scenario, veh_indices)
            obj = obj@OptimizerInterface(scenario);
        end

    end

    methods (Access = protected)

        function tree = create_tree(obj, iter)
            trim = iter.trim_indices;
            x = iter.x0(:, 1);
            y = iter.x0(:, 2);
            yaw = iter.x0(:, 3);
            k = 0;
            g = 0;
            h = 0;
            tree = Tree(x, y, yaw, trim, k, g, h);
        end

    end

end
