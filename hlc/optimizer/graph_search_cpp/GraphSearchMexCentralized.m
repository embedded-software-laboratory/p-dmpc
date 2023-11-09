classdef GraphSearchMexCentralized < OptimizerInterface

    methods

        function obj = GraphSearchMexCentralized(options, mpa, scenario)
            obj = obj@OptimizerInterface();

            graph_search_cpp_centralized_mex(CppOptimizer.InitializeMex, options, mpa, scenario);
        end

        function info_v = run_optimizer(obj, ~, iter, ~, options)
            info_v = ControlResultsInfo(iter.amount, options.Hp, iter.vehicle_ids);

            [next_nodes, info_v.predicted_trims, info_v.y_predicted] = graph_search_cpp_centralized_mex(options.cpp_optimizer, iter);
            info_v.tree = obj.create_tree(iter);
            info_v.tree_path = 1:(options.Hp + 1);

            for i = 1:options.Hp
                info_v.tree.add_node(i, next_nodes{i});
            end

        end

    end

    methods (Access = private)

        function tree = create_tree(~, iter)
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
