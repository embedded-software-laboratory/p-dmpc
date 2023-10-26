classdef GraphSearchMexCentralized < OptimizerInterface

    methods

        function obj = GraphSearchMexCentralized(scenario, mpa)
            obj = obj@OptimizerInterface(scenario, mpa);

            graph_search_cpp_centralized_mex(CppOptimizer.InitializeWithScenario, obj.scenario, obj.mpa);
        end

        function info_v = run_optimizer(obj, iter, veh_index)
            graph_search_data = GraphSearchData(iter, obj.scenario, veh_index);
            info_v = ControlResultsInfo(iter.amount, obj.scenario.options.Hp, iter.vehicle_ids);

            [next_nodes, info_v.predicted_trims, info_v.y_predicted] = graph_search_cpp_centralized_mex(obj.scenario.options.cpp_optimizer, iter);
            info_v.tree = obj.create_tree(iter);
            info_v.tree_path = 1:(obj.scenario.options.Hp + 1);

            for i = 1:obj.scenario.options.Hp
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
