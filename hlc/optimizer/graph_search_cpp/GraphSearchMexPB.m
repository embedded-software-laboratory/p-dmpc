classdef GraphSearchMexPB < OptimizerInterface

    properties (Access = protected)
        mexhosts = matlab.mex.MexHost.empty;
    end

    methods

        function obj = GraphSearchMexPB(options, scenario, mpa, veh_indices)
            obj = obj@OptimizerInterface(scenario, mpa);
            % When using C++, you don't want to send the scenario over
            % and over again, so it is done in the init function
            if scenario.options.mex_out_of_process_execution
                % create mexhost for each vehicle (only if incremental search is used - no option in config yet)
                for i_veh = 1:length(veh_indices)
                    obj.mexhosts(veh_indices(i_veh)) = mexhost;
                    feval(obj.mexhosts(veh_indices(i_veh)), 'graph_search_cpp_priority_mex', CppOptimizer.InitializeWithScenario, scenario, mpa);
                end

            else
                graph_search_cpp_priority_mex(CppOptimizer.InitializeWithScenario, scenario, mpa);
            end

        end

        function info_v = run_optimizer(obj, iter, veh_index)
            graph_search_data = GraphSearchData(iter, obj.scenario, veh_index);
            info_v = ControlResultsInfo(iter.amount, obj.scenario.options.Hp, iter.vehicle_ids);
            [next_nodes, info_v.predicted_trims, info_v.y_predicted, info_v.shapes, info_v.n_expanded, info_v.is_exhausted] = obj.do_graph_search(graph_search_data);
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

        function [next_nodes, predicted_trims, y_predicted, shapes, n_expanded, is_exhausted] = do_graph_search(obj, graph_search_data)

            if obj.scenario.options.mex_out_of_process_execution
                [next_nodes, predicted_trims, y_predicted, shapes, n_expanded, is_exhausted] = ...
                    feval(obj.mexhosts(graph_search_data.veh_idx(1)), 'graph_search_cpp_priority_mex', obj.scenario.options.cpp_optimizer, graph_search_data);
            else
                [next_nodes, predicted_trims, y_predicted, shapes, n_expanded, is_exhausted] = ...
                    graph_search_cpp_priority_mex(obj.scenario.options.cpp_optimizer, graph_search_data);
            end

        end

    end

end
