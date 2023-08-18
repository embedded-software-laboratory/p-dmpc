classdef GraphSearchMexPB < GraphSearchMex

    methods

        function obj = GraphSearchMexPB(scenario, mpa, veh_indices)
            obj = obj@GraphSearchMex(scenario, mpa, veh_indices);
        end

    end

    methods

        function [info_v, graph_search_time] = run_optimizer(obj, iter, veh_index)
            graph_search_timer = tic;
            graph_search_data = GraphSearchData(iter, obj.scenario, veh_index);
            info_v = ControlResultsInfo(iter.amount, obj.scenario.options.Hp, [iter.vehicles.ID]);
            [next_nodes, info_v.predicted_trims, info_v.y_predicted, info_v.shapes, info_v.n_expanded, info_v.is_exhausted] = obj.do_graph_search(graph_search_data);
            graph_search_time = toc(graph_search_timer);
            info_v.tree = obj.create_tree(iter);
            info_v.tree_path = 1:(obj.scenario.options.Hp + 1);

            for i = 1:obj.scenario.options.Hp
                info_v.tree.add_node(i, next_nodes{i});
            end

        end

    end

    methods (Access = private)

        function [next_nodes, predicted_trims, y_predicted, shapes, n_expanded, is_exhausted] = do_graph_search(obj, graph_search_data)

            if obj.scenario.options.mex_out_of_process_execution
                [next_nodes, predicted_trims, y_predicted, shapes, n_expanded, is_exhausted] = ...
                    feval(obj.mexhosts(graph_search_data.veh_idx(1)), 'optimizer', Function.GraphSearchPBOptimal, graph_search_data);
            else
                [next_nodes, predicted_trims, y_predicted, shapes, n_expanded, is_exhausted] = ...
                    optimizer(Function.GraphSearchPBOptimal, graph_search_data);
            end

        end

    end

end
